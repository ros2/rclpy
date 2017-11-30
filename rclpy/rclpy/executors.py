# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from concurrent.futures import ThreadPoolExecutor as _ThreadPoolExecutor
import multiprocessing
from threading import Condition as _Condition
from threading import Lock as _Lock

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.timer import WallTimer as _WallTimer
from rclpy.utilities import ok
from rclpy.utilities import timeout_sec_to_nsec


class _WaitSet:
    """Make sure the wait set gets destroyed when a generator exits."""

    def __enter__(self):
        self.wait_set = _rclpy.rclpy_get_zero_initialized_wait_set()
        return self.wait_set

    def __exit__(self, t, v, tb):
        _rclpy.rclpy_destroy_wait_set(self.wait_set)


class _WorkTracker:
    """Track the amount of work that is in progress."""

    def __init__(self):
        # Number of tasks that are being executed
        self._num_work_executing = 0
        self._work_condition = _Condition()

    def __enter__(self):
        """Increment the amount of executing work by 1."""
        with self._work_condition:
            self._num_work_executing += 1

    def __exit__(self, t, v, tb):
        """Decrement the amount of work executing by 1."""
        with self._work_condition:
            self._num_work_executing -= 1
            self._work_condition.notify_all()

    def wait(self, timeout_sec=None):
        """
        Wait until all work completes.

        :param timeout_sec: Seconds to wait. Block forever if None or negative. Don't wait if 0
        :type timeout_sec: float or None
        :rtype: bool True if all work completed
        """
        if timeout_sec is not None and timeout_sec < 0:
            timeout_sec = None
        # Wait for all work to complete
        with self._work_condition:
            if not self._work_condition.wait_for(
                    lambda: self._num_work_executing == 0, timeout_sec):
                return False
        return True


class TimeoutException(Exception):
    """Signal that a timeout occurred."""

    pass


class Executor:
    """
    A base class for an executor.

    An executor controls the threading model used to process callbacks. Callbacks are units of work
    like subscription callbacks, timer callbacks, service calls, and received client responses. An
    executor controls which threads callbacks get executed in.

    A custom executor must define :func:`Executor.spin_once`. If the executor has any cleanup then
    it should also define :func:`Executor.shutdown`.
    """

    def __init__(self):
        super().__init__()
        self._nodes = set()
        self._nodes_lock = _Lock()
        # This is triggered when wait_for_ready_callbacks should rebuild the wait list
        gc, gc_handle = _rclpy.rclpy_create_guard_condition()
        self._guard_condition = gc
        self._guard_condition_handle = gc_handle
        # True if shutdown has been called
        self._is_shutdown = False
        self._work_tracker = _WorkTracker()
        # State for wait_for_ready_callbacks to reuse generator
        self._cb_iter = None
        self._last_args = None
        self._last_kwargs = None

    def shutdown(self, timeout_sec=None):
        """
        Stop executing callbacks and wait for their completion.

        Return true if all outstanding callbacks finished executing.

        :param timeout_sec: Seconds to wait. Block forever if None or negative. Don't wait if 0
        :type timeout_sec: float or None
        :rtype: bool
        """
        self._is_shutdown = True
        if not self._work_tracker.wait(timeout_sec):
            return False
        # Clean up stuff that won't be used anymore
        with self._nodes_lock:
            self._nodes = set()
        _rclpy.rclpy_destroy_entity(self._guard_condition)

        self._guard_condition = None
        self._cb_iter = None
        self._last_args = None
        self._last_kwargs = None
        return True

    def __del__(self):
        if self._guard_condition is not None:
            _rclpy.rclpy_destroy_entity(self._guard_condition)

    def add_node(self, node):
        """
        Add a node whose callbacks should be managed by this executor.

        Return true if the node was added.

        :rtype: bool
        """
        with self._nodes_lock:
            self._nodes.add(node)
            # Rebuild the wait set so it includes this new node
            _rclpy.rclpy_trigger_guard_condition(self._guard_condition)

    def get_nodes(self):
        """
        Return nodes which have been added to this executor.

        :rtype: list
        """
        with self._nodes_lock:
            return list(self._nodes)

    def spin(self):
        """Execute callbacks until shutdown."""
        while ok():
            self.spin_once()

    def spin_once(self, timeout_sec=None):
        """
        Wait for and execute a single callback.

        A custom executor should use :func:`Executor.wait_for_ready_callbacks` to get work.

        :param timeout_sec: Seconds to wait. Block forever if None or negative. Don't wait if 0
        :type timeout_sec: float or None
        :rtype: None
        """
        raise NotImplementedError

    def _take_timer(self, tmr):
        _rclpy.rclpy_call_timer(tmr.timer_handle)

    def _execute_timer(self, tmr, _):
        tmr.callback()

    def _take_subscription(self, sub):
        msg = _rclpy.rclpy_take(sub.subscription_handle, sub.msg_type)
        return msg

    def _execute_subscription(self, sub, msg):
        if msg:
            sub.callback(msg)

    def _take_client(self, client):
        response = _rclpy.rclpy_take_response(
            client.client_handle, client.srv_type.Response, client.sequence_number)
        return response

    def _execute_client(self, client, response):
        if response:
            # clients spawn their own thread to wait for a response in the
            # wait_for_future function. Users can either use this mechanism or monitor
            # the content of client.response to check if a response has been received
            client.response = response

    def _take_service(self, srv):
        request_and_header = _rclpy.rclpy_take_request(
            srv.service_handle, srv.srv_type.Request)
        return request_and_header

    def _execute_service(self, srv, request_and_header):
        if request_and_header is None:
            return
        (request, header) = request_and_header
        if request:
            response = srv.callback(request, srv.srv_type.Response())
            srv.send_response(response, header)

    def _take_guard_condition(self, gc):
        gc._executor_triggered = False

    def _execute_guard_condition(self, gc, _):
        gc.callback()

    def _make_handler(self, entity, take_from_wait_list, call_callback):
        """
        Make a handler that performs work on an entity.

        :param entity: An entity to wait on
        :param take_from_wait_list: Makes the entity to stop appearing in the wait list
        :type take_from_wait_list: callable
        :param call_callback: Does the work the entity is ready for
        :type call_callback: callable
        :rtype: callable
        """
        gc = self._guard_condition
        work_tracker = self._work_tracker
        is_shutdown = self._is_shutdown
        # Mark this so it doesn't get added back to the wait list
        entity._executor_event = True

        def handler():
            nonlocal entity
            nonlocal gc
            nonlocal is_shutdown
            nonlocal work_tracker
            if is_shutdown or not entity.callback_group.beginning_execution(entity):
                # Didn't get the callback, or the executor has been ordered to stop
                entity._executor_event = False
                _rclpy.rclpy_trigger_guard_condition(gc)
                return
            with work_tracker:
                arg = take_from_wait_list(entity)

                # Signal that this has been 'taken' and can be added back to the wait list
                entity._executor_event = False
                _rclpy.rclpy_trigger_guard_condition(gc)

                try:
                    call_callback(entity, arg)
                finally:
                    entity.callback_group.ending_execution(entity)
                    # Signal that work has been done so the next callback in a mutually exclusive
                    # callback group can get executed
                    _rclpy.rclpy_trigger_guard_condition(gc)
        return handler

    def can_execute(self, entity):
        """
        Determine if a callback for an entity can be executed.

        :param entity: Subscription, Timer, Guard condition, etc
        :returns: True if the entity callback can be executed
        :rtype: bool
        """
        return not entity._executor_event and entity.callback_group.can_execute(entity)

    def _wait_for_ready_callbacks(self, timeout_sec=None, nodes=None):
        """
        Yield callbacks that are ready to be performed.

        Raises :class:`TimeoutException` on timeout.

        :param timeout_sec: Seconds to wait. Block forever if None or negative. Don't wait if 0
        :type timeout_sec: float or None
        :param nodes: A list of nodes to wait on. Wait on all nodes if None.
        :type nodes: list or None
        :rtype: Generator[(callable, entity, :class:`rclpy.node.Node`)]
        """
        timeout_timer = None
        timeout_nsec = timeout_sec_to_nsec(timeout_sec)
        if timeout_nsec > 0:
            timeout_timer = _WallTimer(None, None, timeout_nsec)

        if nodes is None:
            nodes = self.get_nodes()

        yielded_work = False
        while not yielded_work and not self._is_shutdown:
            # Gather entities that can be waited on
            subscriptions = []
            guards = []
            timers = []
            clients = []
            services = []
            for node in nodes:
                subscriptions.extend(filter(self.can_execute, node.subscriptions))
                timers.extend(filter(self.can_execute, node.timers))
                clients.extend(filter(self.can_execute, node.clients))
                services.extend(filter(self.can_execute, node.services))
                node_guards = filter(self.can_execute, node.guards)
                # retrigger a guard condition that was triggered but not handled
                for gc in node_guards:
                    if gc._executor_triggered:
                        gc.trigger()
                    guards.append(gc)
            (sigint_gc, sigint_gc_handle) = _rclpy.rclpy_get_sigint_guard_condition()
            if timeout_timer is not None:
                timers.append(timeout_timer)

            # Construct a wait set
            with _WaitSet() as wait_set:
                _rclpy.rclpy_wait_set_init(
                    wait_set,
                    len(subscriptions),
                    len(guards) + 2,
                    len(timers),
                    len(clients),
                    len(services))

                entities = {
                    'subscription': (subscriptions, 'subscription_handle'),
                    'guard_condition': (guards, 'guard_handle'),
                    'client': (clients, 'client_handle'),
                    'service': (services, 'service_handle'),
                    'timer': (timers, 'timer_handle'),
                }
                for entity, (handles, handle_name) in entities.items():
                    _rclpy.rclpy_wait_set_clear_entities(entity, wait_set)
                    for h in handles:
                        _rclpy.rclpy_wait_set_add_entity(
                            entity, wait_set, h.__getattribute__(handle_name)
                        )
                _rclpy.rclpy_wait_set_add_entity('guard_condition', wait_set, sigint_gc)
                _rclpy.rclpy_wait_set_add_entity(
                    'guard_condition', wait_set, self._guard_condition)

                # Wait for something to become ready
                _rclpy.rclpy_wait(wait_set, timeout_nsec)

                # get ready entities
                subs_ready = _rclpy.rclpy_get_ready_entities('subscription', wait_set)
                guards_ready = _rclpy.rclpy_get_ready_entities('guard_condition', wait_set)
                timers_ready = _rclpy.rclpy_get_ready_entities('timer', wait_set)
                clients_ready = _rclpy.rclpy_get_ready_entities('client', wait_set)
                services_ready = _rclpy.rclpy_get_ready_entities('service', wait_set)

            # Check sigint guard condition
            if sigint_gc_handle in guards_ready:
                raise KeyboardInterrupt
            _rclpy.rclpy_destroy_entity(sigint_gc)

            # Mark all guards as triggered before yielding any handlers since they're auto-taken
            for gc in guards:
                if gc.guard_pointer in guards_ready:
                    gc._executor_triggered = True

            # Process ready entities one node at a time
            for node in nodes:
                for tmr in node.timers:
                    if tmr.timer_pointer in timers_ready:
                        # Check that a timer is ready to workaround rcl issue with cancelled timers
                        if _rclpy.rclpy_is_timer_ready(tmr.timer_handle):
                            if tmr.callback_group.can_execute(tmr):
                                handler = self._make_handler(
                                    tmr, self._take_timer, self._execute_timer)
                                yielded_work = True
                                yield handler, tmr, node

                for sub in node.subscriptions:
                    if sub.subscription_pointer in subs_ready:
                        if sub.callback_group.can_execute(sub):
                            handler = self._make_handler(
                                sub, self._take_subscription, self._execute_subscription)
                            yielded_work = True
                            yield handler, sub, node

                for gc in node.guards:
                    if gc._executor_triggered:
                        if gc.callback_group.can_execute(gc):
                            handler = self._make_handler(
                                gc, self._take_guard_condition, self._execute_guard_condition)
                            yielded_work = True
                            yield handler, gc, node

                for client in node.clients:
                    if client.client_pointer in clients_ready:
                        if client.callback_group.can_execute(client):
                            handler = self._make_handler(
                                client, self._take_client, self._execute_client)
                            yielded_work = True
                            yield handler, client, node

                for srv in node.services:
                    if srv.service_pointer in services_ready:
                        if srv.callback_group.can_execute(srv):
                            handler = self._make_handler(
                                srv, self._take_service, self._execute_service)
                            yielded_work = True
                            yield handler, srv, node

            # Check timeout timer
            if (
                timeout_nsec == 0 or
                (timeout_timer is not None and timeout_timer.timer_pointer in timers_ready)
            ):
                raise TimeoutException()

    def wait_for_ready_callbacks(self, *args, **kwargs):
        """
        Reuse generator and return callbacks that are ready to be performed.

        See :func:`Executor._wait_for_ready_callbacks` for documentation
        """
        # if an old generator is done, this variable makes the loop get a new one before returning
        got_generator = False
        while not got_generator:
            if self._cb_iter is None or self._last_args != args or self._last_kwargs != kwargs:
                # Create a new generator
                self._last_args = args
                self._last_kwargs = kwargs
                self._cb_iter = self._wait_for_ready_callbacks(*args, **kwargs)
                got_generator = True

            try:
                return next(self._cb_iter)
            except StopIteration:
                # Generator ran out of work
                self._cb_iter = None


class SingleThreadedExecutor(Executor):
    """Runs callbacks in the thread which calls :func:`SingleThreadedExecutor.spin`."""

    def spin_once(self, timeout_sec=None):
        try:
            handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except TimeoutException:
            pass
        else:
            handler()


class MultiThreadedExecutor(Executor):
    """Runs callbacks in a pool of threads."""

    def __init__(self, num_threads=None):
        """
        Initialize the executor.

        :param num_threads: number of worker threads in the pool. If None the number of threads
                      will use multiprocessing.cpu_count(). If that's not implemented the number
                      of threads defaults to 1.
        :type num_threads: int
        """
        super().__init__()
        if num_threads is None:
            try:
                num_threads = multiprocessing.cpu_count()
            except NotImplementedError:
                num_threads = 1
        self._executor = _ThreadPoolExecutor(num_threads)

    def spin_once(self, timeout_sec=None):
        try:
            handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except TimeoutException:
            pass
        else:
            self._executor.submit(handler)
