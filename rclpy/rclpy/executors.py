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

from concurrent.futures import ThreadPoolExecutor
import inspect
import multiprocessing
from threading import Condition
from threading import Lock
from threading import RLock

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.task import Task
from rclpy.timer import WallTimer
from rclpy.utilities import get_default_context
from rclpy.utilities import timeout_sec_to_nsec
from rclpy.waitable import NumberOfEntities

# TODO(wjwwood): make _rclpy_wait(...) thread-safe
# Executor.spin_once() ends up calling _rclpy_wait(...), which right now is
# not thread-safe, no matter if different wait sets are used or not.
# See, for example, https://github.com/ros2/rclpy/issues/192
g_wait_set_spinning_lock = Lock()
g_wait_set_spinning = False


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
        self._work_condition = Condition()

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


async def await_or_execute(callback, *args):
    """Await a callback if it is a coroutine, else execute it."""
    if inspect.iscoroutinefunction(callback):
        # Await a coroutine
        return await callback(*args)
    else:
        # Call a normal function
        return callback(*args)


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

    :param context: The context to be associated with, or None for the default global context.
    """

    def __init__(self, *, context=None):
        super().__init__()
        self._context = get_default_context() if context is None else context
        self._nodes = set()
        self._nodes_lock = RLock()
        # Tasks to be executed (oldest first) 3-tuple Task, Entity, Node
        self._tasks = []
        self._tasks_lock = Lock()
        # This is triggered when wait_for_ready_callbacks should rebuild the wait list
        gc, gc_handle = _rclpy.rclpy_create_guard_condition(self._context.handle)
        self._guard_condition = gc
        self._guard_condition_handle = gc_handle
        # True if shutdown has been called
        self._is_shutdown = False
        self._work_tracker = _WorkTracker()
        # State for wait_for_ready_callbacks to reuse generator
        self._cb_iter = None
        self._last_args = None
        self._last_kwargs = None

    @property
    def context(self):
        return self._context

    def create_task(self, callback, *args, **kwargs):
        """
        Add a callback or coroutine to be executed during :meth:`spin` and return a Future.

        Arguments to this function are passed to the callback.

        :param callback: A callback to be run in the executor
        :type callback: callable or coroutine function
        :rtype: :class:`rclpy.task.Future` instance
        """
        task = Task(callback, args, kwargs, executor=self)
        with self._tasks_lock:
            self._tasks.append((task, None, None))
            _rclpy.rclpy_trigger_guard_condition(self._guard_condition)
        # Task inherits from Future
        return task

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
            if node not in self._nodes:
                self._nodes.add(node)
                node.executor = self
                # Rebuild the wait set so it includes this new node
                _rclpy.rclpy_trigger_guard_condition(self._guard_condition)
                return True
            return False

    def remove_node(self, node):
        """Stop managing this node's callbacks."""
        with self._nodes_lock:
            try:
                self._nodes.remove(node)
            except KeyError:
                pass
            else:
                # Rebuild the wait set so it doesn't include this node
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
        while self._context.ok():
            self.spin_once()

    def spin_until_future_complete(self, future):
        """Execute until a given future is done."""
        while self._context.ok() and not future.done():
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

    async def _execute_timer(self, tmr, _):
        await await_or_execute(tmr.callback)

    def _take_subscription(self, sub):
        msg = _rclpy.rclpy_take(sub.subscription_handle, sub.msg_type, sub.raw)
        return msg

    async def _execute_subscription(self, sub, msg):
        if msg:
            await await_or_execute(sub.callback, msg)

    def _take_client(self, client):
        return _rclpy.rclpy_take_response(client.client_handle, client.srv_type.Response)

    async def _execute_client(self, client, seq_and_response):
        sequence, response = seq_and_response
        if sequence is not None:
            try:
                future = client._pending_requests[sequence]
            except KeyError:
                # The request was cancelled
                pass
            else:
                future._set_executor(self)
                future.set_result(response)

    def _take_service(self, srv):
        request_and_header = _rclpy.rclpy_take_request(
            srv.service_handle, srv.srv_type.Request)
        return request_and_header

    async def _execute_service(self, srv, request_and_header):
        if request_and_header is None:
            return
        (request, header) = request_and_header
        if request:
            response = await await_or_execute(srv.callback, request, srv.srv_type.Response())
            srv.send_response(response, header)

    def _take_guard_condition(self, gc):
        gc._executor_triggered = False

    async def _execute_guard_condition(self, gc, _):
        await await_or_execute(gc.callback)

    async def _execute_waitable(self, waitable, data):
        for future in waitable._futures:
            future._set_executor(self)
        await waitable.execute(data)

    def _make_handler(self, entity, node, take_from_wait_list, call_coroutine):
        """
        Make a handler that performs work on an entity.

        :param entity: An entity to wait on
        :param take_from_wait_list: Makes the entity to stop appearing in the wait list
        :type take_from_wait_list: callable
        :param call_coroutine: Does the work the entity is ready for
        :type call_coroutine: coroutine function
        :rtype: callable
        """
        # Mark this so it doesn't get added back to the wait list
        entity._executor_event = True

        async def handler(entity, gc, is_shutdown, work_tracker):
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
                    await call_coroutine(entity, arg)
                finally:
                    entity.callback_group.ending_execution(entity)
                    # Signal that work has been done so the next callback in a mutually exclusive
                    # callback group can get executed
                    _rclpy.rclpy_trigger_guard_condition(gc)
        task = Task(
            handler, (entity, self._guard_condition, self._is_shutdown, self._work_tracker),
            executor=self)
        with self._tasks_lock:
            self._tasks.append((task, entity, node))
        return task

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
            timeout_timer = WallTimer(None, None, timeout_nsec)

        if nodes is None:
            nodes = self.get_nodes()

        yielded_work = False
        while not yielded_work and not self._is_shutdown:
            # Yield tasks in-progress before waiting for new work
            tasks = None
            with self._tasks_lock:
                tasks = list(self._tasks)
            if tasks:
                for task, entity, node in reversed(tasks):
                    if (not task.executing() and not task.done() and
                            (node is None or node in nodes)):
                        yielded_work = True
                        yield task, entity, node
                with self._tasks_lock:
                    # Get rid of any tasks that are done
                    self._tasks = list(filter(lambda t_e_n: not t_e_n[0].done(), self._tasks))

            # Gather entities that can be waited on
            subscriptions = []
            guards = []
            timers = []
            clients = []
            services = []
            waitables = []
            for node in nodes:
                subscriptions.extend(filter(self.can_execute, node.subscriptions))
                timers.extend(filter(self.can_execute, node.timers))
                clients.extend(filter(self.can_execute, node.clients))
                services.extend(filter(self.can_execute, node.services))
                node_guards = filter(self.can_execute, node.guards)
                waitables.extend(filter(self.can_execute, node.waitables))
                # retrigger a guard condition that was triggered but not handled
                for gc in node_guards:
                    if gc._executor_triggered:
                        gc.trigger()
                    guards.append(gc)
            if timeout_timer is not None:
                timers.append(timeout_timer)

            node_entity_count = NumberOfEntities(
                len(subscriptions), len(guards), len(timers), len(clients), len(services))
            executor_entity_count = NumberOfEntities(0, 2, 0, 0, 0)
            entity_count = node_entity_count + executor_entity_count
            for waitable in waitables:
                entity_count += waitable.get_num_entities()

            # Construct a wait set
            with _WaitSet() as wait_set:
                _rclpy.rclpy_wait_set_init(
                    wait_set,
                    entity_count.num_subscriptions,
                    entity_count.num_guard_conditions,
                    entity_count.num_timers,
                    entity_count.num_clients,
                    entity_count.num_services,
                    self._context.handle)

                entities = {
                    'subscription': (subscriptions, 'subscription_handle'),
                    'guard_condition': (guards, 'guard_handle'),
                    'client': (clients, 'client_handle'),
                    'service': (services, 'service_handle'),
                    'timer': (timers, 'timer_handle'),
                }
                _rclpy.rclpy_wait_set_clear_entities(wait_set)
                for entity, (handles, handle_name) in entities.items():
                    for h in handles:
                        _rclpy.rclpy_wait_set_add_entity(
                            entity, wait_set, h.__getattribute__(handle_name)
                        )
                for waitable in waitables:
                    waitable.add_to_wait_set(wait_set)
                (sigint_gc, sigint_gc_handle) = \
                    _rclpy.rclpy_get_sigint_guard_condition(self._context.handle)
                try:
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
                finally:
                    _rclpy.rclpy_destroy_entity(sigint_gc)

                # Mark all guards as triggered before yielding since they're auto-taken
                for gc in guards:
                    if gc.guard_pointer in guards_ready:
                        gc._executor_triggered = True

                # Check waitables before wait set is destroyed
                for node in nodes:
                    for wt in node.waitables:
                        # Only check waitables that were added to the wait set
                        if wt in waitables and wt.is_ready(wait_set):
                            handler = self._make_handler(
                                wt, node, lambda e: e.take_data(), self._execute_waitable)
                            yielded_work = True
                            yield handler, wt, node

            # Process ready entities one node at a time
            for node in nodes:
                for tmr in node.timers:
                    if tmr.timer_pointer in timers_ready:
                        # Check that a timer is ready to workaround rcl issue with cancelled timers
                        if _rclpy.rclpy_is_timer_ready(tmr.timer_handle):
                            if tmr.callback_group.can_execute(tmr):
                                handler = self._make_handler(
                                    tmr, node, self._take_timer, self._execute_timer)
                                yielded_work = True
                                yield handler, tmr, node

                for sub in node.subscriptions:
                    if sub.subscription_pointer in subs_ready:
                        if sub.callback_group.can_execute(sub):
                            handler = self._make_handler(
                                sub, node, self._take_subscription, self._execute_subscription)
                            yielded_work = True
                            yield handler, sub, node

                for gc in node.guards:
                    if gc._executor_triggered:
                        if gc.callback_group.can_execute(gc):
                            handler = self._make_handler(
                                gc, node, self._take_guard_condition,
                                self._execute_guard_condition)
                            yielded_work = True
                            yield handler, gc, node

                for client in node.clients:
                    if client.client_pointer in clients_ready:
                        if client.callback_group.can_execute(client):
                            handler = self._make_handler(
                                client, node, self._take_client, self._execute_client)
                            yielded_work = True
                            yield handler, client, node

                for srv in node.services:
                    if srv.service_pointer in services_ready:
                        if srv.callback_group.can_execute(srv):
                            handler = self._make_handler(
                                srv, node, self._take_service, self._execute_service)
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
        global g_wait_set_spinning_lock
        global g_wait_set_spinning
        with g_wait_set_spinning_lock:
            if g_wait_set_spinning:
                raise RuntimeError(
                    'Executor.wait_for_ready_callbacks() called concurrently in multiple threads')
            g_wait_set_spinning = True

        try:
            # if an old generator is done, this var makes the loop get a new one before returning
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
        finally:
            with g_wait_set_spinning_lock:
                g_wait_set_spinning = False


class SingleThreadedExecutor(Executor):
    """Runs callbacks in the thread which calls :func:`SingleThreadedExecutor.spin`."""

    def __init__(self, *, context=None):
        super().__init__(context=context)

    def spin_once(self, timeout_sec=None):
        try:
            handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except TimeoutException:
            pass
        else:
            handler()
            if handler.exception() is not None:
                raise handler.exception()


class MultiThreadedExecutor(Executor):
    """Runs callbacks in a pool of threads."""

    def __init__(self, num_threads=None, *, context=None):
        """
        Initialize the executor.

        :param num_threads: number of worker threads in the pool. If None the number of threads
                      will use multiprocessing.cpu_count(). If that's not implemented the number
                      of threads defaults to 1.
        :type num_threads: int
        """
        super().__init__(context=context)
        if num_threads is None:
            try:
                num_threads = multiprocessing.cpu_count()
            except NotImplementedError:
                num_threads = 1
        self._executor = ThreadPoolExecutor(num_threads)

    def spin_once(self, timeout_sec=None):
        try:
            handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except TimeoutException:
            pass
        else:
            self._executor.submit(handler)
