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
from contextlib import ExitStack
import inspect
import multiprocessing
from threading import Condition
from threading import Lock
from threading import RLock
import time
from typing import Any
from typing import Callable
from typing import Coroutine
from typing import Generator
from typing import List
from typing import Optional
from typing import Set
from typing import Tuple
from typing import TYPE_CHECKING
from typing import TypeVar
from typing import Union


from rclpy.client import Client
from rclpy.context import Context
from rclpy.guard_condition import GuardCondition
from rclpy.handle import InvalidHandle
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.service import Service
from rclpy.signals import SignalHandlerGuardCondition
from rclpy.subscription import Subscription
from rclpy.task import Future
from rclpy.task import Task
from rclpy.timer import WallTimer
from rclpy.utilities import get_default_context
from rclpy.utilities import timeout_sec_to_nsec
from rclpy.waitable import NumberOfEntities
from rclpy.waitable import Waitable

# For documentation purposes
# TODO(jacobperron): Make all entities implement the 'Waitable' interface for better type checking
WaitableEntityType = TypeVar('WaitableEntityType')

# Avoid import cycle
if TYPE_CHECKING:
    from rclpy.node import Node


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


async def await_or_execute(callback: Union[Callable, Coroutine], *args) -> Any:
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


class ShutdownException(Exception):
    """Signal that executor was shut down."""

    pass


class Executor:
    """
    The base class for an executor.

    An executor controls the threading model used to process callbacks. Callbacks are units of work
    like subscription callbacks, timer callbacks, service calls, and received client responses. An
    executor controls which threads callbacks get executed in.

    A custom executor must define :meth:`spin_once`.
    If the executor has any cleanup then it should also define :meth:`shutdown`.

    :param context: The context to be associated with, or ``None`` for the default global context.
    """

    def __init__(self, *, context: Context = None) -> None:
        super().__init__()
        self._context = get_default_context() if context is None else context
        self._nodes: Set[Node] = set()
        self._nodes_lock = RLock()
        # Tasks to be executed (oldest first) 3-tuple Task, Entity, Node
        self._tasks: List[Tuple[Task, Optional[WaitableEntityType], Optional[Node]]] = []
        self._tasks_lock = Lock()
        # This is triggered when wait_for_ready_callbacks should rebuild the wait list
        gc, gc_handle = _rclpy.rclpy_create_guard_condition(self._context.handle)
        self._guard_condition = gc
        self._guard_condition_handle = gc_handle
        # True if shutdown has been called
        self._is_shutdown = False
        self._work_tracker = _WorkTracker()
        # Protect against shutdown() being called in parallel in two threads
        self._shutdown_lock = Lock()
        # State for wait_for_ready_callbacks to reuse generator
        self._cb_iter = None
        self._last_args = None
        self._last_kwargs = None
        self._sigint_gc = SignalHandlerGuardCondition(context)

    @property
    def context(self) -> Context:
        """Get the context associated with the executor."""
        return self._context

    def create_task(self, callback: Union[Callable, Coroutine], *args, **kwargs) -> Task:
        """
        Add a callback or coroutine to be executed during :meth:`spin` and return a Future.

        Arguments to this function are passed to the callback.

        :param callback: A callback to be run in the executor.
        """
        task = Task(callback, args, kwargs, executor=self)
        with self._tasks_lock:
            self._tasks.append((task, None, None))
            _rclpy.rclpy_trigger_guard_condition(self._guard_condition)
        # Task inherits from Future
        return task

    def shutdown(self, timeout_sec: float = None) -> bool:
        """
        Stop executing callbacks and wait for their completion.

        :param timeout_sec: Seconds to wait. Block forever if ``None`` or negative.
            Don't wait if 0.
        :return: ``True`` if all outstanding callbacks finished executing, or ``False`` if the
            timeot expires before all outstanding work is done.
        """
        with self._shutdown_lock:
            if not self._is_shutdown:
                self._is_shutdown = True
                # Tell executor it's been shut down
                _rclpy.rclpy_trigger_guard_condition(self._guard_condition)

        if not self._work_tracker.wait(timeout_sec):
            return False

        # Clean up stuff that won't be used anymore
        with self._nodes_lock:
            self._nodes = set()

        with self._shutdown_lock:
            if self._guard_condition:
                _rclpy.rclpy_destroy_entity(self._guard_condition)
                self._guard_condition = None
            if self._sigint_gc:
                self._sigint_gc.destroy()
                self._sigint_gc = None
        self._cb_iter = None
        self._last_args = None
        self._last_kwargs = None
        return True

    def __del__(self):
        if self._guard_condition is not None:
            _rclpy.rclpy_destroy_entity(self._guard_condition)
        if self._sigint_gc is not None:
            self._sigint_gc.destroy()

    def add_node(self, node: 'Node') -> bool:
        """
        Add a node whose callbacks should be managed by this executor.

        :param node: The node to add to the executor.
        :return: ``True`` if the node was added, ``False`` otherwise.
        """
        with self._nodes_lock:
            if node not in self._nodes:
                self._nodes.add(node)
                node.executor = self
                # Rebuild the wait set so it includes this new node
                _rclpy.rclpy_trigger_guard_condition(self._guard_condition)
                return True
            return False

    def remove_node(self, node: 'Node') -> None:
        """
        Stop managing this node's callbacks.

        :param node: The node to remove from the executor.
        """
        with self._nodes_lock:
            try:
                self._nodes.remove(node)
            except KeyError:
                pass
            else:
                # Rebuild the wait set so it doesn't include this node
                _rclpy.rclpy_trigger_guard_condition(self._guard_condition)

    def get_nodes(self) -> List['Node']:
        """Return nodes that have been added to this executor."""
        with self._nodes_lock:
            return list(self._nodes)

    def spin(self) -> None:
        """Execute callbacks until shutdown."""
        while self._context.ok():
            self.spin_once()

    def spin_until_future_complete(self, future: Future, timeout_sec: float = None) -> None:
        """Execute callbacks until a given future is done or a timeout occurs."""
        if timeout_sec is None or timeout_sec < 0:
            while self._context.ok() and not future.done():
                self.spin_once(timeout_sec=timeout_sec)
        else:
            start = time.monotonic()
            end = start + timeout_sec
            timeout_left = timeout_sec_to_nsec(timeout_sec)

            while self._context.ok() and not future.done():
                self.spin_once(timeout_sec=timeout_left)
                now = time.monotonic()

                if now >= end:
                    return

                timeout_left = end - now

    def spin_once(self, timeout_sec: float = None) -> None:
        """
        Wait for and execute a single callback.

        A custom executor should use :meth:`wait_for_ready_callbacks` to get work.

        :param timeout_sec: Seconds to wait. Block forever if ``None`` or negative.
            Don't wait if 0.
        """
        raise NotImplementedError

    def _take_timer(self, tmr):
        with tmr.handle as capsule:
            _rclpy.rclpy_call_timer(capsule)

    async def _execute_timer(self, tmr, _):
        await await_or_execute(tmr.callback)

    def _take_subscription(self, sub):
        with sub.handle as capsule:
            msg = _rclpy.rclpy_take(capsule, sub.msg_type, sub.raw)
        return msg

    async def _execute_subscription(self, sub, msg):
        if msg:
            await await_or_execute(sub.callback, msg)

    def _take_client(self, client):
        with client.handle as capsule:
            return _rclpy.rclpy_take_response(capsule, client.srv_type.Response)

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
        with srv.handle as capsule:
            request_and_header = _rclpy.rclpy_take_request(capsule, srv.srv_type.Request)
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

    def _make_handler(
        self,
        entity: WaitableEntityType,
        node: 'Node',
        take_from_wait_list: Callable,
        call_coroutine: Coroutine
    ) -> Task:
        """
        Make a handler that performs work on an entity.

        :param entity: An entity to wait on.
        :param node: The node associated with the entity.
        :param take_from_wait_list: Makes the entity to stop appearing in the wait list.
        :param call_coroutine: Does the work the entity is ready for
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

    def can_execute(self, entity: WaitableEntityType) -> bool:
        """
        Determine if a callback for an entity can be executed.

        :param entity: Subscription, Timer, Guard condition, etc
        :returns: ``True`` if the entity callback can be executed, ``False`` otherwise.
        """
        return not entity._executor_event and entity.callback_group.can_execute(entity)

    def _wait_for_ready_callbacks(
        self,
        timeout_sec: float = None,
        nodes: List['Node'] = None
    ) -> Generator[Tuple[Task, WaitableEntityType, 'Node'], None, None]:
        """
        Yield callbacks that are ready to be executed.

        :raise TimeoutException: on timeout.
        :raise ShutdownException: on if executor was shut down.

        :param timeout_sec: Seconds to wait. Block forever if ``None`` or negative.
            Don't wait if 0.
        :param nodes: A list of nodes to wait on. Wait on all nodes if ``None``.
        """
        timeout_timer = None
        timeout_nsec = timeout_sec_to_nsec(timeout_sec)
        if timeout_nsec > 0:
            timeout_timer = WallTimer(None, None, timeout_nsec)

        yielded_work = False
        while not yielded_work and not self._is_shutdown:
            # Refresh "all" nodes in case executor was woken by a node being added or removed
            nodes_to_use = nodes
            if nodes is None:
                nodes_to_use = self.get_nodes()

            # Yield tasks in-progress before waiting for new work
            tasks = None
            with self._tasks_lock:
                tasks = list(self._tasks)
            if tasks:
                for task, entity, node in reversed(tasks):
                    if (not task.executing() and not task.done() and
                            (node is None or node in nodes_to_use)):
                        yielded_work = True
                        yield task, entity, node
                with self._tasks_lock:
                    # Get rid of any tasks that are done
                    self._tasks = list(filter(lambda t_e_n: not t_e_n[0].done(), self._tasks))

            # Gather entities that can be waited on
            subscriptions: List[Subscription] = []
            guards: List[GuardCondition] = []
            timers: List[WallTimer] = []
            clients: List[Client] = []
            services: List[Service] = []
            waitables: List[Waitable] = []
            for node in nodes_to_use:
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
            with _WaitSet() as wait_set, ExitStack() as context_stack:
                sub_capsules = []
                for sub in subscriptions:
                    try:
                        sub_capsules.append(context_stack.enter_context(sub.handle))
                    except InvalidHandle:
                        entity_count.num_subscriptions -= 1

                client_capsules = []
                for cli in clients:
                    try:
                        client_capsules.append(context_stack.enter_context(cli.handle))
                    except InvalidHandle:
                        entity_count.num_clients -= 1

                service_capsules = []
                for srv in services:
                    try:
                        service_capsules.append(context_stack.enter_context(srv.handle))
                    except InvalidHandle:
                        entity_count.num_services -= 1

                timer_capsules = []
                for tmr in timers:
                    try:
                        timer_capsules.append(context_stack.enter_context(tmr.handle))
                    except InvalidHandle:
                        entity_count.num_timers -= 1

                _rclpy.rclpy_wait_set_init(
                    wait_set,
                    entity_count.num_subscriptions,
                    entity_count.num_guard_conditions,
                    entity_count.num_timers,
                    entity_count.num_clients,
                    entity_count.num_services,
                    self._context.handle)

                entities = {
                    'guard_condition': (guards, 'guard_handle'),
                }
                _rclpy.rclpy_wait_set_clear_entities(wait_set)
                for entity, (handles, handle_name) in entities.items():
                    for h in handles:
                        _rclpy.rclpy_wait_set_add_entity(
                            entity, wait_set, h.__getattribute__(handle_name)
                        )
                for sub_capsule in sub_capsules:
                    _rclpy.rclpy_wait_set_add_entity('subscription', wait_set, sub_capsule)
                for cli_capsule in client_capsules:
                    _rclpy.rclpy_wait_set_add_entity('client', wait_set, cli_capsule)
                for srv_capsule in service_capsules:
                    _rclpy.rclpy_wait_set_add_entity('service', wait_set, srv_capsule)
                for tmr_capsule in timer_capsules:
                    _rclpy.rclpy_wait_set_add_entity('timer', wait_set, tmr_capsule)
                for waitable in waitables:
                    waitable.add_to_wait_set(wait_set)

                sigint_gc = self._sigint_gc.guard_handle
                _rclpy.rclpy_wait_set_add_entity('guard_condition', wait_set, sigint_gc)
                _rclpy.rclpy_wait_set_add_entity(
                    'guard_condition', wait_set, self._guard_condition)

                # Wait for something to become ready
                _rclpy.rclpy_wait(wait_set, timeout_nsec)
                if self._is_shutdown:
                    raise ShutdownException()

                # get ready entities
                subs_ready = _rclpy.rclpy_get_ready_entities('subscription', wait_set)
                guards_ready = _rclpy.rclpy_get_ready_entities('guard_condition', wait_set)
                timers_ready = _rclpy.rclpy_get_ready_entities('timer', wait_set)
                clients_ready = _rclpy.rclpy_get_ready_entities('client', wait_set)
                services_ready = _rclpy.rclpy_get_ready_entities('service', wait_set)

                # Mark all guards as triggered before yielding since they're auto-taken
                for gc in guards:
                    if gc.guard_pointer in guards_ready:
                        gc._executor_triggered = True

                # Check waitables before wait set is destroyed
                for node in nodes_to_use:
                    for wt in node.waitables:
                        # Only check waitables that were added to the wait set
                        if wt in waitables and wt.is_ready(wait_set):
                            handler = self._make_handler(
                                wt, node, lambda e: e.take_data(), self._execute_waitable)
                            yielded_work = True
                            yield handler, wt, node

            # Process ready entities one node at a time
            for node in nodes_to_use:
                for tmr in node.timers:
                    if tmr.handle.pointer in timers_ready:
                        with tmr.handle as capsule:
                            # Check timer is ready to workaround rcl issue with cancelled timers
                            if _rclpy.rclpy_is_timer_ready(capsule):
                                if tmr.callback_group.can_execute(tmr):
                                    handler = self._make_handler(
                                        tmr, node, self._take_timer, self._execute_timer)
                                    yielded_work = True
                                    yield handler, tmr, node

                for sub in node.subscriptions:
                    if sub.handle.pointer in subs_ready:
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
                    if client.handle.pointer in clients_ready:
                        if client.callback_group.can_execute(client):
                            handler = self._make_handler(
                                client, node, self._take_client, self._execute_client)
                            yielded_work = True
                            yield handler, client, node

                for srv in node.services:
                    if srv.handle.pointer in services_ready:
                        if srv.callback_group.can_execute(srv):
                            handler = self._make_handler(
                                srv, node, self._take_service, self._execute_service)
                            yielded_work = True
                            yield handler, srv, node

            # Check timeout timer
            if (
                timeout_nsec == 0 or
                (timeout_timer is not None and timeout_timer.handle.pointer in timers_ready)
            ):
                raise TimeoutException()

    def wait_for_ready_callbacks(self, *args, **kwargs) -> Tuple[Task, WaitableEntityType, 'Node']:
        """
        Return callbacks that are ready to be executed.

        The arguments to this function are passed to the internal method
        :meth:`_wait_for_ready_callbacks` to get a generator for ready callbacks:

        .. Including the docstring for the hidden function for reference
        .. automethod:: _wait_for_ready_callbacks
        """
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


class SingleThreadedExecutor(Executor):
    """Runs callbacks in the thread that calls :meth:`Executor.spin`."""

    def __init__(self, *, context: Context = None) -> None:
        super().__init__(context=context)

    def spin_once(self, timeout_sec: float = None) -> None:
        try:
            handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except ShutdownException:
            pass
        except TimeoutException:
            pass
        else:
            handler()
            if handler.exception() is not None:
                raise handler.exception()


class MultiThreadedExecutor(Executor):
    """
    Runs callbacks in a pool of threads.

    :param num_threads: number of worker threads in the pool. If ``None``, the number of threads
        will use :func:`multiprocessing.cpu_count`. If that's not implemented the number of threads
        defaults to 1.
    :param context: The context associated with the executor.
    """

    def __init__(self, num_threads: int = None, *, context: Context = None) -> None:
        super().__init__(context=context)
        if num_threads is None:
            try:
                num_threads = multiprocessing.cpu_count()
            except NotImplementedError:
                num_threads = 1
        self._executor = ThreadPoolExecutor(num_threads)

    def spin_once(self, timeout_sec: float = None) -> None:
        try:
            handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except ShutdownException:
            pass
        except TimeoutException:
            pass
        else:
            self._executor.submit(handler)
