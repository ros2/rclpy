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
import os
from threading import Condition
from threading import Lock
from threading import RLock
import time
from types import TracebackType
from typing import Any
from typing import Callable
from typing import ContextManager
from typing import Coroutine
from typing import Generator
from typing import List
from typing import Optional
from typing import Set
from typing import Tuple
from typing import Type
from typing import TYPE_CHECKING
from typing import TypeVar
from typing import Union

import warnings

from rclpy.client import Client
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.context import Context
from rclpy.exceptions import InvalidHandle
from rclpy.guard_condition import GuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.service import Service
from rclpy.signals import SignalHandlerGuardCondition
from rclpy.subscription import Subscription
from rclpy.task import Future
from rclpy.task import Task
from rclpy.timer import Timer
from rclpy.utilities import get_default_context
from rclpy.utilities import timeout_sec_to_nsec
from rclpy.waitable import NumberOfEntities
from rclpy.waitable import Waitable

# For documentation purposes
# TODO(jacobperron): Make all entities implement the 'Waitable' interface for better type checking
WaitableEntityType = TypeVar('WaitableEntityType')

# Avoid import cycle
if TYPE_CHECKING:
    from rclpy.node import Node  # noqa: F401


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

    def wait(self, timeout_sec: Optional[float] = None):
        """
        Wait until all work completes.

        :param timeout_sec: Seconds to wait. Block forever if None or negative. Don't wait if 0
        :type timeout_sec: float or None
        :rtype: bool True if all work completed
        """
        if timeout_sec is not None and timeout_sec < 0.0:
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


class ExternalShutdownException(Exception):
    """Context has been shutdown."""

    pass


class ConditionReachedException(Exception):
    """Future has been completed."""

    pass


class TimeoutObject:
    """Use timeout object to save timeout."""

    def __init__(self, timeout: float):
        self._timeout = timeout

    @property
    def timeout(self):
        return self._timeout

    @timeout.setter
    def timeout(self, timeout):
        self._timeout = timeout


class Executor(ContextManager['Executor']):
    """
    The base class for an executor.

    An executor controls the threading model used to process callbacks. Callbacks are units of work
    like subscription callbacks, timer callbacks, service calls, and received client responses. An
    executor controls which threads callbacks get executed in.

    A custom executor must define :meth:`spin_once`.
    If the executor has any cleanup then it should also define :meth:`shutdown`.

    :param context: The context to be associated with, or ``None`` for the default global context.

    :Example:
        >>> from rclpy.executor import Executor
        >>> from rclpy.node import Node
        >>>
        >>> with Executor() as executor:
        >>>     executor.add_node(Node('example_node'))
        >>>     executor.spin_once()
        >>>     len(executor.get_nodes())
        1
    """

    def __init__(self, *, context: Optional[Context] = None) -> None:
        super().__init__()
        self._context = get_default_context() if context is None else context
        self._nodes: Set[Node] = set()
        self._nodes_lock = RLock()
        # Tasks to be executed (oldest first) 3-tuple Task, Entity, Node
        self._tasks: List[Tuple[Task, Optional[WaitableEntityType], Optional[Node]]] = []
        self._tasks_lock = Lock()
        # This is triggered when wait_for_ready_callbacks should rebuild the wait list
        self._guard = GuardCondition(
            callback=None, callback_group=None, context=self._context)
        # True if shutdown has been called
        self._is_shutdown = False
        self._work_tracker = _WorkTracker()
        # Protect against shutdown() being called in parallel in two threads
        self._shutdown_lock = Lock()
        # State for wait_for_ready_callbacks to reuse generator
        self._cb_iter = None
        self._last_args = None
        self._last_kwargs = None
        # Executor cannot use ROS clock because that requires a node
        self._clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._sigint_gc = SignalHandlerGuardCondition(context)
        self._context.on_shutdown(self.wake)

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
            self._guard.trigger()
        # Task inherits from Future
        return task

    def shutdown(self, timeout_sec: Optional[float] = None) -> bool:
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
                self._guard.trigger()
        if not self._is_shutdown:
            if not self._work_tracker.wait(timeout_sec):
                return False

        # Clean up stuff that won't be used anymore
        with self._nodes_lock:
            self._nodes = set()

        with self._shutdown_lock:
            if self._guard:
                self._guard.destroy()
                self._guard = None
            if self._sigint_gc:
                self._sigint_gc.destroy()
                self._sigint_gc = None
        self._cb_iter = None
        self._last_args = None
        self._last_kwargs = None
        return True

    def __del__(self):
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
                self._guard.trigger()
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
                self._guard.trigger()

    def wake(self) -> None:
        """
        Wake the executor because something changed.

        This is used to tell the executor when entities are created or destroyed.
        """
        if self._guard:
            self._guard.trigger()

    def get_nodes(self) -> List['Node']:
        """Return nodes that have been added to this executor."""
        with self._nodes_lock:
            return list(self._nodes)

    def spin(self) -> None:
        """Execute callbacks until shutdown."""
        while self._context.ok() and not self._is_shutdown:
            self.spin_once()

    def spin_until_future_complete(
        self,
        future: Future,
        timeout_sec: Optional[float] = None
    ) -> None:
        """Execute callbacks until a given future is done or a timeout occurs."""
        # Make sure the future wakes this executor when it is done
        future.add_done_callback(lambda x: self.wake())

        if timeout_sec is None or timeout_sec < 0:
            while self._context.ok() and not future.done() and not self._is_shutdown:
                self.spin_once_until_future_complete(future, timeout_sec)
        else:
            start = time.monotonic()
            end = start + timeout_sec
            timeout_left = TimeoutObject(timeout_sec)

            while self._context.ok() and not future.done() and not self._is_shutdown:
                self.spin_once_until_future_complete(future, timeout_left)
                now = time.monotonic()

                if now >= end:
                    return

                timeout_left.timeout = end - now

    def spin_once(self, timeout_sec: Optional[float] = None) -> None:
        """
        Wait for and execute a single callback.

        A custom executor should use :meth:`wait_for_ready_callbacks` to get work.

        This method should not be called from multiple threads.

        :param timeout_sec: Seconds to wait. Block forever if ``None`` or negative.
            Don't wait if 0.
        """
        raise NotImplementedError()

    def spin_once_until_future_complete(
        self,
        future: Future,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None
    ) -> None:
        """
        Wait for and execute a single callback.

        This should behave in the same way as :meth:`spin_once`.
        If needed by the implementation, it should awake other threads waiting.

        :param future: The executor will wait until this future is done.
        :param timeout_sec: Maximum seconds to wait. Block forever if ``None`` or negative.
            Don't wait if 0.
        """
        raise NotImplementedError()

    def _take_timer(self, tmr):
        try:
            with tmr.handle:
                tmr.handle.call_timer()

                async def _execute():
                    await await_or_execute(tmr.callback)
                return _execute
        except InvalidHandle:
            # Timer is a Destroyable, which means that on __enter__ it can throw an
            # InvalidHandle exception if the entity has already been destroyed.  Handle that here
            # by just returning an empty argument, which means we will skip doing any real work
            # in _execute_timer below
            pass

        return None

    def _take_subscription(self, sub):
        try:
            with sub.handle:
                msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
                if msg_info is None:
                    return None

                if sub._callback_type is Subscription.CallbackType.MessageOnly:
                    msg_tuple = (msg_info[0], )
                else:
                    msg_tuple = msg_info

                async def _execute():
                    await await_or_execute(sub.callback, *msg_tuple)

                return _execute
        except InvalidHandle:
            # Subscription is a Destroyable, which means that on __enter__ it can throw an
            # InvalidHandle exception if the entity has already been destroyed.  Handle that here
            # by just returning an empty argument, which means we will skip doing any real work
            # in _execute_subscription below
            pass

        return None

    def _take_client(self, client):
        try:
            with client.handle:
                header_and_response = client.handle.take_response(client.srv_type.Response)

            async def _execute():
                header, response = header_and_response
                if header is None:
                    return
                try:
                    sequence = header.request_id.sequence_number
                    future = client.get_pending_request(sequence)
                except KeyError:
                    # The request was cancelled
                    pass
                else:
                    future._set_executor(self)
                    future.set_result(response)
            return _execute

        except InvalidHandle:
            # Client is a Destroyable, which means that on __enter__ it can throw an
            # InvalidHandle exception if the entity has already been destroyed.  Handle that here
            # by just returning an empty argument, which means we will skip doing any real work
            # in _execute_client below
            pass

        return None

    def _take_service(self, srv):
        try:
            with srv.handle:
                request_and_header = srv.handle.service_take_request(srv.srv_type.Request)

            async def _execute():
                (request, header) = request_and_header
                if header is None:
                    return

                response = await await_or_execute(srv.callback, request, srv.srv_type.Response())
                srv.send_response(response, header)
            return _execute
        except InvalidHandle:
            # Service is a Destroyable, which means that on __enter__ it can throw an
            # InvalidHandle exception if the entity has already been destroyed.  Handle that here
            # by just returning an empty argument, which means we will skip doing any real work
            # in _execute_service below
            pass

        return None

    def _take_guard_condition(self, gc):
        gc._executor_triggered = False

        async def _execute():
            await await_or_execute(gc.callback)
        return _execute

    def _take_waitable(self, waitable):
        data = waitable.take_data()

        async def _execute():
            for future in waitable._futures:
                future._set_executor(self)
            await waitable.execute(data)
        return _execute

    def _make_handler(
        self,
        entity: WaitableEntityType,
        node: 'Node',
        take_from_wait_list: Callable,
    ) -> Task:
        """
        Make a handler that performs work on an entity.

        :param entity: An entity to wait on.
        :param node: The node associated with the entity.
        :param take_from_wait_list: Makes the entity to stop appearing in the wait list.
        """
        # Mark this so it doesn't get added back to the wait list
        entity._executor_event = True

        async def handler(entity, gc, is_shutdown, work_tracker):
            if is_shutdown or not entity.callback_group.beginning_execution(entity):
                # Didn't get the callback, or the executor has been ordered to stop
                entity._executor_event = False
                gc.trigger()
                return
            with work_tracker:
                # The take_from_wait_list method here is expected to return either an async def
                # method or None if there is no work to do.
                call_coroutine = take_from_wait_list(entity)

                # Signal that this has been 'taken' and can be added back to the wait list
                entity._executor_event = False
                gc.trigger()

                try:
                    if call_coroutine is not None:
                        await call_coroutine()
                finally:
                    entity.callback_group.ending_execution(entity)
                    # Signal that work has been done so the next callback in a mutually exclusive
                    # callback group can get executed

                    # Catch expected error where calling executor.shutdown()
                    # from callback causes the GuardCondition to be destroyed
                    try:
                        gc.trigger()
                    except InvalidHandle:
                        pass
        task = Task(
            handler, (entity, self._guard, self._is_shutdown, self._work_tracker),
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
        timeout_sec: Optional[Union[float, TimeoutObject]] = None,
        nodes: Optional[List['Node']] = None,
        condition: Callable[[], bool] = lambda: False,
    ) -> Generator[Tuple[Task, WaitableEntityType, 'Node'], None, None]:
        """
        Yield callbacks that are ready to be executed.

        :raise TimeoutException: on timeout.
        :raise ShutdownException: on if executor was shut down.

        :param timeout_sec: Seconds to wait. Block forever if ``None`` or negative.
            Don't wait if 0.
        :param nodes: A list of nodes to wait on. Wait on all nodes if ``None``.
        :param condition: A callable that makes the function return immediately when it evaluates
            to True.
        """
        timeout_timer = None
        timeout_nsec = timeout_sec_to_nsec(
            timeout_sec.timeout if isinstance(timeout_sec, TimeoutObject) else timeout_sec)
        if timeout_nsec > 0:
            timeout_timer = Timer(None, None, timeout_nsec, self._clock, context=self._context)

        yielded_work = False
        while not yielded_work and not self._is_shutdown and not condition():
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
            timers: List[Timer] = []
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

            guards.append(self._guard)
            guards.append(self._sigint_gc)

            entity_count = NumberOfEntities(
                len(subscriptions), len(guards), len(timers), len(clients), len(services))

            # Construct a wait set
            wait_set = None
            with ExitStack() as context_stack:
                sub_handles = []
                for sub in subscriptions:
                    try:
                        context_stack.enter_context(sub.handle)
                        sub_handles.append(sub.handle)
                    except InvalidHandle:
                        entity_count.num_subscriptions -= 1

                client_handles = []
                for cli in clients:
                    try:
                        context_stack.enter_context(cli.handle)
                        client_handles.append(cli.handle)
                    except InvalidHandle:
                        entity_count.num_clients -= 1

                service_handles = []
                for srv in services:
                    try:
                        context_stack.enter_context(srv.handle)
                        service_handles.append(srv.handle)
                    except InvalidHandle:
                        entity_count.num_services -= 1

                timer_handles = []
                for tmr in timers:
                    try:
                        context_stack.enter_context(tmr.handle)
                        timer_handles.append(tmr.handle)
                    except InvalidHandle:
                        entity_count.num_timers -= 1

                guard_handles = []
                for gc in guards:
                    try:
                        context_stack.enter_context(gc.handle)
                        guard_handles.append(gc.handle)
                    except InvalidHandle:
                        entity_count.num_guard_conditions -= 1

                for waitable in waitables:
                    try:
                        context_stack.enter_context(waitable)
                        entity_count += waitable.get_num_entities()
                    except InvalidHandle:
                        pass

                context_stack.enter_context(self._context.handle)

                wait_set = _rclpy.WaitSet(
                    entity_count.num_subscriptions,
                    entity_count.num_guard_conditions,
                    entity_count.num_timers,
                    entity_count.num_clients,
                    entity_count.num_services,
                    entity_count.num_events,
                    self._context.handle)

                wait_set.clear_entities()
                for sub_handle in sub_handles:
                    wait_set.add_subscription(sub_handle)
                for cli_handle in client_handles:
                    wait_set.add_client(cli_handle)
                for srv_capsule in service_handles:
                    wait_set.add_service(srv_capsule)
                for tmr_handle in timer_handles:
                    wait_set.add_timer(tmr_handle)
                for gc_handle in guard_handles:
                    wait_set.add_guard_condition(gc_handle)
                for waitable in waitables:
                    waitable.add_to_wait_set(wait_set)

                # Wait for something to become ready
                wait_set.wait(timeout_nsec)
                if self._is_shutdown:
                    raise ShutdownException()
                if not self._context.ok():
                    raise ExternalShutdownException()

                # get ready entities
                subs_ready = wait_set.get_ready_entities('subscription')
                guards_ready = wait_set.get_ready_entities('guard_condition')
                timers_ready = wait_set.get_ready_entities('timer')
                clients_ready = wait_set.get_ready_entities('client')
                services_ready = wait_set.get_ready_entities('service')

                # Mark all guards as triggered before yielding since they're auto-taken
                for gc in guards:
                    if gc.handle.pointer in guards_ready:
                        gc._executor_triggered = True

                # Check waitables before wait set is destroyed
                for node in nodes_to_use:
                    for wt in node.waitables:
                        # Only check waitables that were added to the wait set
                        if wt in waitables and wt.is_ready(wait_set):
                            if wt.callback_group.can_execute(wt):
                                handler = self._make_handler(wt, node, self._take_waitable)
                                yielded_work = True
                                yield handler, wt, node

            # Process ready entities one node at a time
            for node in nodes_to_use:
                for tmr in node.timers:
                    if tmr.handle.pointer in timers_ready:
                        # Check timer is ready to workaround rcl issue with cancelled timers
                        if tmr.handle.is_timer_ready():
                            if tmr.callback_group.can_execute(tmr):
                                handler = self._make_handler(tmr, node, self._take_timer)
                                yielded_work = True
                                yield handler, tmr, node

                for sub in node.subscriptions:
                    if sub.handle.pointer in subs_ready:
                        if sub.callback_group.can_execute(sub):
                            handler = self._make_handler(sub, node, self._take_subscription)
                            yielded_work = True
                            yield handler, sub, node

                for gc in node.guards:
                    if gc._executor_triggered:
                        if gc.callback_group.can_execute(gc):
                            handler = self._make_handler(gc, node, self._take_guard_condition)
                            yielded_work = True
                            yield handler, gc, node

                for client in node.clients:
                    if client.handle.pointer in clients_ready:
                        if client.callback_group.can_execute(client):
                            handler = self._make_handler(client, node, self._take_client)
                            yielded_work = True
                            yield handler, client, node

                for srv in node.services:
                    if srv.handle.pointer in services_ready:
                        if srv.callback_group.can_execute(srv):
                            handler = self._make_handler(srv, node, self._take_service)
                            yielded_work = True
                            yield handler, srv, node

            # Check timeout timer
            if (
                timeout_nsec == 0 or
                (timeout_timer is not None and timeout_timer.handle.pointer in timers_ready)
            ):
                raise TimeoutException()
        if self._is_shutdown:
            raise ShutdownException()
        if condition():
            raise ConditionReachedException()

    def wait_for_ready_callbacks(self, *args, **kwargs) -> Tuple[Task, WaitableEntityType, 'Node']:
        """
        Return callbacks that are ready to be executed.

        The arguments to this function are passed to the internal method
        :meth:`_wait_for_ready_callbacks` to get a generator for ready callbacks:

        .. Including the docstring for the hidden function for reference
        .. automethod:: _wait_for_ready_callbacks
        """
        while True:
            if self._cb_iter is None or self._last_args != args or self._last_kwargs != kwargs:
                # Create a new generator
                self._last_args = args
                self._last_kwargs = kwargs
                self._cb_iter = self._wait_for_ready_callbacks(*args, **kwargs)

            try:
                return next(self._cb_iter)
            except StopIteration:
                # Generator ran out of work
                self._cb_iter = None

    def __enter__(self) -> 'Executor':
        # Nothing to do here
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> None:
        self.shutdown()


class SingleThreadedExecutor(Executor):
    """Runs callbacks in the thread that calls :meth:`Executor.spin`."""

    def __init__(self, *, context: Optional[Context] = None) -> None:
        super().__init__(context=context)

    def _spin_once_impl(
        self,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None,
        wait_condition: Callable[[], bool] = lambda: False
    ) -> None:
        try:
            handler, entity, node = self.wait_for_ready_callbacks(
                timeout_sec, None, wait_condition)
        except ShutdownException:
            pass
        except TimeoutException:
            pass
        except ConditionReachedException:
            pass
        else:
            handler()
            if handler.exception() is not None:
                raise handler.exception()

            handler.result()  # raise any exceptions

    def spin_once(self, timeout_sec: Optional[float] = None) -> None:
        self._spin_once_impl(timeout_sec)

    def spin_once_until_future_complete(
        self,
        future: Future,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None
    ) -> None:
        future.add_done_callback(lambda x: self.wake())
        self._spin_once_impl(timeout_sec, future.done)


class MultiThreadedExecutor(Executor):
    """
    Runs callbacks in a pool of threads.

    :param num_threads: number of worker threads in the pool.
        If ``None``, the number of threads will be automatically set by querying the underlying OS
        for the CPU affinity of the process space.
        If the OS doesn't provide this information, defaults to 2.
    :param context: The context associated with the executor.
    """

    def __init__(
        self,
        num_threads: Optional[int] = None,
        *, context: Optional[Context] = None
    ) -> None:
        super().__init__(context=context)
        if num_threads is None:
            # On Linux, it will try to use the number of CPU this process has access to.
            # Other platforms, os.sched_getaffinity() doesn't exist so we use the number of CPUs.
            if hasattr(os, 'sched_getaffinity'):
                num_threads = len(os.sched_getaffinity(0))
            else:
                num_threads = os.cpu_count()
            # The calls above may still return None if they aren't supported
            if num_threads is None:
                num_threads = 2
        if num_threads == 1:
            warnings.warn(
                'MultiThreadedExecutor is used with a single thread.\n'
                'Use the SingleThreadedExecutor instead.')
        self._futures = []
        self._executor = ThreadPoolExecutor(num_threads)

    def _spin_once_impl(
        self,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None,
        wait_condition: Callable[[], bool] = lambda: False
    ) -> None:
        try:
            handler, entity, node = self.wait_for_ready_callbacks(
                timeout_sec, None, wait_condition)
        except ExternalShutdownException:
            pass
        except ShutdownException:
            pass
        except TimeoutException:
            pass
        except ConditionReachedException:
            pass
        else:
            self._executor.submit(handler)
            self._futures.append(handler)
            # make a copy of the list that we iterate over while modifying it
            # (https://stackoverflow.com/q/1207406/3753684)
            for future in self._futures[:]:
                if future.done():
                    self._futures.remove(future)
                    future.result()  # raise any exceptions

    def spin_once(self, timeout_sec: Optional[float] = None) -> None:
        self._spin_once_impl(timeout_sec)

    def spin_once_until_future_complete(
        self,
        future: Future,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None
    ) -> None:
        future.add_done_callback(lambda x: self.wake())
        self._spin_once_impl(timeout_sec, future.done)
