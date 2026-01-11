# Copyright 2025 Nadav Elkabets
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

import asyncio
from functools import partial
from sys import stderr
import traceback
from typing import Callable
from typing import Coroutine
from typing import Dict
from typing import List
from typing import Optional
from typing import Set
from typing import TypeVar
from typing import Union
import warnings

from rclpy.client import Client
from rclpy.clock import Clock, ClockChange, JumpHandle, JumpThreshold, ROSClock, TimeJump
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import BaseExecutor
from rclpy.node import Node
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.time import Time
from rclpy.timer import Timer
from rclpy.utilities import get_default_context

EntityT = TypeVar('EntityT', bound=Union[Subscription, Service, Client])

S_TO_NS = 1e9


class _WaitHandler:
    """
    Generic handler for time-based waiting that handles both wall-clock and ROS time.

    When ROS time is active, relies on jump callbacks. When ROS time is inactive,
    uses asyncio's call_later() for scheduling.
    """

    def __init__(
        self,
        clock: Clock,
        loop: asyncio.AbstractEventLoop,
        is_finished: Callable[[], bool],
        is_ready: Callable[[], bool],
        time_until_ready_sec: Callable[[], float],
        on_ready: Callable[[], None],
        on_clock_change: Optional[Callable[[ClockChange], None]] = None,
    ) -> None:
        self._clock = clock
        self._loop = loop
        self._is_finished = is_finished
        self._is_ready = is_ready
        self._time_until_ready_sec = time_until_ready_sec
        self._on_ready = on_ready
        self._on_clock_change = on_clock_change
        self._call_later: Optional[asyncio.TimerHandle] = None
        self._jump_handle: Optional[JumpHandle] = None
        self._register_jump_handle()
        self._process()

    def cancel(self) -> None:
        """Cancel the wait handler and clean up resources."""
        if self._call_later:
            self._call_later.cancel()
            self._call_later = None
        if self._jump_handle:
            self._jump_handle.unregister()
            self._jump_handle = None

    def _process(self) -> None:
        """Check if ready and schedule next check if needed."""
        if self._is_finished():
            self.cancel()
            return
        if self._is_ready():
            self._on_ready()
            if not self._is_finished() and not self._ros_time_active():
                self._schedule()
        elif not self._ros_time_active():
            self._schedule()

    def _schedule(self) -> None:
        """Schedule the next wall-clock check."""
        if self._call_later:
            self._call_later.cancel()
        delay = max(self._time_until_ready_sec(), 0.0)
        self._call_later = self._loop.call_later(delay, self._process)

    def _register_jump_handle(self) -> None:
        """Register for ROS time jump callbacks."""
        if not isinstance(self._clock, ROSClock):
            return
        threshold = JumpThreshold(min_forward=Duration(nanoseconds=1), min_backward=None)
        self._jump_handle = self._clock.create_jump_callback(
            threshold, post_callback=self._on_jump
        )

    def _on_jump(self, jump: TimeJump) -> None:
        """Handle ROS time jumps. Called from clock thread, dispatches to event loop."""
        # Use call_soon_threadsafe since this callback comes from the clock thread
        self._loop.call_soon_threadsafe(self._handle_jump, jump)

    def _handle_jump(self, jump: TimeJump) -> None:
        """Process a time jump on the event loop thread."""
        if self._is_finished():
            self.cancel()
            return
        if jump.clock_change in (
            ClockChange.ROS_TIME_ACTIVATED,
            ClockChange.ROS_TIME_DEACTIVATED,
        ):
            self.cancel()
            if self._on_clock_change:
                self._on_clock_change(jump.clock_change)
            else:
                print(f'Cancelling due to clock change: {jump.clock_change}', file=stderr)
            return
        self._process()

    def _ros_time_active(self) -> bool:
        """Check if ROS time is currently active."""
        return isinstance(self._clock, ROSClock) and self._clock.ros_time_is_active


class _TimerHandler:
    """Handler for a single Timer, using _WaitHandler for time-based scheduling."""

    def __init__(
        self,
        timer: Timer,
        loop: asyncio.AbstractEventLoop,
        schedule_callback: Callable[[Coroutine], None],
        take_timer: Callable[[Timer], Optional[Coroutine]],
    ) -> None:
        self._timer = timer
        self._loop = loop
        self._schedule_callback = schedule_callback
        self._take_timer = take_timer
        self._waiter: Optional[_WaitHandler] = None
        self._build_waiter()

    def _build_waiter(self) -> None:
        """Create a new _WaitHandler for this timer."""
        self._waiter = _WaitHandler(
            clock=self._timer.clock,
            loop=self._loop,
            is_finished=self._finished,
            is_ready=self._ready,
            time_until_ready_sec=self._time_until_ready,
            on_ready=self._on_ready,
        )

    def _finished(self) -> bool:
        """Check if the timer has been destroyed or canceled."""
        return self._timer.handle.pointer == 0 or self._timer.is_canceled()

    def _ready(self) -> bool:
        """Check if the timer is ready to fire."""
        return self._timer.is_ready()

    def _time_until_ready(self) -> float:
        """Get time in seconds until timer is ready."""
        return self._timer.time_until_next_call() / S_TO_NS

    def _on_ready(self) -> None:
        """Handle the timer being ready to fire."""
        callback = self._take_timer(self._timer)
        if callback:
            self._schedule_callback(callback)

    def on_remove(self) -> None:
        """Clean up when the timer is being removed from the executor."""
        if self._waiter:
            self._waiter.cancel()
            self._waiter = None

    def on_reset(self, _: int = 0) -> None:
        """Rebuild the waiter when the timer is reset."""
        if self._waiter:
            self._waiter.cancel()
        self._build_waiter()


class _SleepWaiter:
    """Handler for async sleep using _WaitHandler for time-based scheduling."""

    def __init__(
        self,
        clock: Clock,
        until: Time,
        loop: asyncio.AbstractEventLoop,
        future: 'asyncio.Future[bool]',
    ) -> None:
        self._clock = clock
        self._until = until
        self._future = future
        self._waiter: Optional[_WaitHandler] = None
        self._build_waiter(loop)

    def _build_waiter(self, loop: asyncio.AbstractEventLoop) -> None:
        """Create a _WaitHandler for this sleep."""
        self._waiter = _WaitHandler(
            clock=self._clock,
            loop=loop,
            is_finished=self._is_finished,
            is_ready=self._is_ready,
            time_until_ready_sec=self._time_until_ready,
            on_ready=self._on_ready,
            on_clock_change=self._on_clock_change,
        )

    def _is_finished(self) -> bool:
        """Check if the sleep has completed."""
        return self._future.done()

    def _is_ready(self) -> bool:
        """Check if the target time has been reached."""
        return self._clock.now() >= self._until

    def _time_until_ready(self) -> float:
        """Get time in seconds until target time."""
        delta = self._until - self._clock.now()
        return delta.nanoseconds / S_TO_NS

    def _on_ready(self) -> None:
        """Complete the future when target time is reached."""
        if not self._future.done():
            self._future.set_result(True)

    def _on_clock_change(self, _clock_change: ClockChange) -> None:
        """Complete the future with False when clock source changes."""
        if not self._future.done():
            self._future.set_result(False)

    def cancel(self) -> None:
        """Cancel the sleep and clean up resources."""
        if self._waiter:
            self._waiter.cancel()
            self._waiter = None
        if not self._future.done():
            self._future.set_result(False)


class AsyncioExecutor(BaseExecutor):
    def __init__(
        self, loop: Optional[asyncio.AbstractEventLoop] = None,
        *,
        context: Optional[Context] = None
    ) -> None:
        self._owns_loop = False
        self._loop = loop or self._get_loop()
        self._context = context or get_default_context()
        self._context.on_shutdown(self._sync_shutdown)
        self._nodes: Set['Node'] = set()
        self._subscription_to_node: Dict[Subscription, 'Node'] = {}
        self._client_to_node: Dict[Client, 'Node'] = {}
        self._service_to_node: Dict[Service, 'Node'] = {}
        self._timer_to_node: Dict[Timer, 'Node'] = {}
        self._timer_handlers: Dict[Timer, _TimerHandler] = {}
        self._node_to_tasks: Dict['Node', Set[asyncio.Task]] = {}
        self._shutdown_task: Optional[asyncio.Task] = None

    def get_nodes(self) -> List['Node']:
        """Return nodes that have been added to this executor."""
        return list(self._nodes)

    @property
    def context(self) -> Context:
        """Get the context associated with the executor."""
        return self._context

    @property
    def loop(self) -> asyncio.AbstractEventLoop:
        """Get the event loop associated with the executor."""
        return self._loop

    def create_future(self) -> asyncio.Future:
        """Create an asyncio.Future attached to this executor's event loop."""
        return self._loop.create_future()

    async def __aenter__(self) -> 'AsyncioExecutor':
        return self

    async def __aexit__(
        self,
        _exc_type: Optional[type[BaseException]],
        _exc_val: Optional[BaseException],
        _exc_tb: Optional[object],
    ) -> None:
        await self.shutdown()

    def spin(self) -> None:
        """Block and process callbacks until shutdown."""
        self._loop.run_forever()

    def _clear_entities(self) -> List[asyncio.Task]:
        self._nodes.clear()
        self._update_entities_from_nodes()

        all_tasks = []
        for tasks in self._node_to_tasks.values():
            for task in tasks:
                task.cancel()
                all_tasks.append(task)
        self._node_to_tasks.clear()
        return all_tasks

    async def shutdown(self) -> None:
        """Clear all nodes and cancel pending tasks."""
        all_tasks = self._clear_entities()
        if all_tasks:
            await asyncio.gather(*all_tasks, return_exceptions=True)

    def __del__(self) -> None:
        if self._owns_loop and not self._loop.is_closed():
            self._loop.close()

    async def _gather_and_stop(self, tasks: List[asyncio.Task]) -> None:
        await asyncio.gather(*tasks, return_exceptions=True)
        if self._owns_loop:
            self._loop.stop()

    def _sync_shutdown(self) -> None:
        """Shut down synchronously when called by context on_shutdown."""
        all_tasks = self._clear_entities()

        if not all_tasks:
            return

        if self._loop.is_closed():
            warnings.warn(
                f'Event loop is closed but {len(all_tasks)} tasks are still pending. '
                'Call "await executor.shutdown()" before closing the event loop.',
                RuntimeWarning,
                stacklevel=2
            )
            return

        if self._loop.is_running():
            self._shutdown_task = self._loop.create_task(self._gather_and_stop(all_tasks))
        else:
            self._loop.run_until_complete(asyncio.gather(*all_tasks, return_exceptions=True))

    def _get_loop(self) -> asyncio.AbstractEventLoop:
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            self._owns_loop = True
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            return loop

    def wake(self) -> None:
        self._update_entities_from_nodes()

    def add_node(self, node: Node) -> bool:
        if node in self._nodes:
            return False

        self._nodes.add(node)
        self._node_to_tasks[node] = set()
        node.executor = self
        self._update_entities_from_nodes()
        return True

    async def remove_node(self, node: Node) -> None:
        if node not in self._nodes:
            return

        self._nodes.remove(node)
        self._update_entities_from_nodes()

        node_tasks = self._node_to_tasks.pop(node)
        for task in node_tasks:
            task.cancel()
        if node_tasks:
            await asyncio.gather(*node_tasks, return_exceptions=True)

    def _update_entities_from_nodes(self) -> None:
        new_subscriptions: Dict[Subscription, Node] = {}
        new_clients: Dict[Client, Node] = {}
        new_services: Dict[Service, Node] = {}
        new_timers: Dict[Timer, Node] = {}
        for node in self._nodes:
            new_subscriptions.update({sub: node for sub in node.subscriptions})
            new_clients.update({cli: node for cli in node.clients})
            new_services.update({srv: node for srv in node.services})
            new_timers.update({tmr: node for tmr in node.timers})

        self._update_entity_set(
            self._subscription_to_node,
            new_subscriptions,
            self._handle_added_subscription,
            self._handle_removed_subscription
        )
        self._update_entity_set(
            self._client_to_node,
            new_clients,
            self._handle_added_client,
            self._handle_removed_client
        )
        self._update_entity_set(
            self._service_to_node,
            new_services,
            self._handle_added_service,
            self._handle_removed_service
        )
        self._update_entity_set(
            self._timer_to_node,
            new_timers,
            self._handle_added_timer,
            self._handle_removed_timer
        )

    def _handle_added_subscription(self, sub: Subscription, node: Node):
        sub.handle.set_on_new_message_callback(
            partial(
                self._loop.call_soon_threadsafe,
                self._handle_ready_entity,
                self._take_subscription,
                sub,
                node,
            )
        )

    def _handle_removed_subscription(self, sub: Subscription):
        sub.handle.clear_on_new_message_callback()

    def _handle_added_client(self, client: Client, node: Node):
        # Warn about pre-existing requests that were created before node was added to executor.
        # Those requests returned rclpy.Future which cannot be awaited in asyncio.
        if client._pending_requests:
            warnings.warn(
                f'Client "{client.srv_name}" has {len(client._pending_requests)} pending '
                'request(s) created before being added to AsyncioExecutor. These requests '
                'returned rclpy.Future which cannot be awaited in asyncio. Add nodes to '
                'AsyncioExecutor before calling call_async().',
                RuntimeWarning,
                stacklevel=4
            )

        client.handle.set_on_new_response_callback(
            partial(
                self._loop.call_soon_threadsafe,
                self._handle_ready_entity,
                self._take_client,
                client,
                node,
            )
        )

    def _handle_removed_client(self, client: Client):
        client.handle.clear_on_new_response_callback()

    def _handle_added_service(self, service: Service, node: Node):
        service.handle.set_on_new_request_callback(
            partial(
                self._loop.call_soon_threadsafe,
                self._handle_ready_entity,
                self._take_service,
                service,
                node,
            )
        )

    def _handle_removed_service(self, service: Service):
        service.handle.clear_on_new_request_callback()

    def _handle_added_timer(self, timer: Timer, node: Node) -> None:
        """Set up a timer handler when a timer is added."""
        handler = _TimerHandler(
            timer=timer,
            loop=self._loop,
            schedule_callback=partial(self._schedule_timer_callback, node),
            take_timer=self._take_timer,
        )
        self._timer_handlers[timer] = handler
        timer.handle.set_on_reset_callback(handler.on_reset)

    def _handle_removed_timer(self, timer: Timer) -> None:
        """Clean up when a timer is removed."""
        timer.handle.clear_on_reset_callback()
        handler = self._timer_handlers.pop(timer, None)
        if handler:
            handler.on_remove()

    def _schedule_timer_callback(
        self,
        node: Node,
        callback: Coroutine,
    ) -> None:
        """Schedule a timer callback as an asyncio task."""
        if node not in self._nodes:
            return

        tasks = self._node_to_tasks[node]
        task = self._loop.create_task(callback)
        task.add_done_callback(partial(self._done_callback, node))
        tasks.add(task)

    def _update_entity_set(
        self,
        current_entity_to_node: Dict[EntityT, Node],
        new_entity_to_node: Dict[EntityT, Node],
        on_added_entity: Callable[[EntityT, Node], None],
        on_removed_entity: Callable[[EntityT], None],
    ) -> bool:
        current_entities = set(current_entity_to_node.keys())
        new_entities = set(new_entity_to_node.keys())

        added_entities = new_entities - current_entities
        for entity in added_entities:
            node = new_entity_to_node[entity]
            current_entity_to_node[entity] = node
            entity.handle.__enter__()
            on_added_entity(entity, node)

        removed_entities = current_entities - new_entities
        for entity in removed_entities:
            on_removed_entity(entity)
            entity.handle.__exit__(None, None, None)
            del current_entity_to_node[entity]

        return bool(added_entities or removed_entities)

    def _handle_ready_entity(
        self,
        take_entity_callback: Callable[[EntityT], Optional[Coroutine]],
        entity: EntityT,
        node: Node,
        number_of_events: int,
    ) -> None:
        if node not in self._nodes:
            return

        tasks = self._node_to_tasks[node]
        for _ in range(number_of_events):
            callback = take_entity_callback(entity)
            if not callback:
                return

            task = self._loop.create_task(callback)
            task.add_done_callback(partial(self._done_callback, node))
            tasks.add(task)

    def _done_callback(
        self,
        node: Node,
        task: asyncio.Task,
    ) -> None:
        if task.cancelled():
            return

        # Guard against node being removed between task creation and done callback
        if node in self._node_to_tasks:
            self._node_to_tasks[node].discard(task)

        exc = task.exception()
        if exc:
            node.get_logger().error(''.join(traceback.format_exception(exc)))

    async def sleep(self, seconds: float, clock: Clock) -> bool:
        """
        Sleep for the specified number of seconds using the given clock.

        When using a ROSClock, ROS time being activated or deactivated causes
        this function to return False early.

        :param seconds: Number of seconds to sleep.
        :param clock: Clock to use for timing.
        :return: True if the full duration was slept, False if interrupted
            by clock source change (only possible with ROSClock).
        """
        until = clock.now() + Duration(seconds=seconds)
        future: asyncio.Future[bool] = self._loop.create_future()
        waiter = _SleepWaiter(clock, until, self._loop, future)
        try:
            return await future
        finally:
            waiter.cancel()
