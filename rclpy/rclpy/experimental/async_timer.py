# Copyright 2026 Open Source Robotics Foundation, Inc.
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
import inspect
from typing import Callable, Optional

from rclpy.clock import ClockChange, JumpHandle, JumpThreshold, TimeJump
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.exceptions import TimeSourceChangedError
from rclpy.executors import await_or_execute
from rclpy.timer import BaseTimer, TimerCallbackUnion, TimerInfo

from .async_clock import AsyncClock


class AsyncTimer(BaseTimer):
    """
    A periodic timer.

    .. admonition:: Experimental

       This API is experimental.
    """

    def __init__(
        self,
        timer_period_ns: int,
        clock: AsyncClock,
        context: Context,
        callback: TimerCallbackUnion,
        autostart: bool,
        on_destroy: Callable[['AsyncTimer'], None],
        tg: Optional[asyncio.TaskGroup] = None,
    ) -> None:
        super().__init__(callback, timer_period_ns, clock, context=context,
                         on_destroy=on_destroy, autostart=autostart)
        self._task: Optional[asyncio.Task] = None
        self._reset_event = asyncio.Event()
        self._sleep_waiter: Optional[asyncio.Future] = None
        self._jump_handle: Optional[JumpHandle] = None
        if tg is not None:
            self._task = tg.create_task(self._run())

    @property
    def callback(self) -> TimerCallbackUnion:
        return self._callback

    @callback.setter
    def callback(self, cb: TimerCallbackUnion) -> None:
        self._callback = cb
        self._pass_info = self._detect_wants_info(cb)

    @staticmethod
    def _detect_wants_info(callback: Callable) -> bool:
        try:
            inspect.signature(callback).bind()
            return False
        except TypeError:
            pass
        try:
            inspect.signature(callback).bind(object())
            return True
        except TypeError:
            pass
        raise RuntimeError(
            'Timer callback must accept zero arguments '
            'or one argument (TimerInfo)')

    def _destroy(self) -> None:
        if self._task is not None:
            self._task.cancel()
        if self._jump_handle is not None:
            self._jump_handle.unregister()
        self.handle.clear_on_reset_callback()
        super()._destroy()

    def cancel(self) -> None:
        super().cancel()
        self._resolve()

    def _on_reset(self, _count: int) -> None:
        self._resolve()
        self._reset_event.set()

    def _resolve(self) -> None:
        if self._sleep_waiter is not None and not self._sleep_waiter.done():
            self._sleep_waiter.set_result(None)

    def _reject(self) -> None:
        if self._sleep_waiter is not None and not self._sleep_waiter.done():
            self._sleep_waiter.set_exception(TimeSourceChangedError())

    def _on_jump(self, time_jump: TimeJump) -> None:
        if time_jump.clock_change in (
            ClockChange.ROS_TIME_ACTIVATED,
            ClockChange.ROS_TIME_DEACTIVATED,
        ):
            self._reject()
        elif time_jump.clock_change == ClockChange.ROS_TIME_NO_CHANGE and self.is_ready():
            self._resolve()

    async def _wait(self) -> None:
        duration_ns = self.time_until_next_call()
        if duration_ns is None or duration_ns <= 0:
            await asyncio.sleep(0)
            return

        loop = asyncio.get_running_loop()
        future: asyncio.Future[None] = loop.create_future()
        self._sleep_waiter = future
        timer_handle: Optional[asyncio.TimerHandle] = None

        if not self._clock.ros_time_is_active:
            timer_handle = loop.call_later(duration_ns / 1e9, self._resolve)

        try:
            await future
        finally:
            self._sleep_waiter = None
            if timer_handle is not None:
                timer_handle.cancel()

    async def _call(self) -> None:
        info = self.handle.call_timer_with_info()
        if self._pass_info:
            timer_info = TimerInfo(
                expected_call_time=info['expected_call_time'],
                actual_call_time=info['actual_call_time'],
                clock_type=self._clock.clock_type)
            await await_or_execute(self.callback, timer_info)
        else:
            await await_or_execute(self.callback)

    async def _run(self) -> None:
        threshold = JumpThreshold(
            min_forward=Duration(nanoseconds=1),
            min_backward=None,
            on_clock_change=True,
        )
        self._jump_handle = self._clock.create_jump_callback(
            threshold, post_callback=self._on_jump)
        self.handle.set_on_reset_callback(self._on_reset)
        try:
            while not self._destroyed:
                try:
                    await self._wait()
                except (TimeSourceChangedError):
                    continue

                if self._reset_event.is_set():
                    self._reset_event.clear()
                    continue
                if self.is_canceled():
                    await self._reset_event.wait()
                    self._reset_event.clear()
                    continue

                await self._call()

        finally:
            self._task = None
            self.destroy()
