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
from typing import Dict, Optional

from rclpy.clock import BaseClock, ClockChange, JumpHandle, JumpThreshold, TimeJump
from rclpy.clock_type import ClockType
from rclpy.duration import Duration
from rclpy.exceptions import TimeSourceChangedError
from rclpy.time import Time


class AsyncClock(BaseClock):
    """
    A ROS clock with async ``sleep``.

    .. admonition:: Experimental

       This API is experimental.
    """

    def __init__(self, *, clock_type: ClockType = ClockType.SYSTEM_TIME) -> None:
        super().__init__(clock_type=clock_type)
        self._pending_sleeps: Dict[asyncio.Future[None], Optional[Time]] = {}
        self._destroyed = False

        threshold = JumpThreshold(
            min_forward=Duration(nanoseconds=1),
            min_backward=None,
            on_clock_change=True,
        )
        self._jump_handle: JumpHandle = self.create_jump_callback(
            threshold, post_callback=self._on_jump)

    @staticmethod
    def _resolve_future(future: asyncio.Future[None]) -> None:
        if not future.done():
            future.set_result(None)

    def _on_jump(self, time_jump: TimeJump) -> None:
        if time_jump.clock_change in (
            ClockChange.ROS_TIME_ACTIVATED,
            ClockChange.ROS_TIME_DEACTIVATED,
        ):
            for future in self._pending_sleeps:
                if not future.done():
                    future.set_exception(TimeSourceChangedError())
        elif time_jump.clock_change == ClockChange.ROS_TIME_NO_CHANGE:
            now = self.now()
            for future, target in self._pending_sleeps.items():
                if target is not None and now >= target:
                    self._resolve_future(future)

    def _destroy(self) -> None:
        """Cancel all pending sleeps. Called by AsyncNode.destroy_node()."""
        if self._destroyed:
            return
        self._destroyed = True
        self._jump_handle.unregister()
        for future in self._pending_sleeps:
            future.cancel()
        self.handle.destroy_when_not_in_use()

    async def sleep(self, duration_sec: float) -> None:
        """
        Sleep for a duration respecting sim time.

        Cancelled on clock destruction. Raises TimeSourceChangedError if ROS
        time is activated or deactivated during the sleep.
        """
        if self._destroyed:
            raise RuntimeError('Cannot sleep on a destroyed clock')
        duration = Duration(seconds=duration_sec)
        if duration <= Duration(nanoseconds=0):
            await asyncio.sleep(0)
            return

        loop = asyncio.get_running_loop()

        if self.ros_time_is_active:
            # _on_jump resolves the future only when self.now() >= target,
            # so this branch already provides "at least duration" semantics.
            future: asyncio.Future[None] = loop.create_future()
            target: Time = self.now() + duration
            self._pending_sleeps[future] = target
            try:
                await future
            finally:
                self._pending_sleeps.pop(future)
            return

        # asyncio.call_later is allowed to fire up to loop._clock_resolution
        # early -- on Windows that window is the OS scheduler tick (~15 ms).
        # Re-arm until the deadline is actually reached so callers get an
        # "at least duration_sec" guarantee, matching the strict semantics
        # of Clock.sleep_for in rclpy/rclcpp.
        deadline = loop.time() + duration_sec
        while True:
            remaining = deadline - loop.time()
            if remaining <= 0:
                return
            future = loop.create_future()
            timer_handle = loop.call_later(
                remaining, AsyncClock._resolve_future, future)
            self._pending_sleeps[future] = None
            try:
                await future
            finally:
                self._pending_sleeps.pop(future)
                timer_handle.cancel()
