# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import threading

from types import TracebackType
from typing import Callable
from typing import Optional
from typing import Type
from typing import Union

from rclpy.callback_groups import CallbackGroup
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.context import Context
from rclpy.exceptions import InvalidHandle, ROSInterruptException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.time import Time
from rclpy.utilities import get_default_context


class TimerInfo:
    """
    Represents a timer call information.

    A ``TimerInfo`` object encapsulate the timer information when called.
    """

    def __init__(
            self, *,
            expected_call_time: int = 0,
            actual_call_time: int = 0,
            clock_type: ClockType = ClockType.SYSTEM_TIME):
        if not isinstance(clock_type, (ClockType, _rclpy.ClockType)):
            raise TypeError('Clock type must be a ClockType enum')
        if expected_call_time < 0 or actual_call_time < 0:
            raise ValueError('call time values must not be negative')
        self._expected_call_time: Time = Time(
            nanoseconds=expected_call_time, clock_type=clock_type)
        self._actual_call_time: Time = Time(
            nanoseconds=actual_call_time, clock_type=clock_type)

    @property
    def expected_call_time(self) -> Time:
        """:return: the expected_call_time."""
        return self._expected_call_time

    @property
    def actual_call_time(self) -> Time:
        """:return: the actual_call_time."""
        return self._actual_call_time


class Timer:

    def __init__(
        self,
        callback: Union[Callable[[], None], Callable[[TimerInfo], None], None],
        callback_group: Optional[CallbackGroup],
        timer_period_ns: int,
        clock: Clock,
        *,
        context: Optional[Context] = None,
        autostart: bool = True
    ):
        """
        Create a Timer.

        If autostart is ``True`` (the default), the timer will be started and every
        ``timer_period_sec`` number of seconds the provided callback function will be called.
        If autostart is ``False``, the timer will be created but not started; it can then be
        started by calling ``reset()`` on the timer object.

        .. warning:: Users should not create a timer with this constructor, instead they
           should call :meth:`.Node.create_timer`.

        :param callback: A user-defined callback function that is called when the timer expires.
        :param callback_group: The callback group for the timer. If ``None``, then the
            default callback group for the node is used.
        :param timer_period_ns: The period (in nanoseconds) of the timer.
        :param clock: The clock which the timer gets time from.
        :param context: The context to be associated with.
        :param autostart: Whether to automatically start the timer after creation; defaults to
            ``True``.
        """
        self._context = get_default_context() if context is None else context
        self._clock = clock
        if self._context.handle is None:
            raise RuntimeError('Context must be initialized before create a _rclpy.Timer.')
        with self._clock.handle, self._context.handle:
            self.__timer = _rclpy.Timer(
                self._clock.handle, self._context.handle, timer_period_ns, autostart)
        self.timer_period_ns = timer_period_ns
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False

    @property
    def handle(self) -> _rclpy.Timer:
        return self.__timer

    def destroy(self) -> None:
        """
        Destroy a container for a ROS timer.

        .. warning:: Users should not destroy a timer with this method, instead they should
           call :meth:`.Node.destroy_timer`.
        """
        self.__timer.destroy_when_not_in_use()

    @property
    def clock(self) -> Clock:
        return self._clock

    @property
    def timer_period_ns(self) -> int:
        with self.__timer:
            val = self.__timer.get_timer_period()
        self.__timer_period_ns = val
        return val

    @timer_period_ns.setter
    def timer_period_ns(self, value: int) -> None:
        val = int(value)
        with self.__timer:
            self.__timer.change_timer_period(val)
        self.__timer_period_ns = val

    def is_ready(self) -> bool:
        with self.__timer:
            return self.__timer.is_timer_ready()

    def is_canceled(self) -> bool:
        with self.__timer:
            return self.__timer.is_timer_canceled()

    def cancel(self) -> None:
        with self.__timer:
            self.__timer.cancel_timer()

    def reset(self) -> None:
        with self.__timer:
            self.__timer.reset_timer()

    def time_since_last_call(self) -> int:
        with self.__timer:
            return self.__timer.time_since_last_call()

    def time_until_next_call(self) -> Optional[int]:
        with self.__timer:
            return self.__timer.time_until_next_call()

    def __enter__(self) -> 'Timer':
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> None:
        self.destroy()


class Rate:
    """A utility for sleeping at a fixed rate."""

    def __init__(self, timer: Timer, *, context: Context):
        """
        Create a Rate.

        .. warning:: Users should not create a rate with this constructor, instead they
           should call :meth:`.Node.create_rate`.
        """
        # Rate is a wrapper around a timer
        self._timer = timer
        self._is_shutdown = False
        self._is_destroyed = False

        # This event is set to wake sleepers
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._num_sleepers = 0

        # Set event when timer fires
        self._timer.callback = self._event.set

        # Set event when ROS is shutdown
        context.on_shutdown(self._on_shutdown)

    def _on_shutdown(self) -> None:
        self._is_shutdown = True
        self.destroy()

    def destroy(self) -> None:
        """
        Destroy a container for a ROS rate.

        .. warning:: Users should not destroy a rate with this method, instead they should
           call :meth:`.Node.destroy_rate`.
        """
        self._is_destroyed = True
        self._event.set()

    def _presleep(self) -> None:
        if self._is_shutdown:
            raise ROSInterruptException()
        if self._is_destroyed:
            raise RuntimeError('Rate cannot sleep because it has been destroyed')
        if not self._timer.handle:
            self.destroy()
            raise InvalidHandle('Rate cannot sleep because the timer has been destroyed')
        with self._lock:
            self._num_sleepers += 1

    def _postsleep(self) -> None:
        with self._lock:
            self._num_sleepers -= 1
            if self._num_sleepers == 0:
                self._event.clear()
        if self._is_shutdown:
            self.destroy()
            raise ROSInterruptException()

    def sleep(self) -> None:
        """
        Block until timer triggers.

        Care should be taken when calling this from a callback.
        This may block forever if called in a callback in a SingleThreadedExecutor.
        """
        self._presleep()
        try:
            self._event.wait()
        finally:
            self._postsleep()
