# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

from .duration import Duration


ClockType = _rclpy.ClockType
ClockChange = _rclpy.ClockChange


class JumpThreshold:

    def __init__(self, *, min_forward, min_backward, on_clock_change=True):
        """
        Initialize an instance of JumpThreshold.

        :param min_forward: Minimum jump forwards to be considered exceeded, or None.
        :param min_backward: Negative duration indicating minimum jump backwards to be considered
                             exceeded, or None.
        :param on_clock_change: True to make a callback happen when ROS_TIME is activated
                                or deactivated.
        """
        self.min_forward = min_forward
        self.min_backward = min_backward
        self.on_clock_change = on_clock_change


class TimeJump:

    def __init__(self, clock_change, delta):
        if not isinstance(clock_change, ClockChange):
            raise TypeError('clock_change must be an instance of rclpy.clock.ClockChange')
        self._clock_change = clock_change
        self._delta = delta

    @property
    def clock_change(self):
        return self._clock_change

    @property
    def delta(self):
        return self._delta


class JumpHandle:

    def __init__(self, *, clock, threshold, pre_callback, post_callback):
        """
        Register a clock jump callback.

        :param clock: Clock that time jump callback is registered to
        :param pre_callback: Callback to be called before new time is set.
        :param post_callback: Callback to be called after new time is set, accepting a dictionary
            with keys "clock_change" and "delta".
        """
        if pre_callback is None and post_callback is None:
            raise ValueError('One of pre_callback or post_callback must be callable')
        if pre_callback is not None and not callable(pre_callback):
            raise ValueError('pre_callback must be callable if given')
        if post_callback is not None and not callable(post_callback):
            raise ValueError('post_callback must be callable if given')
        self._clock = clock
        self._pre_callback = pre_callback
        self._post_callback = post_callback

        min_forward = 0
        if threshold.min_forward is not None:
            min_forward = threshold.min_forward.nanoseconds
        min_backward = 0
        if threshold.min_backward is not None:
            min_backward = threshold.min_backward.nanoseconds

        with self._clock.handle:
            self._clock.handle.add_clock_callback(
                 self, threshold.on_clock_change, min_forward, min_backward)

    def unregister(self):
        """Remove a jump callback from the clock."""
        if self._clock is not None:
            with self._clock.handle:
                self._clock.handle.remove_clock_callback(self)
            self._clock = None


class Clock:

    def __new__(cls, *, clock_type=ClockType.SYSTEM_TIME):
        if not isinstance(clock_type, ClockType):
            raise TypeError('Clock type must be a ClockType enum')
        if clock_type is ClockType.ROS_TIME:
            self = super().__new__(ROSClock)
        else:
            self = super().__new__(cls)
        self.__clock = _rclpy.Clock(clock_type)
        self._clock_type = clock_type
        return self

    @property
    def clock_type(self):
        return self._clock_type

    @property
    def handle(self):
        return self.__clock

    def __repr__(self):
        return 'Clock(clock_type={0})'.format(self.clock_type.name)

    def now(self):
        from rclpy.time import Time
        with self.handle:
            rcl_time = self.__clock.get_now()
        return Time(nanoseconds=rcl_time.nanoseconds, clock_type=self.clock_type)

    def create_jump_callback(self, threshold, *, pre_callback=None, post_callback=None):
        """
        Create callback handler for clock time jumps.

        The callbacks must remain valid as long as the returned JumpHandler is valid.
        A callback should execute as quick as possible and must not block when called.
        If a callback raises then no time jump callbacks added after it will be called.

        :param threshold: Criteria for activating time jump.
        :param pre_callback: Callback to be called before new time is set.
        :param post_callback: Callback to be called after new time is set accepting
        :rtype: :class:`rclpy.clock.JumpHandler`
        """
        if post_callback is not None and callable(post_callback):
            original_callback = post_callback

            def callback_shim(jump_dict):
                nonlocal original_callback
                clock_change = jump_dict['clock_change']
                duration = Duration(nanoseconds=jump_dict['delta'])
                original_callback(TimeJump(clock_change, duration))

            post_callback = callback_shim

        return JumpHandle(
            clock=self, threshold=threshold, pre_callback=pre_callback,
            post_callback=post_callback)


class ROSClock(Clock):

    def __new__(cls):
        return super().__new__(Clock, clock_type=ClockType.ROS_TIME)

    @property
    def ros_time_is_active(self):
        with self.handle:
            return self.handle.get_ros_time_override_is_enabled()

    def _set_ros_time_is_active(self, enabled):
        # This is not public because it is only to be called by a TimeSource managing the Clock
        with self.handle:
            self.handle.set_ros_time_override_is_enabled(enabled)

    def set_ros_time_override(self, time):
        from rclpy.time import Time
        if not isinstance(time, Time):
            raise TypeError(
                'Time must be specified as rclpy.time.Time. Received type: {0}'.format(type(time)))
        with self.handle:
            self.handle.set_ros_time_override(time._time_handle)
