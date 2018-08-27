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

from enum import IntEnum

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class ClockType(IntEnum):
    """
    Enum for clock type.

    This enum matches the one defined in rcl/time.h
    """

    ROS_TIME = 1
    SYSTEM_TIME = 2
    STEADY_TIME = 3


class ClockChange(IntEnum):

    # The time type before and after the jump is ROS_TIME
    ROS_TIME_NO_CHANGE = 1
    # The time type switched to ROS_TIME from SYSTEM_TIME
    ROS_TIME_ACTIVATED = 2
    # The time type switched to SYSTEM_TIME from ROS_TIME
    ROS_TIME_DEACTIVATED = 3
    # The time type before and after the jump is SYSTEM_TIME
    SYSTEM_TIME_NO_CHANGE = 4


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


class JumpHandle:

    def __init__(self, *, clock, threshold, pre_callback, post_callback):
        """
        Register a clock jump callback

        :param clock: Clock that time jump callback is registered to
        :param pre_callback: Callback to be called before new time is set.
        :param post_callback: Callback to be called after new time is set, accepting a dictionary
            with keys "clock_source_changed" and "delta".
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

       _rclpy.rclpy_add_clock_callback(
            clock, self, threshold.on_clock_change, threshold.min_forward.nanoseconds,
            threshold.min_backward.nanosecond)

    def unregister(self):
        """Removes a jump callback from the clock."""
        if self._clock is not None:
            _rclpy.rclpy_remove_clock_callback(self._clock, self)
            self._clock = None

    def __del__(self):
        self.unregister()


class Clock:

    def __new__(cls, *, clock_type=ClockType.SYSTEM_TIME):
        if not isinstance(clock_type, ClockType):
            raise TypeError('Clock type must be a ClockType enum')
        if clock_type is ClockType.ROS_TIME:
            self = super().__new__(ROSClock)
        else:
            self = super().__new__(cls)
        self._clock_handle = _rclpy.rclpy_create_clock(clock_type)
        self._clock_type = clock_type
        return self

    @property
    def clock_type(self):
        return self._clock_type

    def __repr__(self):
        return 'Clock(clock_type={0})'.format(self.clock_type.name)

    def now(self):
        from rclpy.time import Time
        time_handle = _rclpy.rclpy_clock_get_now(self._clock_handle)
        # TODO(dhood): Return a python object from the C extension
        return Time(
            nanoseconds=_rclpy.rclpy_time_point_get_nanoseconds(time_handle),
            clock_type=self.clock_type)

    def create_jump_callback(self, threshold, *, pre_callback=None, post_callback=None):
        """
        Create callback handler for clock time jumps.

        The callbacks must remain valid as long as the returned JumpHandler is valid.
        A callback takes a single argument of an instance of :class:`rclpy.time_source.TimeJump`.
        A callback should execute as quick as possible and must not block when called.

        :param threshold: Criteria for activating time jump.
        :param pre_callback: Callback to be called before new time is set.
        :param post_callback: Callback to be called after new time is set accepting
        :rtype: :class:`rclpy.clock.JumpHandler`
        """
        handler = JumpHandle(
            threshold=threshold, pre_callback=pre_callback, post_callback=post_callback)
        return handler


class ROSClock(Clock):

    def __new__(cls):
        return super().__new__(Clock, clock_type=ClockType.ROS_TIME)

    @property
    def ros_time_is_active(self):
        return _rclpy.rclpy_clock_get_ros_time_override_is_enabled(self._clock_handle)

    def _set_ros_time_is_active(self, enabled):
        # This is not public because it is only to be called by a TimeSource managing the Clock
        _rclpy.rclpy_clock_set_ros_time_override_is_enabled(self._clock_handle, enabled)

    def set_ros_time_override(self, time):
        from rclpy.time import Time
        if not isinstance(time, Time):
            TypeError(
                'Time must be specified as rclpy.time.Time. Received type: {0}'.format(type(time)))
        _rclpy.rclpy_clock_set_ros_time_override(self._clock_handle, time._time_handle)
