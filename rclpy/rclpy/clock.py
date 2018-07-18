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


class Clock:

    def __init__(self, *, clock_type=ClockType.SYSTEM_TIME):
        if not isinstance(clock_type, ClockType):
            raise TypeError('Clock type must be a ClockType enum')
        if clock_type is ClockType.ROS_TIME:
            raise NotImplementedError
        self._clock_handle = _rclpy.rclpy_create_clock(clock_type)
        self._clock_type = clock_type

    @property
    def clock_type(self):
        return self._clock_type

    def now(self):
        from rclpy.time import Time
        time_handle = _rclpy.rclpy_clock_get_now(self._clock_handle)
        # TODO(dhood): Return a python object from the C extension
        return Time(
            nanoseconds=_rclpy.rclpy_time_point_get_nanoseconds(time_handle),
            clock_type=self.clock_type)
