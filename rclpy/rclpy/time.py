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

import builtin_interfaces
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class Time:

    def __init__(self, *, seconds=0, nanoseconds=0, clock_type=ClockType.SYSTEM_TIME):
        if not isinstance(clock_type, ClockType):
            raise TypeError('Clock type must be a ClockType enum')
        if seconds < 0:
            raise ValueError('Seconds value must not be negative')
        if nanoseconds < 0:
            raise ValueError('Nanoseconds value must not be negative')
        total_nanoseconds = seconds * 1e9
        total_nanoseconds += nanoseconds
        self._time_handle = _rclpy.rclpy_create_time_point(int(total_nanoseconds), clock_type)
        self._clock_type = clock_type

    @property
    def nanoseconds(self):
        return _rclpy.rclpy_time_point_get_nanoseconds(self._time_handle)

    @property
    def clock_type(self):
        return self._clock_type

    def __add__(self, other):
        if isinstance(other, Duration):
            # TODO(dhood): overflow checking
            return Time(
                nanoseconds=(self.nanoseconds + other.nanoseconds),
                clock_type=self.clock_type)
        else:
            return NotImplemented

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't subtract times with different clock types")
            # TODO(dhood): underflow checking
            return Duration(nanoseconds=(self.nanoseconds - other.nanoseconds))
        if isinstance(other, Duration):
            # TODO(dhood): underflow checking
            return Time(
                nanoseconds=(self.nanoseconds - other.nanoseconds),
                clock_type=self.clock_type)
        else:
            return NotImplemented

    def to_msg(self):
        seconds = int(self.nanoseconds * 1e-9)
        nanoseconds = int(self.nanoseconds % 1e9)
        return builtin_interfaces.msg.Time(sec=seconds, nanosec=nanoseconds)

    @classmethod
    def from_msg(cls, msg, clock_type=ClockType.ROS_TIME):
        if not isinstance(msg, builtin_interfaces.msg.Time):
            raise TypeError('Must pass a builtin_interfaces.msg.Time object')
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec, clock_type=clock_type)
