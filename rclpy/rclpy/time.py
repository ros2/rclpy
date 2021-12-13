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

import builtin_interfaces.msg
from rclpy.duration import Duration
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


CONVERSION_CONSTANT = 10 ** 9


class Time:

    def __init__(
            self, *,
            seconds=0, nanoseconds=0,
            clock_type: _rclpy.ClockType = _rclpy.ClockType.SYSTEM_TIME):
        if not isinstance(clock_type, _rclpy.ClockType):
            raise TypeError('Clock type must be a ClockType enum')
        if seconds < 0:
            raise ValueError('Seconds value must not be negative')
        if nanoseconds < 0:
            raise ValueError('Nanoseconds value must not be negative')
        total_nanoseconds = int(seconds * CONVERSION_CONSTANT)
        total_nanoseconds += int(nanoseconds)
        if total_nanoseconds >= 2**63:
            # pybind11 would raise TypeError, but we want OverflowError
            raise OverflowError(
                'Total nanoseconds value is too large to store in C time point.')
        self._time_handle = _rclpy.rcl_time_point_t(total_nanoseconds, clock_type)

    @property
    def nanoseconds(self):
        return self._time_handle.nanoseconds

    def seconds_nanoseconds(self):
        """
        Get time as separate seconds and nanoseconds components.

        :returns: 2-tuple seconds and nanoseconds
        :rtype: tuple(int, int)
        """
        nanoseconds = self.nanoseconds
        return (nanoseconds // CONVERSION_CONSTANT, nanoseconds % CONVERSION_CONSTANT)

    @property
    def clock_type(self):
        return self._time_handle.clock_type

    def __repr__(self):
        return 'Time(nanoseconds={0}, clock_type={1})'.format(
            self.nanoseconds, self.clock_type.name)

    def __add__(self, other):
        if isinstance(other, Duration):
            try:
                return Time(
                    nanoseconds=(self.nanoseconds + other.nanoseconds),
                    clock_type=self.clock_type)
            except OverflowError as e:
                raise OverflowError('Addition leads to overflow in C storage.') from e
        else:
            return NotImplemented

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't subtract times with different clock types")
            try:
                return Duration(nanoseconds=(self.nanoseconds - other.nanoseconds))
            except ValueError as e:
                raise ValueError('Subtraction leads to negative duration.') from e
        if isinstance(other, Duration):
            try:
                return Time(
                    nanoseconds=(self.nanoseconds - other.nanoseconds),
                    clock_type=self.clock_type)
            except ValueError as e:
                raise ValueError('Subtraction leads to negative time.') from e
        else:
            return NotImplemented

    def __eq__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds == other.nanoseconds
        # Raise instead of returning NotImplemented to prevent comparison with invalid types,
        # e.g. ints.
        # Otherwise `Time(nanoseconds=5) == 5` will return False instead of raising, and this
        # could lead to hard-to-find bugs.
        raise TypeError("Can't compare time with object of type: ", type(other))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds < other.nanoseconds
        return NotImplemented

    def __le__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds <= other.nanoseconds
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds > other.nanoseconds
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds >= other.nanoseconds
        return NotImplemented

    def to_msg(self):
        seconds, nanoseconds = self.seconds_nanoseconds()
        return builtin_interfaces.msg.Time(sec=seconds, nanosec=nanoseconds)

    @classmethod
    def from_msg(cls, msg, clock_type: _rclpy.ClockType = _rclpy.ClockType.ROS_TIME):
        if not isinstance(msg, builtin_interfaces.msg.Time):
            raise TypeError('Must pass a builtin_interfaces.msg.Time object')
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec, clock_type=clock_type)
