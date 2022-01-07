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
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


CONVERSION_CONSTANT = 10 ** 9


class Duration:

    def __init__(self, *, seconds=0, nanoseconds=0):
        total_nanoseconds = int(seconds) * CONVERSION_CONSTANT
        total_nanoseconds += int(nanoseconds)
        if total_nanoseconds >= 2**63 or total_nanoseconds < -2**63:
            # pybind11 would raise TypeError, but we want OverflowError
            raise OverflowError(
                'Total nanoseconds value is too large to store in C duration.')
        self._duration_handle = _rclpy.rcl_duration_t(total_nanoseconds)

    @property
    def nanoseconds(self):
        return self._duration_handle.nanoseconds

    def __repr__(self):
        return 'Duration(nanoseconds={0})'.format(self.nanoseconds)

    def __str__(self):
        if self == Infinite:
            return 'Infinite'
        return f'{self.nanoseconds} nanoseconds'

    def __eq__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds == other.nanoseconds
        # Raise instead of returning NotImplemented to prevent comparison with invalid types,
        # e.g. ints.
        # Otherwise `Duration(nanoseconds=5) == 5` will return False instead of raising, and this
        # could lead to hard-to-find bugs.
        raise TypeError("Can't compare duration with object of type: ", type(other))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds < other.nanoseconds
        return NotImplemented

    def __le__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds <= other.nanoseconds
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds > other.nanoseconds
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds >= other.nanoseconds
        return NotImplemented

    def to_msg(self):
        seconds, nanoseconds = divmod(self.nanoseconds, CONVERSION_CONSTANT)
        return builtin_interfaces.msg.Duration(sec=seconds, nanosec=nanoseconds)

    @classmethod
    def from_msg(cls, msg):
        if not isinstance(msg, builtin_interfaces.msg.Duration):
            raise TypeError('Must pass a builtin_interfaces.msg.Duration object')
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec)

    def get_c_duration(self):
        return self._duration_handle


# Constant representing an infinite amount of time.
Infinite = Duration(nanoseconds=_rclpy.RMW_DURATION_INFINITE)
