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


class Duration:

    def __init__(self, *, seconds=0, nanoseconds=0):
        total_nanoseconds = int(seconds * 1e9)
        total_nanoseconds += int(nanoseconds)
        try:
            self._duration_handle = _rclpy.rclpy_create_duration(total_nanoseconds)
        except OverflowError as e:
            raise OverflowError(
                'Total nanoseconds value is too large to store in C time point.') from e

    @property
    def nanoseconds(self):
        return _rclpy.rclpy_duration_get_nanoseconds(self._duration_handle)

    def __repr__(self):
        return 'Duration(nanoseconds={0})'.format(self.nanoseconds)

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
        seconds = int(self.nanoseconds * 1e-9)
        nanoseconds = int(self.nanoseconds % 1e9)
        return builtin_interfaces.msg.Duration(sec=seconds, nanosec=nanoseconds)

    @classmethod
    def from_msg(cls, msg):
        if not isinstance(msg, builtin_interfaces.msg.Duration):
            raise TypeError('Must pass a builtin_interfaces.msg.Duration object')
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec)

    def get_c_duration(self):
        return self._duration_handle
