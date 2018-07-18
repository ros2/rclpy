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
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class Duration:

    def __init__(self, *, seconds=0, nanoseconds=0):
        if seconds < 0:
            raise ValueError('Seconds value must not be negative')
        if nanoseconds < 0:
            raise ValueError('Nanoseconds value must not be negative')
        total_nanoseconds = seconds * 1e9
        total_nanoseconds += nanoseconds
        self._duration_handle = _rclpy.rclpy_create_duration(int(total_nanoseconds))

    @property
    def nanoseconds(self):
        return _rclpy.rclpy_duration_get_nanoseconds(self._duration_handle)

    def to_msg(self):
        # TODO(dhood): break into sec and nanosec
        return builtin_interfaces.msg.Duration(nanosec=self.nanoseconds)

    @classmethod
    def from_msg(cls, msg):
        if not isinstance(msg, builtin_interfaces.msg.Duration):
            raise TypeError('Must pass a builtin_interfaces.msg.Time object')
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec)
