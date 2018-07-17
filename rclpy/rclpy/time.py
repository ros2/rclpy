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


class Time:

    # TODO(dhood): specify clock type
    # TODO(dhood): convenience functions so not always ns
    def __init__(self, nanoseconds):
        if nanoseconds < 0:
            raise ValueError('Nanoseconds value must not be negative')
        self.time_handle = _rclpy.rclpy_create_time_point(nanoseconds)

    @property
    def nanoseconds(self):
        return _rclpy.rclpy_get_time_point_nanoseconds(self.time_handle)
