# Copyright 2017 Open Source Robotics Foundation, Inc.
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


class GuardCondition:

    def __init__(self, callback, callback_group):
        self.guard_handle, self.guard_pointer = _rclpy.rclpy_create_guard_condition()
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False
        # True when the executor sees this has been triggered but has not yet been handled
        self._executor_triggered = False

    def trigger(self):
        _rclpy.rclpy_trigger_guard_condition(self.guard_handle)
