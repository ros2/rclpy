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

from rclpy.executor_handle import ExecutorHandle
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class GuardCondition:

    def __init__(self, callback, callback_group):
        self.guard_handle, self.guard_pointer = _rclpy.rclpy_create_guard_condition()
        self.callback = callback
        self.callback_group = callback_group
        # Holds info the executor uses to do work for this entity
        self._executor_handle = ExecutorHandle(
            self._take, self._execute, cancel_ready_callback=self._cancel_ready)

    def _take(self):
        pass

    def _execute(self, _):
        return self.callback()

    def _cancel_ready(self):
        # guard conditions are auto-taken when they come up in the wait-list, so retrigger
        self.trigger()

    def trigger(self):
        _rclpy.rclpy_trigger_guard_condition(self.guard_handle)
