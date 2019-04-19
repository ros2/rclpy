# Copyright 2019 Open Source Robotics Foundation, Inc.
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
from rclpy.impl.implementation_singleton import rclpy_signal_handler_implementation as _signals
from rclpy.utilities import get_default_context


class SignalHandlerGuardCondition:

    def __init__(self, context=None):
        if context is None:
            context = get_default_context()
        self.guard_handle, _ = _rclpy.rclpy_create_guard_condition(context.handle)
        _signals.rclpy_register_sigint_guard_condition(self.guard_handle)

    def __del__(self):
        self.destroy()

    def destroy(self):
        if self.guard_handle is None:
            return
        _signals.rclpy_unregister_sigint_guard_condition(self.guard_handle)
        _rclpy.rclpy_destroy_entity(self.guard_handle)
        self.guard_handle = None
