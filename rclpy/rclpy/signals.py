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

from rclpy.exceptions import InvalidHandle
from rclpy.guard_condition import GuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class SignalHandlerGuardCondition(GuardCondition):

    def __init__(self, context=None):
        super().__init__(callback=None, callback_group=None, context=context)
        with self.handle:
            _rclpy.register_sigint_guard_condition(self.handle)

    def __del__(self):
        try:
            self.destroy()
        except InvalidHandle:
            # already destroyed
            pass
        except ValueError:
            # Guard condition was not registered
            pass

    def destroy(self):
        with self.handle:
            _rclpy.unregister_sigint_guard_condition(self.handle)
        super().destroy()
