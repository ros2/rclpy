# Copyright 2016 Open Source Robotics Foundation, Inc.
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


class Subscription:

    def __init__(
            self, subscription_handle, subscription_pointer,
            msg_type, topic, callback, callback_group, qos_profile, node_handle):
        self.node_handle = node_handle
        self.subscription_handle = subscription_handle
        self.subscription_pointer = subscription_pointer
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.callback_group = callback_group
        # Holds info the executor uses to do work for this entity
        self._executor_handle = ExecutorHandle(self._take, self._execute)
        self.qos_profile = qos_profile

    def _take(self):
        msg = _rclpy.rclpy_take(self.subscription_handle, self.msg_type)
        return msg

    def _execute(self, msg):
        if msg:
            return self.callback(msg)
