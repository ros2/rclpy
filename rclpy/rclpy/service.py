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

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class Service:
    def __init__(
            self, node_handle, service_handle, service_pointer,
            srv_type, srv_name, callback, callback_group, qos_profile):
        self.node_handle = node_handle
        self.service_handle = service_handle
        self.service_pointer = service_pointer
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False
        self.qos_profile = qos_profile

    def send_response(self, response, header):
        _rclpy.rclpy_send_response(self.service_handle, response, header)
