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

import rclpy


class Client:
    def __init__(self, node_handle, client_handle, client_pointer, srv_type, srv_name, qos_profile):
        self.node_handle = node_handle
        self.client_handle = client_handle
        self.client_pointer = client_pointer
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.qos_profile = qos_profile
        self.sequence_number = 0
        self.response = None

    def call(self, req):
        self.sequence_number = rclpy._rclpy.rclpy_send_request(self.client_handle, req)

    def __del__(self):
        rclpy._rclpy.rclpy_destroy_entity(
            'client', self.client_handle, self.node_handle)
