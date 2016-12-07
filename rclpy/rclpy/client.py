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

import threading

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class ResponseThread(threading.Thread):
    def __init__(self, client):
        threading.Thread.__init__(self)
        self.client = client
        self.wait_set = _rclpy.rclpy_get_zero_initialized_wait_set()
        _rclpy.rclpy_wait_set_init(self.wait_set, 0, 0, 0, 1, 0)
        _rclpy.rclpy_wait_set_clear_entities('client', self.wait_set)
        _rclpy.rclpy_wait_set_add_entity(
            'client', self.wait_set, self.client.client_handle)

    def run(self):
        _rclpy.rclpy_wait(self.wait_set, -1)
        response = _rclpy.rclpy_take_response(
            self.client.client_handle, self.client.srv_type.Response, self.client.sequence_number)
        if response:
            self.client.response = response


class Client:
    def __init__(
            self, node_handle, client_handle, client_pointer,
            srv_type, srv_name, qos_profile):
        self.node_handle = node_handle
        self.client_handle = client_handle
        self.client_pointer = client_pointer
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.qos_profile = qos_profile
        self.sequence_number = 0
        self.response = None

    def call(self, req):
        self.response = None
        self.sequence_number = _rclpy.rclpy_send_request(self.client_handle, req)

    # TODO(mikaelarguedas) this function can only be used if nobody is spinning
    # need to be updated once guard_conditions are supported
    def wait_for_future(self):
        thread1 = ResponseThread(self)
        thread1.start()
        thread1.join()
