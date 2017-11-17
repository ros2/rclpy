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

import inspect

from rclpy.executor_handle import ExecutorHandle
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
        # Holds info the executor uses to do work for this entity
        self._executor_handle = ExecutorHandle(self._take, self._execute)
        self.qos_profile = qos_profile

    def _take(self):
        request_and_header = _rclpy.rclpy_take_request(self.service_handle, self.srv_type.Request)
        return request_and_header

    def _execute(self, request_and_header):
        if request_and_header is None:
            return
        (request, header) = request_and_header
        if request:

            async def exec_service(request, header):
                """Enable service callbacks to be coroutines."""
                nonlocal self
                response = self.callback(request, self.srv_type.Response())
                if inspect.isawaitable(response):
                    response = await response
                # This special method is needed to make sure send_response gets called after the
                # user's callback even if it yields
                self.send_response(response, header)

            return exec_service(request, header)

    def send_response(self, response, header):
        _rclpy.rclpy_send_response(self.service_handle, response, header)
