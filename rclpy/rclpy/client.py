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
from rclpy.impl.implementation_singleton import rclpy_wait_set_implementation as _rclpy_wait_set
import rclpy.utilities


class ResponseThread(threading.Thread):

    def __init__(self, client):
        threading.Thread.__init__(self)
        self.client = client
        self._wait_set = _rclpy_wait_set.WaitSet()

    def run(self):
        [sigint_gc, sigint_gc_handle] = _rclpy.rclpy_get_sigint_guard_condition()
        self._wait_set.clear()
        self._wait_set.add_guard_conditions([sigint_gc])
        self._wait_set.add_clients([self.client.client_handle])

        self._wait_set.wait(-1)

        # destroying here to make sure we dont call shutdown before cleaning up
        _rclpy.rclpy_destroy_entity(sigint_gc)
        if self._wait_set.is_ready(sigint_gc):
            rclpy.utilities.shutdown()
            return
        response = _rclpy.rclpy_take_response(
            self.client.client_handle,
            self.client.srv_type.Response,
            self.client.sequence_number)
        if response:
            self.client.response = response


class Client:
    def __init__(
            self, node_handle, client_handle, client_pointer,
            srv_type, srv_name, qos_profile, callback_group):
        self.node_handle = node_handle
        self.client_handle = client_handle
        self.client_pointer = client_pointer
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.qos_profile = qos_profile
        self.sequence_number = 0
        self.response = None
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False

    def call(self, req):
        self.response = None
        self.sequence_number = _rclpy.rclpy_send_request(self.client_handle, req)

    # TODO(mikaelarguedas) this function can only be used if nobody is spinning
    # need to be updated once guard_conditions are supported
    def wait_for_future(self):
        thread1 = ResponseThread(self)
        thread1.start()
        thread1.join()
