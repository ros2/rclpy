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
import time

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.task import Future


class Client:
    def __init__(
            self, node_handle, context, client_handle, client_pointer,
            srv_type, srv_name, qos_profile, callback_group):
        self.node_handle = node_handle
        self.context = context
        self.client_handle = client_handle
        self.client_pointer = client_pointer
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.qos_profile = qos_profile
        # Key is a sequence number, value is an instance of a Future
        self._pending_requests = {}
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False

    def call(self, req):
        """
        Make a service request and wait for the result.

        Do not call this method in a callback or a deadlock may occur.

        :param req: The service request
        :return: The service response
        """
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        future = self.call_async(req)
        future.add_done_callback(unblock)

        event.wait()
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    def remove_pending_request(self, future):
        """
        Remove a future from the list of pending requests.

        This prevents a future from receiving a request and executing its done callbacks.
        :param future: a future returned from :meth:`call_async`
        :type future: rclpy.task.Future
        """
        for seq, req_future in self._pending_requests.items():
            if future == req_future:
                try:
                    del self._pending_requests[seq]
                except KeyError:
                    pass
                break

    def call_async(self, req):
        """
        Make a service request and asyncronously get the result.

        :return: a Future instance that completes when the request does
        :rtype: :class:`rclpy.task.Future` instance
        """
        sequence_number = _rclpy.rclpy_send_request(self.client_handle, req)
        if sequence_number in self._pending_requests:
            raise RuntimeError('Sequence (%r) conflicts with pending request' % sequence_number)

        future = Future()
        self._pending_requests[sequence_number] = future

        future.add_done_callback(self.remove_pending_request)

        return future

    def service_is_ready(self):
        return _rclpy.rclpy_service_server_is_available(self.node_handle, self.client_handle)

    def wait_for_service(self, timeout_sec=None):
        # TODO(sloretz) Return as soon as the service is available
        # This is a temporary implementation. The sleep time is arbitrary.
        sleep_time = 0.25
        if timeout_sec is None:
            timeout_sec = float('inf')
        while self.context.ok() and not self.service_is_ready() and timeout_sec > 0.0:
            time.sleep(sleep_time)
            timeout_sec -= sleep_time

        return self.service_is_ready()
