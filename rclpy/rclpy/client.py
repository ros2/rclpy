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
from typing import Dict
from typing import TypeVar

from rclpy.callback_groups import CallbackGroup
from rclpy.context import Context
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.task import Future

# Used for documentation purposes only
SrvType = TypeVar('SrvType')
SrvTypeRequest = TypeVar('SrvTypeRequest')
SrvTypeResponse = TypeVar('SrvTypeResponse')


class Client:
    def __init__(
        self,
        context: Context,
        client_handle,
        srv_type: SrvType,
        srv_name: str,
        qos_profile: QoSProfile,
        callback_group: CallbackGroup
    ) -> None:
        """
        Create a container for a ROS service client.

        .. warning:: Users should not create a service client with this constuctor, instead they
           should call :meth:`.Node.create_client`.

        :param context: The context associated with the service client.
        :param client_handle: :class:`Handle` wrapping the underlying ``rcl_client_t`` object.
        :param srv_type: The service type.
        :param srv_name: The name of the service.
        :param qos_profile: The quality of service profile to apply the service client.
        :param callback_group: The callback group for the service client. If ``None``, then the
            nodes default callback group is used.
        """
        self.context = context
        self.__handle = client_handle
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.qos_profile = qos_profile
        # Key is a sequence number, value is an instance of a Future
        self._pending_requests: Dict[int, Future] = {}
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False

    def call(self, request: SrvTypeRequest) -> SrvTypeResponse:
        """
        Make a service request and wait for the result.

        .. warning:: Do not call this method in a callback or a deadlock may occur.

        :param request: The service request.
        :return: The service response.
        :raises: TypeError if the type of the passed request isn't an instance
          of the Request type of the provided service when the client was
          constructed.
        """
        if not isinstance(request, self.srv_type.Request):
            raise TypeError()

        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        future = self.call_async(request)
        future.add_done_callback(unblock)

        event.wait()
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    def remove_pending_request(self, future: Future) -> None:
        """
        Remove a future from the list of pending requests.

        This prevents a future from receiving a response and executing its done callbacks.

        :param future: A future returned from :meth:`call_async`
        """
        for seq, req_future in self._pending_requests.items():
            if future == req_future:
                try:
                    del self._pending_requests[seq]
                except KeyError:
                    pass
                break

    def call_async(self, request: SrvTypeRequest) -> Future:
        """
        Make a service request and asyncronously get the result.

        :param request: The service request.
        :return: A future that completes when the request does.
        :raises: TypeError if the type of the passed request isn't an instance
          of the Request type of the provided service when the client was
          constructed.
        """
        if not isinstance(request, self.srv_type.Request):
            raise TypeError()

        with self.handle as capsule:
            sequence_number = _rclpy.rclpy_send_request(capsule, request)
        if sequence_number in self._pending_requests:
            raise RuntimeError('Sequence (%r) conflicts with pending request' % sequence_number)

        future = Future()
        self._pending_requests[sequence_number] = future

        future.add_done_callback(self.remove_pending_request)

        return future

    def service_is_ready(self) -> bool:
        """
        Check if there is a service server ready.

        :return: ``True`` if a server is ready, ``False`` otherwise.
        """
        with self.handle as capsule:
            return _rclpy.rclpy_service_server_is_available(capsule)

    def wait_for_service(self, timeout_sec: float = None) -> bool:
        """
        Wait for a service server to become ready.

        Returns as soon as a server becomes ready or if the timeout expires.

        :param timeout_sec: Seconds to wait. If ``None``, then wait forever.
        :return: ``True`` if server became ready while waiting or ``False`` on a timeout.
        """
        # TODO(sloretz) Return as soon as the service is available
        # This is a temporary implementation. The sleep time is arbitrary.
        sleep_time = 0.25
        if timeout_sec is None:
            timeout_sec = float('inf')
        while self.context.ok() and not self.service_is_ready() and timeout_sec > 0.0:
            time.sleep(sleep_time)
            timeout_sec -= sleep_time

        return self.service_is_ready()

    @property
    def handle(self):
        return self.__handle

    def destroy(self):
        self.handle.destroy()
