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
from typing import Optional
from typing import TypeVar

from rclpy.callback_groups import CallbackGroup
from rclpy.clock import Clock
from rclpy.context import Context
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.service_introspection import ServiceIntrospectionState
from rclpy.task import Future

# Used for documentation purposes only
SrvType = TypeVar('SrvType')
SrvTypeRequest = TypeVar('SrvTypeRequest')
SrvTypeResponse = TypeVar('SrvTypeResponse')


class Client:
    def __init__(
        self,
        context: Context,
        client_impl: _rclpy.Client,
        srv_type: SrvType,
        srv_name: str,
        qos_profile: QoSProfile,
        callback_group: CallbackGroup
    ) -> None:
        """
        Create a container for a ROS service client.

        .. warning:: Users should not create a service client with this constructor, instead they
           should call :meth:`.Node.create_client`.

        :param context: The context associated with the service client.
        :param client_impl: :class:`_rclpy.Client` wrapping the underlying ``rcl_client_t`` object.
        :param srv_type: The service type.
        :param srv_name: The name of the service.
        :param qos_profile: The quality of service profile to apply the service client.
        :param callback_group: The callback group for the service client. If ``None``, then the
            nodes default callback group is used.
        """
        self.context = context
        self.__client = client_impl
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.qos_profile = qos_profile
        # Key is a sequence number, value is an instance of a Future
        self._pending_requests: Dict[int, Future] = {}
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False

        self._lock = threading.Lock()

    def call(
        self,
        request: SrvTypeRequest,
        timeout_sec: Optional[float] = None
    ) -> Optional[SrvTypeResponse]:
        """
        Make a service request and wait for the result.

        .. warning:: Do not call this method in a callback, or a deadlock or timeout may occur.

        :param request: The service request.
        :param timeout_sec: Seconds to wait. If ``None``, then wait forever.
        :return: The service response, or None if timed out.
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

        # Check future.done() before waiting on the event.
        # The callback might have been added after the future is completed,
        # resulting in the event never being set.
        if not future.done():
            if not event.wait(timeout_sec):
                # Timed out. remove_pending_request() to free resources
                self.remove_pending_request(future)
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    def call_async(self, request: SrvTypeRequest) -> Future:
        """
        Make a service request and asynchronously get the result.

        :param request: The service request.
        :return: A future that completes when the request does.
        :raises: TypeError if the type of the passed request isn't an instance
          of the Request type of the provided service when the client was
          constructed.
        """
        if not isinstance(request, self.srv_type.Request):
            raise TypeError()

        with self._lock:
            with self.handle:
                sequence_number = self.__client.send_request(request)
            if sequence_number in self._pending_requests:
                raise RuntimeError(f'Sequence ({sequence_number}) conflicts with pending request')

            future = Future()
            self._pending_requests[sequence_number] = future

            future.add_done_callback(self.remove_pending_request)

        return future

    def get_pending_request(self, sequence_number: int) -> Future:
        """
        Get a future from the list of pending requests.

        :param sequence_number: Number identifying the pending request.
        :return: The future corresponding to the sequence_number.
        :raises: KeyError if the sequence_number is not in the pending requests.
        """
        with self._lock:
            return self._pending_requests[sequence_number]

    def remove_pending_request(self, future: Future) -> None:
        """
        Remove a future from the list of pending requests.

        This prevents a future from receiving a response and executing its done callbacks.

        :param future: A future returned from :meth:`call_async`
        """
        with self._lock:
            for seq, req_future in self._pending_requests.items():
                if future is req_future:
                    del self._pending_requests[seq]
                    break

    def service_is_ready(self) -> bool:
        """
        Check if there is a service server ready.

        :return: ``True`` if a server is ready, ``False`` otherwise.
        """
        with self.handle:
            return self.__client.service_server_is_available()

    def wait_for_service(self, timeout_sec: Optional[float] = None) -> bool:
        """
        Wait for a service server to become ready.

        Returns as soon as a server becomes ready or if the timeout expires.

        :param timeout_sec: Seconds to wait. If ``None``, then wait forever.
        :return: ``True`` if server became ready while waiting or ``False`` on a timeout.
        """
        # TODO(sloretz) Return as soon as the service is available
        # This is a temporary implementation. The sleep time is arbitrary.
        # https://github.com/ros2/rclpy/issues/58
        sleep_time = 0.25
        if timeout_sec is None:
            timeout_sec = float('inf')
        while self.context.ok() and not self.service_is_ready() and timeout_sec > 0.0:
            time.sleep(sleep_time)
            timeout_sec -= sleep_time

        return self.service_is_ready()

    def configure_introspection(
        self, clock: Clock,
        service_event_qos_profile: QoSProfile,
        introspection_state: ServiceIntrospectionState
    ) -> None:
        """
        Configure client introspection.

        :param clock: Clock to use for generating timestamps.
        :param service_event_qos_profile: QoSProfile to use when creating service event publisher.
        :param introspection_state: ServiceIntrospectionState to set introspection.
        """
        with self.handle:
            self.__client.configure_introspection(clock.handle,
                                                  service_event_qos_profile.get_c_qos_profile(),
                                                  introspection_state)

    @property
    def handle(self):
        return self.__client

    @property
    def service_name(self) -> str:
        with self.handle:
            return self.__client.service_name

    def destroy(self):
        """
        Destroy a container for a ROS service client.

        .. warning:: Users should not destroy a service client with this destructor, instead they
           should call :meth:`.Node.destroy_client`.
        """
        self.__client.destroy_when_not_in_use()
