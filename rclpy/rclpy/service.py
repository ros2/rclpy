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

from typing import Callable
from typing import TypeVar

from rclpy.callback_groups import CallbackGroup
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile

# Used for documentation purposes only
SrvType = TypeVar('SrvType')
SrvTypeRequest = TypeVar('SrvTypeRequest')
SrvTypeResponse = TypeVar('SrvTypeResponse')


class Service:
    def __init__(
        self,
        service_handle,
        srv_type: SrvType,
        srv_name: str,
        callback: Callable[[SrvTypeRequest, SrvTypeResponse], SrvTypeResponse],
        callback_group: CallbackGroup,
        qos_profile: QoSProfile
    ) -> None:
        """
        Create a container for a ROS service server.

        .. warning:: Users should not create a service server with this constuctor, instead they
           should call :meth:`.Node.create_service`.

        :param context: The context associated with the service server.
        :param service_handle: Capsule pointing to the underlying ``rcl_service_t`` object.
        :param srv_type: The service type.
        :param srv_name: The name of the service.
        :param callback_group: The callback group for the service server. If ``None``, then the
            nodes default callback group is used.
        :param qos_profile: The quality of service profile to apply the service server.
        """
        self.__handle = service_handle
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False
        self.qos_profile = qos_profile

    def send_response(self, response: SrvTypeResponse, header) -> None:
        """
        Send a service response.

        :param response: The service response.
        :param header: Capsule pointing to the service header from the original request.
        :raises: TypeError if the type of the passed response isn't an instance
          of the Response type of the provided service when the service was
          constructed.
        """
        if not isinstance(response, self.srv_type.Response):
            raise TypeError()
        with self.handle as capsule:
            _rclpy.rclpy_send_response(capsule, response, header)

    @property
    def handle(self):
        return self.__handle

    def destroy(self):
        self.handle.destroy()
