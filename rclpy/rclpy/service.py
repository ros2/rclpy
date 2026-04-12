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

from types import TracebackType
from typing import Awaitable, Callable
from typing import Generic
from typing import Optional
from typing import Type
from typing import TypeVar
from typing import Union

from rclpy.callback_groups import CallbackGroup
from rclpy.clock import Clock
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.service_introspection import ServiceIntrospectionState
from rclpy.type_support import Srv, SrvRequestT, SrvResponseT
from typing_extensions import Self
from typing_extensions import TypeAlias

# Used for documentation purposes only
SrvType = TypeVar('SrvType')
SrvTypeRequest = TypeVar('SrvTypeRequest')
SrvTypeResponse = TypeVar('SrvTypeResponse')

GenericServiceCallback: TypeAlias = Callable[[SrvRequestT, SrvResponseT], SrvResponseT]
AsyncGenericServiceCallback: TypeAlias = Callable[
    [SrvRequestT, SrvResponseT],
    Awaitable[SrvResponseT]
]
ServiceCallbackUnion: TypeAlias = Union[
    GenericServiceCallback[SrvRequestT, SrvResponseT],
    AsyncGenericServiceCallback[SrvRequestT, SrvResponseT]
]


class BaseService(Generic[SrvRequestT, SrvResponseT]):

    def __init__(
        self,
        service_impl: '_rclpy.Service[SrvRequestT, SrvResponseT]',
        srv_type: type[Srv[SrvRequestT, SrvResponseT]],
        srv_name: str,
        callback: ServiceCallbackUnion[SrvRequestT, SrvResponseT],
        qos_profile: QoSProfile,
        *,
        on_destroy: Optional[Callable[['BaseService[SrvRequestT, SrvResponseT]'], None]] = None,
    ) -> None:
        self.__service = service_impl
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.callback = callback
        self.qos_profile = qos_profile
        self._on_destroy = on_destroy
        self._destroyed = False

    def _send_response(
        self,
        response: SrvResponseT,
        header: Union[_rclpy.rmw_service_info_t, _rclpy.rmw_request_id_t]
    ) -> None:
        if not isinstance(response, self.srv_type.Response):
            raise TypeError()
        with self.handle:
            if isinstance(header, _rclpy.rmw_service_info_t):
                self.__service.service_send_response(response, header.request_id)
            elif isinstance(header, _rclpy.rmw_request_id_t):
                self.__service.service_send_response(response, header)
            else:
                raise TypeError()

    def configure_introspection(
        self, clock: Clock,
        service_event_qos_profile: QoSProfile,
        introspection_state: ServiceIntrospectionState
    ) -> None:
        """
        Configure service introspection.

        :param clock: Clock to use for generating timestamps.
        :param service_event_qos_profile: QoSProfile to use when creating service event publisher.
        :param introspection_state: ServiceIntrospectionState to set introspection.
        """
        with self.handle:
            self.__service.configure_introspection(clock.handle,
                                                   service_event_qos_profile.get_c_qos_profile(),
                                                   introspection_state)

    @property
    def handle(self) -> '_rclpy.Service[SrvRequestT, SrvResponseT]':
        return self.__service

    @property
    def service_name(self) -> str:
        with self.handle:
            return self.__service.name

    @property
    def logger_name(self) -> str:
        """Get the name of the logger associated with the node of the service."""
        with self.handle:
            return self.__service.get_logger_name()

    def destroy(self) -> None:
        """Destroy the service, notifying the owning node and releasing the handle."""
        if self._destroyed:
            return
        self._destroyed = True
        if self._on_destroy is not None:
            self._on_destroy(self)
            self._on_destroy = None
        self._destroy()

    def _destroy(self) -> None:
        self.handle.destroy_when_not_in_use()

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> None:
        self.destroy()


class Service(BaseService[SrvRequestT, SrvResponseT], Generic[SrvRequestT, SrvResponseT]):
    def __init__(
        self,
        service_impl: '_rclpy.Service[SrvRequestT, SrvResponseT]',
        srv_type: type[Srv[SrvRequestT, SrvResponseT]],
        srv_name: str,
        callback: ServiceCallbackUnion[SrvRequestT, SrvResponseT],
        qos_profile: QoSProfile,
        *,
        on_destroy: Optional[Callable[['Service[SrvRequestT, SrvResponseT]'], None]] = None,
        callback_group: CallbackGroup
    ) -> None:
        """
        Create a container for a ROS service server.

        .. warning:: Users should not create a service server with this constructor, instead they
           should call :meth:`.Node.create_service`.

        :param service_impl: :class:`_rclpy.Service` wrapping the underlying ``rcl_service_t``
            object.
        :param srv_type: The service type.
        :param srv_name: The name of the service.
        :param callback: The callback that should be called to handle the request.
        :param callback_group: The callback group for the service server. If ``None``, then the
            nodes default callback group is used.
        :param qos_profile: The quality of service profile to apply the service server.
        """
        super().__init__(
            service_impl=service_impl,
            srv_type=srv_type,
            srv_name=srv_name,
            callback=callback,
            qos_profile=qos_profile,
            on_destroy=on_destroy
        )
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False

    def send_response(
        self,
        response: SrvResponseT,
        header: Union[_rclpy.rmw_service_info_t, _rclpy.rmw_request_id_t]
    ) -> None:
        """
        Send a service response.

        :param response: The service response.
        :param header: Service header from the original request.
        :raises: TypeError if the type of the passed response isn't an instance
          of the Response type of the provided service when the service was
          constructed.
        """
        self._send_response(response, header)
