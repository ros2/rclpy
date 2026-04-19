# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import asyncio
from types import TracebackType
from typing import Any, Awaitable, Callable, Optional, Set, Type, Union

from rclpy.clock_type import ClockType
from rclpy.context import Context
from rclpy.node import BaseNode
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_rosout_default
from rclpy.qos import qos_profile_services_default
from rclpy.qos import QoSProfile
from rclpy.subscription import AsyncGenericSubscriptionCallback
from rclpy.subscription_content_filter_options import ContentFilterOptions
from rclpy.timer import AsyncTimerCallbackType
from rclpy.type_support import MsgT, Srv, SrvRequestT, SrvResponseT

from .async_client import AsyncClient
from .async_clock import AsyncClock
from .async_publisher import AsyncPublisher
from .async_service import AsyncService
from .async_subscription import AsyncSubscription
from .async_timer import AsyncTimer

AsyncEntity = Union[
    AsyncPublisher, AsyncSubscription, AsyncService, AsyncClient, AsyncTimer]


class AsyncNode(BaseNode):
    """
    Async node with two mutually exclusive entry points.

    .. admonition:: Experimental

       This API is experimental.

    Simple reactive node::

        node = AsyncNode('my_node')
        node.create_subscription(topic, MsgType, callback, qos)
        await node.run()

    Composable with user-controlled lifetime::

        async with AsyncNode('my_node') as node:
            node.create_subscription(topic, MsgType, callback, qos)
            await my_foreground_task()
    """

    def __init__(
        self,
        node_name: str,
        *,
        context: Optional[Context] = None,
        cli_args: Optional[list[str]] = None,
        namespace: Optional[str] = None,
        use_global_arguments: bool = True,
        enable_rosout: bool = True,
        rosout_qos_profile: Union[QoSProfile, int] = qos_profile_rosout_default,
        start_parameter_services: bool = True,
        parameter_overrides: Optional[list[Parameter[Any]]] = None,
        allow_undeclared_parameters: bool = False,
        automatically_declare_parameters_from_overrides: bool = False,
        enable_logger_service: bool = False
    ) -> None:
        """Create an async ROS node."""
        self._clock = AsyncClock(clock_type=ClockType.ROS_TIME)
        self._tg: Optional[asyncio.TaskGroup] = None
        self._entities: Set[AsyncEntity] = set()
        self._destroyed = asyncio.Event()

        super().__init__(
            node_name=node_name,
            context=context,
            cli_args=cli_args,
            namespace=namespace,
            use_global_arguments=use_global_arguments,
            enable_rosout=enable_rosout,
            rosout_qos_profile=rosout_qos_profile,
            start_parameter_services=start_parameter_services,
            parameter_overrides=parameter_overrides,
            allow_undeclared_parameters=allow_undeclared_parameters,
            automatically_declare_parameters_from_overrides=(
                automatically_declare_parameters_from_overrides),
            enable_logger_service=enable_logger_service,
        )

    async def __aenter__(self) -> 'AsyncNode':
        if self._destroyed.is_set():
            raise RuntimeError('Node has been destroyed and cannot be reentered')
        if self._tg is not None:
            raise RuntimeError('Node is already running')
        tg = asyncio.TaskGroup()
        self._tg = await tg.__aenter__()
        for entity in self._entities:
            if hasattr(entity, '_run'):
                entity._task = self._tg.create_task(entity._run())
        return self

    async def __aexit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> None:
        try:
            self.destroy_node()
            await self._tg.__aexit__(exc_type, exc_val, exc_tb)
        finally:
            self._tg = None

    def get_clock(self) -> AsyncClock:
        """Get the async clock used by the node."""
        return self._clock

    def destroy_node(self) -> None:
        if self._destroyed.is_set():
            return
        self._destroyed.set()
        self._context.untrack_node(self)
        for entity in list(self._entities):
            entity.destroy()
        self._clock._destroy()
        self.handle.destroy_when_not_in_use()

    async def wait_for_node(
        self,
        fully_qualified_node_name: str,
        *,
        check_interval: float = 0.1
    ) -> None:
        """
        Wait until node name is present in the system.

        To apply a timeout, wrap the call with ``async with asyncio.timeout()``.

        :param fully_qualified_node_name: Fully qualified name of the node to wait for.
        :param check_interval: Seconds between checks. Defaults to 0.1.
        """
        if not fully_qualified_node_name.startswith('/'):
            fully_qualified_node_name = f'/{fully_qualified_node_name}'

        while fully_qualified_node_name not in self.get_fully_qualified_node_names():
            await asyncio.sleep(check_interval)

    async def run(self) -> None:
        """
        Run the node until destroy_node() is called.

        Mutually exclusive with ``async with``. Raises RuntimeError if the
        node is already running under a context manager.
        """
        async with self:
            await self._destroyed.wait()

    def create_publisher(
        self,
        msg_type: Type[MsgT],
        topic: str,
        qos_profile: Union[QoSProfile, int],
    ) -> AsyncPublisher[MsgT]:
        if self._destroyed.is_set():
            raise RuntimeError('Cannot create publisher on a destroyed node')
        qos_profile = self._validate_qos_or_depth_parameter(qos_profile)

        publisher_handle = self._create_publisher_handle(
            msg_type, topic, qos_profile)

        pub = AsyncPublisher(
            publisher_handle,
            msg_type,
            topic,
            qos_profile,
            on_destroy=self._entities.discard,
        )
        self._entities.add(pub)
        return pub

    def create_subscription(
        self,
        msg_type: Type[MsgT],
        topic: str,
        callback: AsyncGenericSubscriptionCallback[MsgT],
        qos_profile: Union[QoSProfile, int],
        *,
        raw: bool = False,
        concurrent: bool = False,
        content_filter_options: Optional[ContentFilterOptions] = None,
    ) -> AsyncSubscription[MsgT]:
        if self._destroyed.is_set():
            raise RuntimeError('Cannot create subscription on a destroyed node')
        qos_profile = self._validate_qos_or_depth_parameter(qos_profile)

        subscription_handle = self._create_subscription_handle(
            msg_type, topic, qos_profile,
            content_filter_options=content_filter_options)

        sub = AsyncSubscription(
            subscription_handle,
            msg_type,
            topic,
            callback,
            qos_profile,
            on_destroy=self._entities.discard,
            raw=raw,
            concurrent=concurrent,
            tg=self._tg,
        )
        self._entities.add(sub)
        return sub

    def _create_service(
        self,
        service_impl: object,
        srv_type: Type[Srv[SrvRequestT, SrvResponseT]],
        srv_name: str,
        callback: Callable[[SrvRequestT, SrvResponseT], Awaitable[SrvResponseT]],
        qos_profile: QoSProfile,
        *,
        concurrent: bool = False,
    ) -> AsyncService[SrvRequestT, SrvResponseT]:
        srv = AsyncService(
            service_impl,
            srv_type,
            srv_name,
            callback,
            qos_profile,
            on_destroy=self._entities.discard,
            concurrent=concurrent,
            tg=self._tg,
        )
        self._entities.add(srv)
        return srv

    def create_service(
        self,
        srv_type: Type[Srv[SrvRequestT, SrvResponseT]],
        srv_name: str,
        callback: Callable[[SrvRequestT, SrvResponseT], Awaitable[SrvResponseT]],
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
        concurrent: bool = False,
    ) -> AsyncService[SrvRequestT, SrvResponseT]:
        if self._destroyed.is_set():
            raise RuntimeError('Cannot create service on a destroyed node')
        service_handle = self._create_service_handle(
            srv_type, srv_name, qos_profile=qos_profile)
        return self._create_service(
            service_handle, srv_type, srv_name, callback, qos_profile,
            concurrent=concurrent)

    def create_client(
        self,
        srv_type: Type[Srv[SrvRequestT, SrvResponseT]],
        srv_name: str,
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
    ) -> AsyncClient[SrvRequestT, SrvResponseT]:
        if self._destroyed.is_set():
            raise RuntimeError('Cannot create client on a destroyed node')

        client_handle = self._create_client_handle(
            srv_type, srv_name, qos_profile=qos_profile)

        client = AsyncClient(
            self.context,
            client_handle,
            srv_type,
            srv_name,
            qos_profile,
            on_destroy=self._entities.discard,
            tg=self._tg,
        )
        self._entities.add(client)
        return client

    def create_timer(
        self,
        timer_period_sec: float,
        callback: AsyncTimerCallbackType,
        autostart: bool = True
    ) -> AsyncTimer:
        if self._destroyed.is_set():
            raise RuntimeError('Cannot create timer on a destroyed node')

        timer_period_ns = int(float(timer_period_sec) * 1e9)
        timer = AsyncTimer(
            timer_period_ns,
            self._clock,
            self.context,
            callback,
            autostart,
            on_destroy=self._entities.discard,
            tg=self._tg,
        )
        self._entities.add(timer)
        return timer
