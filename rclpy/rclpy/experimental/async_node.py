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
from typing import Any, Literal, Optional, overload, Type, Union

from rclpy.clock_type import ClockType
from rclpy.context import Context
from rclpy.node import BaseNode
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_rosout_default
from rclpy.qos import qos_profile_services_default
from rclpy.qos import QoSProfile
from rclpy.service import ServiceCallbackUnion
from rclpy.subscription import GenericSubscriptionCallbackUnion, SubscriptionCallbackUnion
from rclpy.subscription_content_filter_options import ContentFilterOptions
from rclpy.timer import TimerCallbackUnion
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
    A Node in the ROS graph that runs on an asyncio event loop.

    Like :class:`Node`, ``AsyncNode`` is the primary entrypoint for communication — it creates
    publishers, subscriptions, services, clients, and timers. Unlike :class:`Node`, it does not
    require an rclpy executor: the node owns its own :class:`asyncio.TaskGroup`, and DDS callbacks
    are dispatched directly onto the running event loop.

    .. admonition:: Experimental

       This API is experimental.

    Example::

        async with AsyncNode('my_node') as node:
            node.create_subscription(String, '/topic', callback, 10)
            await my_async_task()

    See :meth:`run` for a shorthand when the node is the only foreground task.
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
        """
        Create an AsyncNode.

        :param node_name: A name to give to this node. Validated by :func:`validate_node_name`.
        :param context: The context to be associated with, or ``None`` for the default global
            context.
        :param cli_args: A list of strings of command line args to be used only by this node.
            These arguments are used to extract remappings used by the node and other ROS specific
            settings, as well as user defined non-ROS arguments.
        :param namespace: The namespace to which relative topic and service names will be prefixed.
            Validated by :func:`validate_namespace`.
        :param use_global_arguments: ``False`` if the node should ignore process-wide command line
            args.
        :param enable_rosout: ``False`` if the node should ignore rosout logging.
        :param rosout_qos_profile: A QoSProfile or a history depth to apply to rosout publisher.
            In the case that a history depth is provided, the QoS history is set to KEEP_LAST,
            the QoS history depth is set to the value of the parameter,
            and all other QoS settings are set to their default value.
        :param start_parameter_services: ``False`` if the node should not create parameter
            services.
        :param parameter_overrides: A list of overrides for initial values for parameters declared
            on the node.
        :param allow_undeclared_parameters: True if undeclared parameters are allowed.
            This flag affects the behavior of parameter-related operations.
        :param automatically_declare_parameters_from_overrides: If True, the "parameter overrides"
            will be used to implicitly declare parameters on the node during creation.
        :param enable_logger_service: ``True`` if ROS2 services are created to allow external nodes
            to get and set logger levels of this node. Otherwise, logger levels are only managed
            locally. That is, logger levels cannot be changed remotely.
        """
        self._clock = AsyncClock(clock_type=ClockType.ROS_TIME)
        self._tg: Optional[asyncio.TaskGroup] = None
        self._entities: set[AsyncEntity] = set()
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
        """Destroy the node."""
        if self._destroyed.is_set():
            return
        self._destroyed.set()
        for entity in list(self._entities):
            entity.destroy()
        self._clock._destroy()
        super().destroy_node()

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

        Mutually exclusive with ``async with``.

        :raises RuntimeError: If the node is already running under a context manager.
        """
        async with self:
            await self._destroyed.wait()

    def create_publisher(
        self,
        msg_type: Type[MsgT],
        topic: str,
        qos_profile: Union[QoSProfile, int],
    ) -> AsyncPublisher[MsgT]:
        """
        Create a new publisher.

        :param msg_type: The type of ROS messages the publisher will publish.
        :param topic: The name of the topic the publisher will publish to.
        :param qos_profile: A QoSProfile or a history depth to apply to the publisher.
            In the case that a history depth is provided, the QoS history is set to
            KEEP_LAST, the QoS history depth is set to the value of the parameter,
            and all other QoS settings are set to their default values.
        :return: The new publisher.
        :raises RuntimeError: If the node has been destroyed.
        """
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

    @overload
    def create_subscription(
        self,
        msg_type: Type[MsgT],
        topic: str,
        callback: GenericSubscriptionCallbackUnion[bytes],
        qos_profile: Union[QoSProfile, int],
        *,
        raw: Literal[True],
        concurrent: bool = False,
        content_filter_options: Optional[ContentFilterOptions] = None,
    ) -> AsyncSubscription[MsgT]: ...

    @overload
    def create_subscription(
        self,
        msg_type: Type[MsgT],
        topic: str,
        callback: GenericSubscriptionCallbackUnion[MsgT],
        qos_profile: Union[QoSProfile, int],
        *,
        raw: Literal[False],
        concurrent: bool = False,
        content_filter_options: Optional[ContentFilterOptions] = None,
    ) -> AsyncSubscription[MsgT]: ...

    @overload
    def create_subscription(
        self,
        msg_type: Type[MsgT],
        topic: str,
        callback: SubscriptionCallbackUnion[MsgT],
        qos_profile: Union[QoSProfile, int],
        *,
        raw: bool = False,
        concurrent: bool = False,
        content_filter_options: Optional[ContentFilterOptions] = None,
    ) -> AsyncSubscription[MsgT]: ...

    def create_subscription(
        self,
        msg_type: Type[MsgT],
        topic: str,
        callback: SubscriptionCallbackUnion[MsgT],
        qos_profile: Union[QoSProfile, int],
        *,
        raw: bool = False,
        concurrent: bool = False,
        content_filter_options: Optional[ContentFilterOptions] = None,
    ) -> AsyncSubscription[MsgT]:
        """
        Create a new subscription.

        :param msg_type: The type of ROS messages the subscription will subscribe to.
        :param topic: The name of the topic the subscription will subscribe to.
        :param callback: A user-defined callback invoked for each received message. May be a
            sync ``def`` or ``async def`` function.
        :param qos_profile: A QoSProfile or a history depth to apply to the subscription.
            In the case that a history depth is provided, the QoS history is set to
            KEEP_LAST, the QoS history depth is set to the value of the parameter,
            and all other QoS settings are set to their default values.
        :param raw: If ``True``, received messages will be delivered in raw binary
            representation.
        :param concurrent: If ``True``, each message is taken as soon as it arrives and its
            callback is dispatched as an independent task, so callbacks may run concurrently.
            If ``False`` (default), the next message is not taken from DDS until the previous
            callback has finished — callback execution back-pressures the read loop.
        :param content_filter_options: The filter expression and parameters for content filtering.
        :return: The new subscription.
        :raises RuntimeError: If the node has been destroyed.
        """
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
        callback: ServiceCallbackUnion[SrvRequestT, SrvResponseT],
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
        callback: ServiceCallbackUnion[SrvRequestT, SrvResponseT],
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
        concurrent: bool = False,
    ) -> AsyncService[SrvRequestT, SrvResponseT]:
        """
        Create a new service server.

        :param srv_type: The service type.
        :param srv_name: The name of the service.
        :param callback: A user-defined callback invoked for each received request; its
            return value is sent as the response. May be a sync ``def`` or ``async def``
            function.
        :param qos_profile: The quality of service profile to apply to the service server.
        :param concurrent: If ``True``, each request is taken as soon as it arrives and its
            handler is dispatched as an independent task, so handlers may run concurrently.
            If ``False`` (default), the next request is not taken from DDS until the previous
            handler has finished — handler execution back-pressures the read loop.
        :return: The new service.
        :raises RuntimeError: If the node has been destroyed.
        """
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
        """
        Create a new service client.

        :param srv_type: The service type.
        :param srv_name: The name of the service.
        :param qos_profile: The quality of service profile to apply to the service client.
        :return: The new client.
        :raises RuntimeError: If the node has been destroyed.
        """
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
        callback: TimerCallbackUnion,
        autostart: bool = True
    ) -> AsyncTimer:
        """
        Create a new timer.

        If autostart is ``True`` (the default), the timer will be started and every
        ``timer_period_sec`` number of seconds the provided callback will be invoked.
        If autostart is ``False``, the timer will be created but not started; it can
        then be started by calling ``reset()`` on the timer object.

        :param timer_period_sec: The period (in seconds) of the timer.
        :param callback: A user-defined callback invoked when the timer expires. May be a
            sync ``def`` or ``async def`` function.
        :param autostart: Whether to automatically start the timer after creation; defaults to
            ``True``.
        :return: The new timer.
        :raises RuntimeError: If the node has been destroyed.
        """
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
