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

from __future__ import annotations

from enum import Enum
import inspect
from types import TracebackType
from typing import Awaitable
from typing import Callable
from typing import Generic
from typing import Literal
from typing import Optional
from typing import overload
from typing import Type
from typing import TYPE_CHECKING
from typing import TypedDict
from typing import TypeVar
from typing import Union


from rclpy.callback_groups import CallbackGroup
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.subscription_content_filter_options import ContentFilterOptions
from rclpy.type_support import MsgT
from typing_extensions import Self
from typing_extensions import TypeAlias

if TYPE_CHECKING:
    from typing_extensions import TypeIs


class PublisherGID(TypedDict):
    implementation_identifier: str
    data: bytes


class MessageInfo(TypedDict):
    source_timestamp: int
    received_timestamp: int
    publication_sequence_number: Optional[int]
    reception_sequence_number: Optional[int]
    publisher_gid: Optional[PublisherGID]


# Re-export exception defined in _rclpy C extension.
RCLError = _rclpy.RCLError

# Left to support Legacy TypeVars.
MsgType = TypeVar('MsgType')

# Can be redone with TypeVar(default=MsgT) when either typing-extensions4.11.0+ or python3.13+
T = TypeVar('T')

GenericSubscriptionCallback: TypeAlias = Union[Callable[[T], None],
                                               Callable[[T, MessageInfo], None]]
AsyncGenericSubscriptionCallback: TypeAlias = Union[Callable[[T], Awaitable[None]],
                                                    Callable[[T, MessageInfo], Awaitable[None]]]
GenericSubscriptionCallbackUnion: TypeAlias = Union[GenericSubscriptionCallback[T],
                                                    AsyncGenericSubscriptionCallback[T]]
SubscriptionCallbackUnion: TypeAlias = Union[GenericSubscriptionCallbackUnion[MsgT],
                                             GenericSubscriptionCallbackUnion[bytes]]

SubscriptionCallback: TypeAlias = Callable[[MsgT | bytes], None] | \
                                  Callable[[MsgT | bytes], Awaitable[None]]
MessageAndInfo: TypeAlias = tuple[MsgT | bytes, MessageInfo]
SubscriptionCallbackWithMessageInfo: TypeAlias = Callable[[*MessageAndInfo[MsgT]], None] | \
                                                 Callable[[*MessageAndInfo[MsgT]], Awaitable[None]]


class BaseSubscription(Generic[MsgT]):

    class CallbackType(Enum):
        MessageOnly = 0
        WithMessageInfo = 1

    @overload
    def __init__(
         self,
         subscription_impl: '_rclpy.Subscription[MsgT]',
         msg_type: Type[MsgT],
         topic: str,
         callback: GenericSubscriptionCallbackUnion[bytes],
         qos_profile: QoSProfile,
         raw: Literal[True],
         *,
         on_destroy: Optional[Callable[[Self], None]] = None,
    ) -> None: ...

    @overload
    def __init__(
         self,
         subscription_impl: '_rclpy.Subscription[MsgT]',
         msg_type: Type[MsgT],
         topic: str,
         callback: GenericSubscriptionCallbackUnion[MsgT],
         qos_profile: QoSProfile,
         raw: Literal[False],
         *,
         on_destroy: Optional[Callable[[Self], None]] = None,
    ) -> None: ...

    @overload
    def __init__(
         self,
         subscription_impl: '_rclpy.Subscription[MsgT]',
         msg_type: Type[MsgT],
         topic: str,
         callback: SubscriptionCallbackUnion[MsgT],
         qos_profile: QoSProfile,
         raw: bool,
         *,
         on_destroy: Optional[Callable[[Self], None]] = None,
    ) -> None: ...

    def __init__(
         self,
         subscription_impl: '_rclpy.Subscription[MsgT]',
         msg_type: Type[MsgT],
         topic: str,
         callback: SubscriptionCallbackUnion[MsgT],
         qos_profile: QoSProfile,
         raw: bool,
         *,
         on_destroy: Optional[Callable[[Self], None]] = None,
    ) -> None:
        """
        Create a container for a ROS subscription.

        .. warning:: Users should not create a subscription with this constructor, instead they
           should call :meth:`.Node.create_subscription` or :meth:`.AsyncNode.create_subscription`.
        """
        self.__subscription = subscription_impl
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos_profile = qos_profile
        self.raw = raw
        self._on_destroy = on_destroy
        self._destroyed = False

    def get_publisher_count(self) -> int:
        """Get the number of publishers that this subscription has."""
        with self.handle:
            return self.__subscription.get_publisher_count()

    @property
    def handle(self) -> '_rclpy.Subscription[MsgT]':
        return self.__subscription

    def destroy(self: Self) -> None:
        """Destroy the subscription, notifying the owning node and releasing the handle."""
        if self._destroyed:
            return
        self._destroyed = True
        if self._on_destroy is not None:
            self._on_destroy(self)
            self._on_destroy = None
        self._destroy()

    def _destroy(self) -> None:
        self.handle.destroy_when_not_in_use()

    @property
    def topic_name(self) -> str:
        with self.handle:
            return self.__subscription.get_topic_name()

    @property
    def callback(self) -> SubscriptionCallbackUnion[MsgT]:
        return self._callback

    @callback.setter
    def callback(self, value: SubscriptionCallbackUnion[MsgT]) -> None:
        self._set_callback_type(value)
        self._callback = value

    @staticmethod
    def _detect_wants_info(callback: SubscriptionCallbackUnion[MsgT]
                           ) -> TypeIs[SubscriptionCallbackWithMessageInfo[MsgT]]:
        try:
            inspect.signature(callback).bind(object())
            return False
        except TypeError:
            pass
        try:
            inspect.signature(callback).bind(object(), object())
            return True
        except TypeError:
            pass
        raise RuntimeError(
            'Subscription callback should be either be callable with one argument'
            '(to get only the message) or two (to get message and message info)')

    def _set_callback_type(self, callback: SubscriptionCallbackUnion[MsgT]) -> None:
        if BaseSubscription._detect_wants_info(callback):
            self._callback_type = BaseSubscription.CallbackType.WithMessageInfo
        else:
            self._callback_type = BaseSubscription.CallbackType.MessageOnly

    @property
    def logger_name(self) -> str:
        """Get the name of the logger associated with the node of the subscription."""
        with self.handle:
            return self.__subscription.get_logger_name()

    @property
    def is_cft_supported(self) -> bool:
        """Check if content filtering is supported for this subscription."""
        with self.handle:
            return self.__subscription.is_cft_supported()

    @property
    def is_cft_enabled(self) -> bool:
        """Check if content filtering is enabled for the subscription."""
        with self.handle:
            return self.__subscription.is_cft_enabled()

    def set_content_filter(self, filter_expression: str, expression_parameters: list[str]) -> None:
        """
        Set the filter expression and expression parameters for the subscription.

        :param filter_expression: The filter expression to set.
        :param expression_parameters: The expression parameters to set.
        :raises: RCLError if internal error occurred when calling the rcl function.
        """
        with self.handle:
            self.__subscription.set_content_filter(filter_expression, expression_parameters)

    def get_content_filter(self) -> ContentFilterOptions:
        """
        Get the filter expression and expression parameters for the subscription.

        :return: ContentFilterOptions object containing the filter expression and expression
            parameters.
        :raises: RCLError if internal error occurred when calling the rcl function.
        """
        with self.handle:
            return self.__subscription.get_content_filter()

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> None:
        self.destroy()


class Subscription(BaseSubscription[MsgT], Generic[MsgT]):

    @overload
    def __init__(
         self,
         subscription_impl: '_rclpy.Subscription[MsgT]',
         msg_type: Type[MsgT],
         topic: str,
         callback: GenericSubscriptionCallbackUnion[bytes],
         qos_profile: QoSProfile,
         raw: Literal[True],
         *,
         on_destroy: Optional[Callable[['Subscription[MsgT]'], None]] = None,
         callback_group: CallbackGroup,
         event_callbacks: SubscriptionEventCallbacks,
    ) -> None: ...

    @overload
    def __init__(
         self,
         subscription_impl: '_rclpy.Subscription[MsgT]',
         msg_type: Type[MsgT],
         topic: str,
         callback: GenericSubscriptionCallbackUnion[MsgT],
         qos_profile: QoSProfile,
         raw: Literal[False],
         *,
         on_destroy: Optional[Callable[['Subscription[MsgT]'], None]] = None,
         callback_group: CallbackGroup,
         event_callbacks: SubscriptionEventCallbacks,
    ) -> None: ...

    @overload
    def __init__(
         self,
         subscription_impl: '_rclpy.Subscription[MsgT]',
         msg_type: Type[MsgT],
         topic: str,
         callback: SubscriptionCallbackUnion[MsgT],
         qos_profile: QoSProfile,
         raw: bool,
         *,
         on_destroy: Optional[Callable[['Subscription[MsgT]'], None]] = None,
         callback_group: CallbackGroup,
         event_callbacks: SubscriptionEventCallbacks,
    ) -> None: ...

    def __init__(
         self,
         subscription_impl: '_rclpy.Subscription[MsgT]',
         msg_type: Type[MsgT],
         topic: str,
         callback: SubscriptionCallbackUnion[MsgT],
         qos_profile: QoSProfile,
         raw: bool,
         *,
         on_destroy: Optional[Callable[['Subscription[MsgT]'], None]] = None,
         callback_group: CallbackGroup,
         event_callbacks: SubscriptionEventCallbacks,
    ) -> None:
        """
        Create a container for a ROS subscription.

        .. warning:: Users should not create a subscription with this constructor, instead they
           should call :meth:`.Node.create_subscription`.

        :param subscription_impl: :class:`Subscription` wrapping the underlying
            ``rcl_subscription_t`` object.
        :param msg_type: The type of ROS messages the subscription will subscribe to.
        :param topic: The name of the topic the subscription will subscribe to.
        :param callback: A user-defined callback function that is called when a message is
            received by the subscription.
        :param callback_group: The callback group for the subscription. If ``None``, then the
            nodes default callback group is used.
        :param qos_profile: The quality of service profile to apply to the subscription.
        :param raw: If ``True``, then received messages will be stored in raw binary
            representation.
        """
        super().__init__(
            subscription_impl=subscription_impl,
            msg_type=msg_type,
            topic=topic,
            callback=callback,
            qos_profile=qos_profile,
            raw=raw
        )
        self._on_destroy = on_destroy
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False

        self.event_handlers = event_callbacks.create_event_handlers(
            callback_group, subscription_impl, topic)

    def _destroy(self) -> None:
        for handler in self.event_handlers:
            handler.destroy()
        super()._destroy()
