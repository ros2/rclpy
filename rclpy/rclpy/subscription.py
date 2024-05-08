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


from enum import Enum
import inspect
from typing import Callable, Generic, Protocol, Type, TypeVar, TypedDict, Union, Tuple, Optional

from rclpy.destroyable import DestroyableType
from rclpy.callback_groups import CallbackGroup
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.type_support import MsgT


class MessageInfo(TypedDict):
    source_timestamp: Optional[int]
    received_timestamp: Optional[int]
    publication_sequence_number: Optional[int]
    reception_sequence_number: Optional[int]


class SubscriptionHandle(DestroyableType, Protocol[MsgT]):

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""
        ...

    def take_message(self, pymsg_type: Type[MsgT], raw: bool) -> Tuple[MsgT, MessageInfo]:
        """Take a message and its metadata from a subscription."""
        ...

    def get_logger_name(self) -> str:
        """Get the name of the logger associated with the node of the subscription."""
        ...

    def get_topic_name(self) -> str:
        """Return the resolved topic name of a subscription."""
        ...

    def get_publisher_count(self) -> int:
        """Count the publishers from a subscription."""
        ...


# Left to support Legacy TypeVars.
MsgType = TypeVar('MsgType')


class Subscription(Generic[MsgT]):

    class CallbackType(Enum):
        MessageOnly = 0
        WithMessageInfo = 1

    def __init__(
         self,
         subscription_impl: SubscriptionHandle[MsgT],
         msg_type: Type[MsgT],
         topic: str,
         callback: Union[Callable[[MsgT], None], Callable[[MsgT, MessageInfo], None]],
         callback_group: CallbackGroup,
         qos_profile: QoSProfile,
         raw: bool,
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
        self.__subscription = subscription_impl
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False
        self.qos_profile = qos_profile
        self.raw = raw

        self.event_handlers = event_callbacks.create_event_handlers(
            callback_group, subscription_impl, topic)

    def get_publisher_count(self) -> int:
        """Get the number of publishers that this subscription has."""
        with self.handle:
            return self.__subscription.get_publisher_count()

    @property
    def handle(self) -> SubscriptionHandle[MsgT]:
        return self.__subscription

    def destroy(self) -> None:
        """
        Destroy a container for a ROS subscription.

        .. warning:: Users should not destroy a subscription with this method, instead they
           should call :meth:`.Node.destroy_subscription`.
        """
        for handler in self.event_handlers:
            handler.destroy()
        self.handle.destroy_when_not_in_use()

    @property
    def topic_name(self) -> str:
        with self.handle:
            return self.__subscription.get_topic_name()

    @property
    def callback(self) -> Union[Callable[[MsgT], None], Callable[[MsgT, MessageInfo], None]]:
        return self._callback

    @callback.setter
    def callback(self, value: Union[Callable[[MsgT], None],
                                    Callable[[MsgT, MessageInfo], None]]) -> None:
        self._callback = value
        self._callback_type = Subscription.CallbackType.MessageOnly
        try:
            inspect.signature(value).bind(object())
            return
        except TypeError:
            pass
        try:
            inspect.signature(value).bind(object(), object())
            self._callback_type = Subscription.CallbackType.WithMessageInfo
            return
        except TypeError:
            pass
        raise RuntimeError(
            'Subscription.__init__(): callback should be either be callable with one argument'
            '(to get only the message) or two (to get message and message info)')
