# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from typing import Generic, Tuple, Type, TYPE_CHECKING, TypedDict, Union

from rclpy.callback_groups import CallbackGroup
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.type_support import MsgT

from .managed_entity import SimpleManagedEntity

if TYPE_CHECKING:
    from typing import TypeAlias, Unpack
    LifecyclePublisherArgs: TypeAlias = Tuple[_rclpy.Publisher[MsgT], Type[MsgT], str, QoSProfile,
                                              PublisherEventCallbacks, CallbackGroup]

    class LifecyclePublisherKWArgs(TypedDict, Generic[MsgT]):
        publisher_impl: _rclpy.Publisher[MsgT]
        msg_type: Type[MsgT]
        topic: str
        qos_profile: QoSProfile
        event_callbacks: PublisherEventCallbacks
        callback_group: CallbackGroup


class LifecyclePublisher(SimpleManagedEntity, Publisher[MsgT]):
    """Managed publisher entity."""

    def __init__(
        self,
        *args: 'Unpack[LifecyclePublisherArgs]',
        **kwargs: 'Unpack[LifecyclePublisherKWArgs[MsgT]]'
    ) -> None:
        SimpleManagedEntity.__init__(self)
        Publisher.__init__(self, *args, **kwargs)

    @SimpleManagedEntity.when_enabled
    def publish(self, msg: Union[MsgT, bytes]) -> None:
        """
        Publish a message if the lifecycle publisher is enabled.

        See rclpy.publisher.Publisher.publish() for more details.
        """
        Publisher.publish(self, msg)
