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

from typing import Type, TYPE_CHECKING, Union

from rclpy.publisher import Publisher
from rclpy.type_support import MsgT

from .managed_entity import SimpleManagedEntity


if TYPE_CHECKING:
    from rclpy.qos import QoSProfile
    from rclpy.callback_groups import CallbackGroup
    from rclpy.event_handler import PublisherEventCallbacks
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class LifecyclePublisher(SimpleManagedEntity, Publisher[MsgT]):
    """Managed publisher entity."""

    def __init__(
        self,
        publisher_impl: '_rclpy.Publisher[MsgT]',
        msg_type: Type[MsgT],
        topic: str,
        qos_profile: 'QoSProfile',
        event_callbacks: 'PublisherEventCallbacks',
        callback_group: 'CallbackGroup'
    ) -> None:
        SimpleManagedEntity.__init__(self)
        Publisher.__init__(self, publisher_impl, msg_type, topic, qos_profile, event_callbacks,
                           callback_group)

    @SimpleManagedEntity.when_enabled
    def publish(self, msg: Union[MsgT, bytes]) -> None:
        """
        Publish a message if the lifecycle publisher is enabled.

        See rclpy.publisher.Publisher.publish() for more details.
        """
        Publisher.publish(self, msg)
