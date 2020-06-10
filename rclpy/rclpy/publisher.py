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

from typing import TypeVar, Union

from rclpy.callback_groups import CallbackGroup
from rclpy.handle import Handle
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.qos_event import PublisherEventCallbacks

MsgType = TypeVar('MsgType')


class Publisher:

    def __init__(
        self,
        publisher_handle: Handle,
        msg_type: MsgType,
        topic: str,
        qos_profile: QoSProfile,
        event_callbacks: PublisherEventCallbacks,
        callback_group: CallbackGroup,
    ) -> None:
        """
        Create a container for a ROS publisher.

        .. warning:: Users should not create a publisher with this constuctor, instead they should
           call :meth:`.Node.create_publisher`.

        A publisher is used as a primary means of communication in a ROS system by publishing
        messages on a ROS topic.

        :param publisher_handle: Capsule pointing to the underlying ``rcl_publisher_t`` object.
        :param msg_type: The type of ROS messages the publisher will publish.
        :param topic: The name of the topic the publisher will publish to.
        :param qos_profile: The quality of service profile to apply to the publisher.
        """
        self.__handle = publisher_handle
        self.msg_type = msg_type
        self.topic = topic
        self.qos_profile = qos_profile

        self.event_handlers = event_callbacks.create_event_handlers(
            callback_group, publisher_handle)

    def publish(self, msg: Union[MsgType, bytes]) -> None:
        """
        Send a message to the topic for the publisher.

        :param msg: The ROS message to publish.
        :raises: TypeError if the type of the passed message isn't an instance
          of the provided type when the publisher was constructed.
        """
        with self.handle as capsule:
            if isinstance(msg, self.msg_type):
                _rclpy.rclpy_publish(capsule, msg)
            elif isinstance(msg, bytes):
                _rclpy.rclpy_publish_raw(capsule, msg)
            else:
                raise TypeError()

    def get_subscription_count(self) -> int:
        """Get the amount of subscribers that this publisher has."""
        with self.handle as capsule:
            return _rclpy.rclpy_publisher_get_subscription_count(capsule)

    @property
    def topic_name(self) -> str:
        with self.handle as capsule:
            return _rclpy.rclpy_publisher_get_topic_name(capsule)

    @property
    def handle(self):
        return self.__handle

    def destroy(self):
        self.handle.destroy()

    def assert_liveliness(self) -> None:
        """
        Manually assert that this Publisher is alive.

        If the QoS Liveliness policy is set to RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC, the
        application must call this at least as often as ``QoSProfile.liveliness_lease_duration``.
        """
        with self.handle as capsule:
            _rclpy.rclpy_assert_liveliness(capsule)
