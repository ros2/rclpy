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
from rclpy.duration import Duration
from rclpy.event_handler import EventHandler
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile

MsgType = TypeVar('MsgType')


class Publisher:

    def __init__(
        self,
        publisher_impl: _rclpy.Publisher,
        msg_type: MsgType,
        topic: str,
        qos_profile: QoSProfile,
        event_callbacks: PublisherEventCallbacks,
        callback_group: CallbackGroup,
    ) -> None:
        """
        Create a container for a ROS publisher.

        .. warning:: Users should not create a publisher with this constructor, instead they should
           call :meth:`.Node.create_publisher`.

        A publisher is used as a primary means of communication in a ROS system by publishing
        messages on a ROS topic.

        :param publisher_impl: Publisher wrapping the underlying ``rcl_publisher_t`` object.
        :param msg_type: The type of ROS messages the publisher will publish.
        :param topic: The name of the topic the publisher will publish to.
        :param qos_profile: The quality of service profile to apply to the publisher.
        """
        self.__publisher = publisher_impl
        self.msg_type = msg_type
        self.topic = topic
        self.qos_profile = qos_profile

        self.event_handlers: EventHandler = event_callbacks.create_event_handlers(
            callback_group, publisher_impl, topic)

    def publish(self, msg: Union[MsgType, bytes]) -> None:
        """
        Send a message to the topic for the publisher.

        :param msg: The ROS message to publish.
        :raises: TypeError if the type of the passed message isn't an instance
          of the provided type when the publisher was constructed.
        """
        with self.handle:
            if isinstance(msg, self.msg_type):
                self.__publisher.publish(msg)
            elif isinstance(msg, bytes):
                self.__publisher.publish_raw(msg)
            else:
                raise TypeError('Expected {}, got {}'.format(self.msg_type, type(msg)))

    def get_subscription_count(self) -> int:
        """Get the amount of subscribers that this publisher has."""
        with self.handle:
            return self.__publisher.get_subscription_count()

    @property
    def topic_name(self) -> str:
        with self.handle:
            return self.__publisher.get_topic_name()

    @property
    def handle(self):
        return self.__publisher

    def destroy(self):
        """
        Destroy a container for a ROS publisher.

        .. warning:: Users should not destroy a publisher with this method, instead they should
           call :meth:`.Node.destroy_publisher`.
        """
        for handler in self.event_handlers:
            handler.destroy()
        self.__publisher.destroy_when_not_in_use()

    def assert_liveliness(self) -> None:
        """
        Manually assert that this Publisher is alive.

        If the QoS Liveliness policy is set to MANUAL_BY_TOPIC, the
        application must call this at least as often as ``QoSProfile.liveliness_lease_duration``.
        """
        with self.handle:
            _rclpy.rclpy_assert_liveliness(self.handle)

    def wait_for_all_acked(self, timeout: Duration = Duration(seconds=-1)) -> bool:
        """
        Wait until all published message data is acknowledged or until the timeout elapses.

        If the timeout is negative then this function will block indefinitely until all published
        message data is acknowledged.
        If the timeout is 0 then it will check if all published message has been acknowledged
        without waiting.
        If the timeout is greater than 0 then it will return after that period of time has elapsed
        or all published message data is acknowledged.
        This function only waits for acknowledgments if the publisher's QOS profile is RELIABLE.
        Otherwise this function will immediately return true.

        :param timeout: the duration to wait for all published message data to be acknowledged.
        :returns: true if all published message data is acknowledged before the timeout, otherwise
            false.
        """
        with self.handle:
            return self.__publisher.wait_for_all_acked(timeout._duration_handle)
