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
from rclpy.qos import QoSProfile

# For documentation only
SUB_MSG_TYPE = TypeVar('Msg')


class Subscription:

    def __init__(
         self,
         subscription_handle,
         subscription_pointer: int,
         msg_type: SUB_MSG_TYPE,
         topic: str,
         callback: Callable,
         callback_group: CallbackGroup,
         qos_profile: QoSProfile,
         node_handle,
         raw: bool
    ) -> None:
        """
        Create a container for a ROS subscription.

        .. warning:: Users should not create a subscription with this constuctor, instead they should
           call :meth:`.Node.create_subscription`.

        :param subscription_handle: Capsule pointing to the underlying ``rcl_subscription_t``
            object.
        :param subscription_pointer: Memory address of the ``rcl_subscription_t`` implementation.
        :param msg_type: The type of ROS messages the subscription will subscribe to.
        :param topic: The name of the topic the subscription will subscribe to.
        :param callback: A user-defined callback function that is called when a message is
            received by the subscription.
        :param callback_group: The callback group for the subscription. If ``None``, then the
            nodes default callback group is used.
        :param qos_profile: The quality of service profile to apply to the subscription.
        :node_handle: Capsule pointing to the underlying ``rcl_node_t`` object for the node the
            subscription is associated with.
        :param raw: If ``True``, then received messages will be stored in raw binary
            representation.
        """
        self.node_handle = node_handle
        self.subscription_handle = subscription_handle
        self.subscription_pointer = subscription_pointer
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False
        self.qos_profile = qos_profile
        self.raw = raw
