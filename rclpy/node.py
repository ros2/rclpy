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

import rclpy._rclpy__rmw_opensplice_cpp as _rclpy

from rclpy.publisher import Publisher
from rclpy.subscription import Subscription


class Node:

    def __init__(self, handle):
        self.handle = handle
        self.subscriptions = []

    def create_publisher(self, msg_type, topic, qos_profile):
        publisher_handle = _rclpy.rclpy_create_publisher(self.handle, msg_type, topic)

        return Publisher(publisher_handle, msg_type, topic, qos_profile)

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        subscription_handle = _rclpy.rclpy_create_subscription(self.handle, msg_type, topic)

        subscription = Subscription(subscription_handle, msg_type, topic, callback, qos_profile)
        self.subscriptions.append(subscription)
        return subscription
