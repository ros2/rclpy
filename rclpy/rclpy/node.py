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

import rclpy

from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rosidl_generator_py import import_type_support
from rosidl_cmake import convert_camel_case_to_lower_case_underscore


class Node:

    def __init__(self, handle):
        self.handle = handle
        self.subscriptions = []

    def create_publisher(self, msg_type, topic, qos_profile):
        if msg_type.__class__._TYPE_SUPPORT is None:
            self._import_type_support(msg_type)
        publisher_handle = rclpy._rclpy.rclpy_create_publisher(self.handle, msg_type, topic)

        return Publisher(publisher_handle, msg_type, topic, qos_profile)

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        if msg_type.__class__._TYPE_SUPPORT is None:
            self._import_type_support(msg_type)
        subscription_handle = rclpy._rclpy.rclpy_create_subscription(self.handle, msg_type, topic)

        subscription = Subscription(subscription_handle, msg_type, topic, callback, qos_profile)
        self.subscriptions.append(subscription)
        return subscription

    def _import_type_support(self, msg_type):
        mod = msg_type.__class__.__module__
        pkg_name = mod[:mod.find('.')]
        subfolder = mod[mod.find('.') + 1:mod.rfind('.')]
        module = import_type_support(
            pkg_name, subfolder, msg_type,
            rclpy._rclpy.rclpy_get_rmw_implementation_identifier())
        lowercase_msg_type = convert_camel_case_to_lower_case_underscore(msg_type.__name__)
        msg_type.__class__._CONVERT_FROM_PY = module.__dict__.get(
            'convert_from_py_' + lowercase_msg_type)
        msg_type.__class__._CONVERT_TO_PY = module.__dict__.get(
            'convert_to_py_' + lowercase_msg_type)
        msg_type.__class__._TYPE_SUPPORT = module.__dict__.get(
            'type_support_' + lowercase_msg_type)
        # FIXME(mikaelarguedas) what about nested messages ? Should import all these recursively
