# Copyright 2020 Open Source Robotics Foundation, Inc.
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
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSPolicyKind, QoSProfile
from rclpy.qos_overriding_options import _declare_qos_parameteres, QoSOverridingOptions


def test_declare_qos_parameters():
    rclpy.init()
    node = Node("my_node")
    _declare_qos_parameteres(
        Publisher, node, '/my_topic', QoSProfile(depth=10),
        QoSOverridingOptions(policy_kinds=(QoSPolicyKind.RELIABILITY, QoSPolicyKind.HISTORY))
    )
    rclpy.shutdown()
