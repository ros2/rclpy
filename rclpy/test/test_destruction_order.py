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


def test_destruction_order():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node1 = Node('test_destruction_order_node_1', context=context)
    node2 = Node('test_destruction_order_node_2', context=context)
    node3 = Node('test_destruction_order_node_3', context=context)

    node1.cyclic_ref = node1
    node2.cyclic_ref = node2
    node1.node3 = node3
    node2.node3 = node3

    node1.handle.context_handle = context.handle
    node2.handle.context_handle = context.handle
