# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import pytest
import rclpy
from rclpy.handle import InvalidHandle


def test_handle_destroyed_immediately():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    try:
        node = rclpy.create_node('test_handle_destroyed_immediately', context=context)
        node.handle.destroy()
        with pytest.raises(InvalidHandle):
            with node.handle:
                pass
    finally:
        rclpy.shutdown(context=context)


def test_handle_destroyed_when_not_used():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    try:
        node = rclpy.create_node('test_handle_destroyed_when_not_used', context=context)
        with node.handle:
            node.handle.destroy()
            with node.handle:
                pass

        with pytest.raises(InvalidHandle):
            with node.handle:
                pass
    finally:
        rclpy.shutdown(context=context)


def test_handle_destroys_dependents():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    try:
        node1 = rclpy.create_node('test_handle_destroys_dependents1', context=context)
        node2 = rclpy.create_node('test_handle_destroys_dependents2', context=context)
        node2.handle.requires(node1.handle)

        with node1.handle:
            pass
        with node2.handle:
            pass

        node1.handle.destroy()
        with pytest.raises(InvalidHandle):
            with node1.handle:
                pass
        with pytest.raises(InvalidHandle):
            with node2.handle:
                pass
    finally:
        rclpy.shutdown(context=context)


def test_handle_does_not_destroy_requirements():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    try:
        node1 = rclpy.create_node('test_handle_does_not_destroy_requirements1', context=context)
        node2 = rclpy.create_node('test_handle_does_not_destroy_requirements2', context=context)
        node2.handle.requires(node1.handle)

        with node1.handle:
            pass
        with node2.handle:
            pass

        node2.handle.destroy()
        with pytest.raises(InvalidHandle):
            with node2.handle:
                pass
        with node1.handle:
            pass
    finally:
        rclpy.shutdown(context=context)
