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
        node.destroy_node()
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
            node.destroy_node()

        with pytest.raises(InvalidHandle):
            with node.handle:
                pass
    finally:
        rclpy.shutdown(context=context)
