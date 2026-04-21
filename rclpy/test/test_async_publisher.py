# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import asyncio

import pytest

import rclpy
from rclpy.experimental import AsyncNode
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from test_msgs.msg import Strings

TEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


@pytest.fixture(autouse=True)
def rclpy_context():
    """Initialize and shut down rclpy for each test."""
    with rclpy.init():
        yield


@pytest.mark.asyncio
async def test_publish_on_destroyed_publisher():
    """Publishing on a destroyed publisher raises RuntimeError."""
    async with AsyncNode('test_pub_destroyed_node') as node:
        pub = node.create_publisher(Strings, '/topic', TEST_QOS)
        pub.destroy()
        with pytest.raises(RuntimeError):
            pub.publish(Strings(string_value='nope'))


@pytest.mark.asyncio
async def test_publish_before_aenter():
    """Publisher works on a node that has never been entered (no _run task)."""
    received = asyncio.Event()

    async def callback(msg):
        received.set()

    pub_node = AsyncNode('test_pub_pre_aenter_pub_node')
    pub = pub_node.create_publisher(
        Strings, '/test_pub_pre_aenter_topic', TEST_QOS)

    async with AsyncNode('test_pub_pre_aenter_sub_node') as sub_node:
        sub_node.create_subscription(
            Strings, '/test_pub_pre_aenter_topic', callback, TEST_QOS)

        async with asyncio.timeout(5):
            while pub.get_subscription_count() < 1:
                await asyncio.sleep(0.05)

        pub.publish(Strings(string_value='hello'))

        async with asyncio.timeout(5):
            await received.wait()

    pub_node.destroy_node()
