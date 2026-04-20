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
async def test_subscription_receives_message():
    """Subscription callback fires when a message is published (sync callback)."""
    received = asyncio.Event()
    received_data = []

    def callback(msg):
        received_data.append(msg.string_value)
        received.set()

    async with AsyncNode('test_sub_node') as node:
        pub = node.create_publisher(Strings, '/test_sub_topic', TEST_QOS)
        node.create_subscription(Strings, '/test_sub_topic', callback, TEST_QOS)

        pub.publish(Strings(string_value='hello'))

        async with asyncio.timeout(5):
            await received.wait()

        assert received_data == ['hello']


@pytest.mark.asyncio
async def test_subscription_callback_exception_sequential():
    """Exception in sequential callback propagates as ExceptionGroup."""
    async def bad_callback(msg):
        raise ValueError('boom')

    node = AsyncNode('test_seq_exc_node')
    pub = node.create_publisher(Strings, '/test_seq_exc_topic', TEST_QOS)
    node.create_subscription(
        Strings, '/test_seq_exc_topic', bad_callback, TEST_QOS)
    pub.publish(Strings(string_value='trigger'))

    with pytest.raises(ExceptionGroup) as exc_info:
        async with asyncio.timeout(5):
            await node.run()

    assert exc_info.value.subgroup(ValueError)


@pytest.mark.asyncio
async def test_subscription_callback_exception_concurrent():
    """Exception in concurrent callback propagates as ExceptionGroup."""
    async def bad_callback(msg):
        raise ValueError('concurrent boom')

    node = AsyncNode('test_conc_exc_node')
    pub = node.create_publisher(
        Strings, '/test_conc_exc_topic', TEST_QOS)
    node.create_subscription(
        Strings, '/test_conc_exc_topic', bad_callback, TEST_QOS,
        concurrent=True)
    pub.publish(Strings(string_value='trigger'))

    with pytest.raises(ExceptionGroup) as exc_info:
        async with asyncio.timeout(5):
            await node.run()

    assert exc_info.value.subgroup(ValueError)


@pytest.mark.asyncio
async def test_subscription_concurrent_dispatch():
    """Concurrent dispatch runs callbacks in parallel, not sequentially."""
    NUM_MESSAGES = 3
    barrier = asyncio.Barrier(NUM_MESSAGES)
    done = asyncio.Event()

    async def callback(msg):
        await barrier.wait()
        done.set()

    async with AsyncNode('test_conc_dispatch_node') as node:
        pub = node.create_publisher(
            Strings, '/test_conc_dispatch_topic', TEST_QOS)
        node.create_subscription(
            Strings, '/test_conc_dispatch_topic', callback, TEST_QOS,
            concurrent=True)

        for i in range(NUM_MESSAGES):
            pub.publish(Strings(string_value=f'msg_{i}'))

        async with asyncio.timeout(5):
            await done.wait()


@pytest.mark.asyncio
async def test_subscription_callback_with_message_info():
    """Subscription callback receives MessageInfo when it accepts two params."""
    received = asyncio.Event()
    received_info = []

    async def callback(msg, info):
        received_info.append(info)
        received.set()

    async with AsyncNode('test_sub_info_node') as node:
        pub = node.create_publisher(Strings, '/test_sub_info_topic', TEST_QOS)
        node.create_subscription(
            Strings, '/test_sub_info_topic', callback, TEST_QOS)

        pub.publish(Strings(string_value='hello'))

        async with asyncio.timeout(5):
            await received.wait()

        assert len(received_info) == 1
        assert 'source_timestamp' in received_info[0]
        assert 'received_timestamp' in received_info[0]
