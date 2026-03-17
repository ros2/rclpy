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

from collections.abc import Generator
import time
from typing import List
from typing import Optional
from unittest.mock import Mock

import pytest

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.subscription_content_filter_options import ContentFilterOptions

from test_msgs.msg import BasicTypes
from test_msgs.msg import Empty


NODE_NAME = 'test_node'


@pytest.fixture(scope='session', autouse=True)
def setup_ros() -> None:
    rclpy.init()


@pytest.fixture
def test_node() -> Generator[Node, None, None]:
    node = Node(NODE_NAME)
    yield node
    node.destroy_node()


@pytest.mark.parametrize('topic_name, namespace, expected', [
    # No namespaces
    ('topic', None, '/topic'),
    ('example/topic', None, '/example/topic'),
    # Using topics with namespaces
    ('topic', 'ns', '/ns/topic'),
    ('example/topic', 'ns', '/ns/example/topic'),
    ('example/topic', 'my/ns', '/my/ns/example/topic'),
    ('example/topic', '/my/ns', '/my/ns/example/topic'),
    # Global topics
    ('/topic', 'ns', '/topic'),
    ('/example/topic', 'ns', '/example/topic'),
])
def test_get_subscription_topic_name(topic_name: str, namespace: Optional[str],
                                     expected: str) -> None:
    node = Node('node_name', namespace=namespace, cli_args=None)
    sub = node.create_subscription(
        msg_type=Empty,
        topic=topic_name,
        callback=lambda _: None,
        qos_profile=10,
    )
    assert sub.topic_name == expected

    sub.destroy()

    node.destroy_node()


def test_logger_name_is_equal_to_node_name(test_node: Node) -> None:
    sub = test_node.create_subscription(
        msg_type=Empty,
        topic='topic',
        callback=lambda _: None,
        qos_profile=10,
    )

    assert sub.logger_name == NODE_NAME


@pytest.mark.parametrize('topic_name, namespace, cli_args, expected', [
    ('topic', None, ['--ros-args', '--remap', 'topic:=new_topic'], '/new_topic'),
    ('topic', 'ns', ['--ros-args', '--remap', 'topic:=new_topic'], '/ns/new_topic'),
    ('topic', 'ns', ['--ros-args', '--remap', 'topic:=example/new_topic'],
     '/ns/example/new_topic'),
    ('example/topic', 'ns', ['--ros-args', '--remap', 'example/topic:=new_topic'],
     '/ns/new_topic'),
])
def test_get_subscription_topic_name_after_remapping(topic_name: str, namespace: Optional[str],
                                                     cli_args: List[str], expected: str) -> None:
    node = Node('node_name', namespace=namespace, cli_args=cli_args)
    sub = node.create_subscription(
        msg_type=Empty,
        topic=topic_name,
        callback=lambda _: None,
        qos_profile=10,
    )
    assert sub.topic_name == expected

    sub.destroy()

    node.destroy_node()


def test_subscription_callback_type() -> None:
    node = Node('test_node', namespace='test_subscription/test_subscription_callback_type')
    sub = node.create_subscription(
        msg_type=Empty,
        topic='test_subscription/test_subscription_callback_type/topic',
        qos_profile=10,
        callback=lambda _: None)
    assert sub._callback_type == Subscription.CallbackType.MessageOnly
    sub.destroy()

    sub = node.create_subscription(
        msg_type=Empty,
        topic='test_subscription/test_subscription_callback_type/topic',
        qos_profile=10,
        callback=lambda _, _2: None)
    assert sub._callback_type == Subscription.CallbackType.WithMessageInfo
    sub.destroy()

    with pytest.raises(RuntimeError):
        node.create_subscription(
            msg_type=Empty,
            topic='test_subscription/test_subscription_callback_type/topic',
            qos_profile=10,
            callback=lambda _, _2, _3: None)  # type: ignore[arg-type]

    node.destroy_node()


def test_subscription_callback_setter() -> None:
    node = Node('test_node', namespace='test_subscription/test_subscription_callback_setter')
    sub = node.create_subscription(
        msg_type=Empty,
        topic='test_subscription/test_subscription_callback_/topic',
        qos_profile=10,
        callback=lambda _, _2: None)

    sub.callback = lambda _: None
    assert sub._callback_type == Subscription.CallbackType.MessageOnly

    sub.callback = callback = lambda _, _2: None
    assert sub._callback_type == Subscription.CallbackType.WithMessageInfo

    with pytest.raises(RuntimeError):
        sub.callback = lambda: None

    assert sub.callback is callback

    sub.destroy()
    node.destroy_node()


def test_subscription_context_manager() -> None:
    node = Node('test_node', namespace='test_subscription/test_subscription_callback_type')
    with node.create_subscription(
            msg_type=Empty,
            topic='test_subscription/test_subscription_callback_type/topic',
            qos_profile=10,
            callback=lambda _: None) as sub:
        assert sub._callback_type == Subscription.CallbackType.MessageOnly

    with node.create_subscription(
            msg_type=Empty,
            topic='test_subscription/test_subscription_callback_type/topic',
            qos_profile=10,
            callback=lambda _, _2: None) as sub:
        assert sub._callback_type == Subscription.CallbackType.WithMessageInfo

    node.destroy_node()


def test_subscription_publisher_count() -> None:
    topic_name = 'test_subscription/test_subscription_publisher_count/topic'
    node = Node('test_node', namespace='test_subscription/test_subscription_publisher_count')
    sub = node.create_subscription(
        msg_type=Empty,
        topic=topic_name,
        qos_profile=10,
        callback=lambda _: None)

    assert sub.get_publisher_count() == 0

    pub = node.create_publisher(Empty, topic_name, 10)

    max_seconds_to_wait = 5
    end_time = time.time() + max_seconds_to_wait
    while sub.get_publisher_count() != 1:
        time.sleep(0.05)
        assert time.time() <= end_time  # timeout waiting for pub/sub to discover each other

    assert sub.get_publisher_count() == 1

    pub.destroy()
    sub.destroy()

    node.destroy_node()


def test_on_new_message_callback(test_node: Node) -> None:
    topic_name = '/topic'
    cb = Mock()
    sub = test_node.create_subscription(
        msg_type=Empty,
        topic=topic_name,
        qos_profile=10,
        callback=cb)
    pub = test_node.create_publisher(Empty, topic_name, 10)

    # Wait for publisher and subscriber to discover each other
    max_seconds_to_wait = 5
    end_time = time.time() + max_seconds_to_wait
    while sub.get_publisher_count() != 1:
        time.sleep(0.1)
        assert time.time() <= end_time  # timeout waiting for pub/sub to discover each other

    sub.handle.set_on_new_message_callback(cb)
    cb.assert_not_called()
    pub.publish(Empty())

    # Wait for the callback to be invoked
    end_time = time.time() + max_seconds_to_wait
    while cb.call_count == 0 and time.time() <= end_time:
        time.sleep(0.1)

    cb.assert_called_once_with(1)
    sub.handle.clear_on_new_message_callback()
    cb.reset_mock()
    pub.publish(Empty())

    # Wait a bit to ensure the message would have been received if callback was still set
    time.sleep(1.0)
    cb.assert_not_called()


def test_subscription_set_content_filter(test_node: Node) -> None:

    if rclpy.get_rmw_implementation_identifier() != 'rmw_fastrtps_cpp' and \
       rclpy.get_rmw_implementation_identifier() != 'rmw_connextdds':
        pytest.skip('Content filter is now only supported in FastDDS and ConnextDDS.')

    topic_name = '/topic'
    with test_node.create_subscription(
        msg_type=BasicTypes,
        topic=topic_name,
        qos_profile=10,
            callback=lambda _: None) as sub:

        filter_expression = 'int32_value > %0'
        expression_parameters: List[str] = ['10']

        # There should not be any exceptions.
        try:
            sub.set_content_filter(
                filter_expression,
                expression_parameters
            )
        except Exception as e:
            pytest.fail(f'Unexpected exception raised: {e}')


def test_subscription_is_cft_supported(test_node: Node) -> None:

    # The current rmw implementations for content filter support
    # rmw_fastrtps_cpp: Yes
    # rmw_connextdds: Yes
    # rmw_cyclonedds: No
    # rmw_zenoh: No
    rmw_implementation = rclpy.get_rmw_implementation_identifier()
    rmw_cft_supported = rmw_implementation in ('rmw_fastrtps_cpp', 'rmw_connextdds')

    topic_name = '/topic'
    with test_node.create_subscription(
        msg_type=BasicTypes,
        topic=topic_name,
        qos_profile=10,
            callback=lambda _: None) as sub:

        # There should not be any exceptions.
        try:
            is_cft_supported = sub.is_cft_supported
        except Exception as e:
            pytest.fail(f'Unexpected exception raised: {e}')

        assert is_cft_supported == rmw_cft_supported


def test_subscription_is_cft_enabled(test_node: Node) -> None:

    topic_name = '/topic'
    with test_node.create_subscription(
        msg_type=BasicTypes,
        topic=topic_name,
        qos_profile=10,
            callback=lambda _: None) as sub:

        if not sub.is_cft_supported:
            pytest.skip(
                f'{rclpy.get_rmw_implementation_identifier()} does not support content filter.')

        sub.set_content_filter(
            filter_expression='bool_value = %0',
            expression_parameters=['TRUE']
        )

        # There should not be any exceptions.
        try:
            _ = sub.is_cft_enabled
        except Exception as e:
            pytest.fail(f'Unexpected exception raised: {e}')


def test_subscription_get_content_filter(test_node: Node) -> None:

    topic_name = '/topic'
    with test_node.create_subscription(
        msg_type=BasicTypes,
        topic=topic_name,
        qos_profile=10,
            callback=lambda _: None) as sub:

        if not sub.is_cft_supported:
            pytest.skip(
                f'{rclpy.get_rmw_implementation_identifier()} does not support content filter.')

        assert sub.is_cft_enabled is False

        filter_expression = 'int32_value > %0'
        expression_parameters: List[str] = ['60']

        sub.set_content_filter(
            filter_expression,
            expression_parameters
        )

        assert sub.is_cft_enabled is True

        cf_option = sub.get_content_filter()
        assert cf_option.filter_expression == filter_expression
        assert len(cf_option.expression_parameters) == len(expression_parameters)
        assert cf_option.expression_parameters[0] == expression_parameters[0]


def test_subscription_content_filter_effect(test_node: Node) -> None:

    topic_name = '/topic'

    received_msgs = []

    def sub_callback(msg: BasicTypes) -> None:
        received_msgs.append(msg.int32_value)

    sub = test_node.create_subscription(
        msg_type=BasicTypes,
        topic=topic_name,
        qos_profile=10,
        callback=sub_callback)

    if not sub.is_cft_supported:
        sub.destroy()
        pytest.skip(
            f'{rclpy.get_rmw_implementation_identifier()} does not support content filter.')

    pub = test_node.create_publisher(BasicTypes, topic_name, 10)

    assert sub.is_cft_enabled is False

    def wait_msgs(timeout: float, expected_msg_count: int) -> None:
        end_time = time.time() + timeout
        while rclpy.ok() and time.time() < end_time and len(received_msgs) < expected_msg_count:
            rclpy.spin_once(test_node, timeout_sec=0.2)

    # Publish 3 messages
    def publish_messages(pub: Publisher[BasicTypes]) -> None:
        msg = BasicTypes()
        msg.int32_value = 10
        pub.publish(msg)
        msg.int32_value = 20
        pub.publish(msg)
        msg.int32_value = 30
        pub.publish(msg)

    # Publish messages and them should be all received.
    publish_messages(pub)

    # Check within 2 seconds whether the desired number of messages has been received.
    expected_msg_count = 3
    wait_msgs(timeout=2.0, expected_msg_count=expected_msg_count)
    assert len(received_msgs) == expected_msg_count

    # Set content filter to filter out messages with int32_value <= 15
    sub.set_content_filter(
        filter_expression='int32_value > %0',
        expression_parameters=['15']
    )
    assert sub.is_cft_enabled is True

    received_msgs = []
    # Publish messages again and part of messages should be received.
    publish_messages(pub)

    # Check within 2 seconds whether the desired number of messages has been received.
    expected_msg_count = 2
    wait_msgs(timeout=2.0, expected_msg_count=expected_msg_count)
    assert len(received_msgs) == expected_msg_count
    assert received_msgs[0] == 20
    assert received_msgs[1] == 30

    pub.destroy()
    sub.destroy()


def test_subscription_content_filter_reset(test_node: Node) -> None:

    topic_name = '/topic'

    received_msgs = []

    def sub_callback(msg: BasicTypes) -> None:
        received_msgs.append(msg.int32_value)

    sub = test_node.create_subscription(
        msg_type=BasicTypes,
        topic=topic_name,
        qos_profile=10,
        callback=sub_callback)

    if not sub.is_cft_supported:
        sub.destroy()
        pytest.skip(
            f'{rclpy.get_rmw_implementation_identifier()} does not support content filter.')

    pub = test_node.create_publisher(BasicTypes, topic_name, 10)

    # Set content filter to filter out messages with int32_value <= 15
    sub.set_content_filter(
        filter_expression='int32_value > %0',
        expression_parameters=['15']
    )
    assert sub.is_cft_enabled is True

    def wait_msgs(timeout: float, expected_msg_count: int) -> None:
        end_time = time.time() + timeout
        while rclpy.ok() and time.time() < end_time and len(received_msgs) < expected_msg_count:
            rclpy.spin_once(test_node, timeout_sec=0.2)

    # Publish 3 messages
    def publish_messages(pub: Publisher[BasicTypes]) -> None:
        msg = BasicTypes()
        msg.int32_value = 10
        pub.publish(msg)
        msg.int32_value = 20
        pub.publish(msg)
        msg.int32_value = 30
        pub.publish(msg)

    publish_messages(pub)

    expected_msg_count = 2
    wait_msgs(timeout=2.0, expected_msg_count=expected_msg_count)
    assert len(received_msgs) == expected_msg_count

    received_msgs = []

    # Reset content filter
    sub.set_content_filter(
        filter_expression='',
        expression_parameters=[]
    )
    assert sub.is_cft_enabled is False

    publish_messages(pub)

    expected_msg_count = 3
    wait_msgs(timeout=2.0, expected_msg_count=expected_msg_count)
    assert len(received_msgs) == expected_msg_count


def test_subscription_content_filter_at_create_subscription(test_node: Node) -> None:

    topic_name = '/topic'

    received_msgs = []

    content_filter_options = ContentFilterOptions(
        filter_expression='int32_value > %0',
        expression_parameters=['15'])

    def sub_callback(msg: BasicTypes) -> None:
        received_msgs.append(msg.int32_value)

    sub = test_node.create_subscription(
        msg_type=BasicTypes,
        topic=topic_name,
        qos_profile=10,
        callback=sub_callback,
        content_filter_options=content_filter_options)

    if not sub.is_cft_supported:
        sub.destroy()
        pytest.skip(
            f'{rclpy.get_rmw_implementation_identifier()} does not support content filter.')

    pub = test_node.create_publisher(BasicTypes, topic_name, 10)

    assert sub.is_cft_enabled is True

    def wait_msgs(timeout: float, expected_msg_count: int) -> None:
        end_time = time.time() + timeout
        while rclpy.ok() and time.time() < end_time and len(received_msgs) < expected_msg_count:
            rclpy.spin_once(test_node, timeout_sec=0.2)

    # Publish 3 messages
    def publish_messages(pub: Publisher[BasicTypes]) -> None:
        msg = BasicTypes()
        msg.int32_value = 10
        pub.publish(msg)
        msg.int32_value = 20
        pub.publish(msg)
        msg.int32_value = 30
        pub.publish(msg)

    # Publish messages and part of messages should be received.
    publish_messages(pub)

    # Check within 2 seconds whether the desired number of messages has been received.
    expected_msg_count = 2
    wait_msgs(timeout=2.0, expected_msg_count=expected_msg_count)
    assert len(received_msgs) == expected_msg_count
    assert received_msgs[0] == 20
    assert received_msgs[1] == 30

    pub.destroy()
    sub.destroy()
