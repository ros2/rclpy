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

import time

import pytest

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription

from test_msgs.msg import Empty


@pytest.fixture(scope='session', autouse=True)
def setup_ros():
    rclpy.init()


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
def test_get_subscription_topic_name(topic_name, namespace, expected):
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


@pytest.mark.parametrize('topic_name, namespace, cli_args, expected', [
    ('topic', None, ['--ros-args', '--remap', 'topic:=new_topic'], '/new_topic'),
    ('topic', 'ns', ['--ros-args', '--remap', 'topic:=new_topic'], '/ns/new_topic'),
    ('topic', 'ns', ['--ros-args', '--remap', 'topic:=example/new_topic'],
     '/ns/example/new_topic'),
    ('example/topic', 'ns', ['--ros-args', '--remap', 'example/topic:=new_topic'],
     '/ns/new_topic'),
])
def test_get_subscription_topic_name_after_remapping(topic_name, namespace, cli_args, expected):
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


def test_subscription_callback_type():
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
            callback=lambda _, _2, _3: None)

    node.destroy_node()


def test_subscription_publisher_count():
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
