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

import pytest

import rclpy
from rclpy.node import Node
from test_msgs.msg import Empty


class TestSubscriber(Node):

    def __init__(self, topic):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Empty,
            topic,
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        pass

    @property
    def topic_name(self):
        return self.subscription.topic_name


@pytest.fixture(scope="session", autouse=True)
def setup_ros():
    rclpy.init()


@pytest.mark.parametrize('topic_name, expected', [
    ('topic', '/topic'),
    ('/topic', '/topic'),
    ('sample/topic', '/sample/topic'),
    ('/sample/topic', '/sample/topic'),
    ('sample/topic/name', '/sample/topic/name'),
    ('/sample/topic/name', '/sample/topic/name'),
])
def test_get_subscription_topic_name(topic_name, expected):
    sub = TestSubscriber(topic_name)
    assert sub.topic_name == expected
    sub.destroy_node()
