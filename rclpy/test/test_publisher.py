# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import pathlib
import unittest

from rclpy.parameter import Parameter
from test_msgs.msg import BasicTypes

import rclpy

TEST_NODE = 'my_publisher_node'
TEST_NAMESPACE = '/my_publisher_ns'

# TODO Add more topic styles
TEST_TOPIC = "my_topic"
TEST_TOPIC_POSITION = "my_topic/position"
TEST_TOPIC_SPEED = "my_topic/speed"


TEST_RESOURCES_DIR = pathlib.Path(__file__).resolve().parent / 'resources' / 'test_node'


def full_topic(topic_name):
    if not topic_name:
        raise(Exception("Invalid topic name, empty!"))
    if topic_name[0] == "/":
        return topic_name

    return TEST_NAMESPACE + "/" + topic_name


class TestPublisher(unittest.TestCase):

    @classmethod
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            TEST_NODE,
            namespace=TEST_NAMESPACE,
            context=self.context,
            parameter_overrides=[
                Parameter('initial_foo', Parameter.Type.INTEGER, 4321),
                Parameter('initial_bar', Parameter.Type.STRING, 'init_param'),
                Parameter('initial_baz', Parameter.Type.DOUBLE, 3.14)
            ],
            cli_args=[
                '--ros-args', '-p', 'initial_fizz:=buzz',
                '--params-file', str(TEST_RESOURCES_DIR / 'test_parameters.yaml'),
                '-p', 'initial_buzz:=1.'
            ],
            automatically_declare_parameters_from_overrides=False
        )
        self.base_publisher = self.node.create_publisher(BasicTypes, TEST_TOPIC, 0)
        self.position_publisher = self.node.create_publisher(BasicTypes, TEST_TOPIC_POSITION, 0)
        self.speed_publisher = self.node.create_publisher(BasicTypes, TEST_TOPIC_SPEED, 0)

    @classmethod
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_resolved_name(self):
        self.assertEqual(self.base_publisher.resolved_name(), full_topic(TEST_TOPIC))
        self.assertEqual(self.position_publisher.resolved_name(), full_topic(TEST_TOPIC_POSITION))
        self.assertEqual(self.speed_publisher.resolved_name(), full_topic(TEST_TOPIC_SPEED))


if __name__ == '__main__':
    unittest.main()
