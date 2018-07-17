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

import unittest

from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.exceptions import InvalidServiceNameException
from rclpy.exceptions import InvalidTopicNameException
from std_msgs.msg import String

TEST_NODE = 'my_node'
TEST_NAMESPACE = '/my_ns'


class TestNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(TEST_NODE, namespace=TEST_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_accessors(self):
        self.assertIsNotNone(self.node.handle)
        with self.assertRaises(AttributeError):
            self.node.handle = 'garbage'
        self.assertEqual(self.node.get_name(), TEST_NODE)
        self.assertEqual(self.node.get_namespace(), TEST_NAMESPACE)

    def test_create_publisher(self):
        self.node.create_publisher(String, 'chatter')
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not contain characters'):
            self.node.create_publisher(String, 'chatter?')
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not start with a number'):
            self.node.create_publisher(String, '/chatter/42_is_the_answer')
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_publisher(String, 'chatter/{bad_sub}')

    def test_create_subscription(self):
        self.node.create_subscription(String, 'chatter', lambda msg: print(msg))
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not contain characters'):
            self.node.create_subscription(String, 'chatter?', lambda msg: print(msg))
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not start with a number'):
            self.node.create_subscription(String, '/chatter/42ish', lambda msg: print(msg))
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_subscription(String, 'foo/{bad_sub}', lambda msg: print(msg))

    def test_create_client(self):
        self.node.create_client(GetParameters, 'get/parameters')
        with self.assertRaisesRegex(InvalidServiceNameException, 'must not contain characters'):
            self.node.create_client(GetParameters, 'get/parameters?')
        with self.assertRaisesRegex(InvalidServiceNameException, 'must not start with a number'):
            self.node.create_client(GetParameters, '/get/42parameters')
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_client(GetParameters, 'foo/{bad_sub}')

    def test_create_service(self):
        self.node.create_service(GetParameters, 'get/parameters', lambda req: None)
        with self.assertRaisesRegex(InvalidServiceNameException, 'must not contain characters'):
            self.node.create_service(GetParameters, 'get/parameters?', lambda req: None)
        with self.assertRaisesRegex(InvalidServiceNameException, 'must not start with a number'):
            self.node.create_service(GetParameters, '/get/42parameters', lambda req: None)
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_service(GetParameters, 'foo/{bad_sub}', lambda req: None)

    def test_service_names_and_types(self):
        # test that it doesn't raise
        self.node.get_service_names_and_types()

    def test_topic_names_and_types(self):
        # test that it doesn't raise
        self.node.get_topic_names_and_types(no_demangle=True)
        self.node.get_topic_names_and_types(no_demangle=False)

    def test_node_names(self):
        # test that it doesn't raise
        self.node.get_node_names()

    def test_count_publishers_subscribers(self):
        short_topic_name = 'chatter'
        fq_topic_name = '%s/%s' % (TEST_NAMESPACE, short_topic_name)

        self.assertEqual(0, self.node.count_publishers(fq_topic_name))
        self.assertEqual(0, self.node.count_subscribers(fq_topic_name))

        self.node.create_publisher(String, short_topic_name)
        self.assertEqual(1, self.node.count_publishers(short_topic_name))
        self.assertEqual(1, self.node.count_publishers(fq_topic_name))

        self.node.create_subscription(String, short_topic_name, lambda msg: print(msg))
        self.assertEqual(1, self.node.count_subscribers(short_topic_name))
        self.assertEqual(1, self.node.count_subscribers(fq_topic_name))

        self.node.create_subscription(String, short_topic_name, lambda msg: print(msg))
        self.assertEqual(2, self.node.count_subscribers(short_topic_name))
        self.assertEqual(2, self.node.count_subscribers(fq_topic_name))

        # error cases
        with self.assertRaisesRegex(TypeError, 'Argument topic_name is not a'):
            self.node.count_subscribers(1)
        with self.assertRaisesRegex(ValueError, 'is invalid'):
            self.node.count_subscribers('42')
        with self.assertRaisesRegex(ValueError, 'is invalid'):
            self.node.count_publishers('42')

    def test_node_logger(self):
        node_logger = self.node.get_logger()
        expected_name = '%s.%s' % (TEST_NAMESPACE.replace('/', '.')[1:], TEST_NODE)
        self.assertEqual(node_logger.name, expected_name)
        node_logger.set_level(rclpy.logging.LoggingSeverity.INFO)
        node_logger.debug('test')


class TestCreateNode(unittest.TestCase):

    def test_use_global_arguments(self):
        rclpy.init(args=['process_name', '__node:=global_node_name'])
        try:
            node1 = rclpy.create_node('my_node', namespace='/my_ns', use_global_arguments=True)
            node2 = rclpy.create_node('my_node', namespace='/my_ns', use_global_arguments=False)
            self.assertEqual('global_node_name', node1.get_name())
            self.assertEqual('my_node', node2.get_name())
            node1.destroy_node()
            node2.destroy_node()
        finally:
            rclpy.shutdown()

    def test_node_arguments(self):
        rclpy.init()
        try:
            node = rclpy.create_node('my_node', namespace='/my_ns', cli_args=['__ns:=/foo/bar'])
            self.assertEqual('/foo/bar', node.get_namespace())
            node.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
