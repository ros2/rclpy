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
from unittest.mock import Mock

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.clock import ClockType
from rclpy.exceptions import InvalidServiceNameException
from rclpy.exceptions import InvalidTopicNameException
from rclpy.parameter import Parameter
from test_msgs.msg import Primitives

TEST_NODE = 'my_node'
TEST_NAMESPACE = '/my_ns'


class TestNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node(TEST_NODE, namespace=TEST_NAMESPACE, context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_accessors(self):
        self.assertIsNotNone(self.node.handle)
        with self.assertRaises(AttributeError):
            self.node.handle = 'garbage'
        self.assertEqual(self.node.get_name(), TEST_NODE)
        self.assertEqual(self.node.get_namespace(), TEST_NAMESPACE)
        self.assertEqual(self.node.get_clock().clock_type, ClockType.ROS_TIME)

    def test_create_publisher(self):
        self.node.create_publisher(Primitives, 'chatter')
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not contain characters'):
            self.node.create_publisher(Primitives, 'chatter?')
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not start with a number'):
            self.node.create_publisher(Primitives, '/chatter/42_is_the_answer')
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_publisher(Primitives, 'chatter/{bad_sub}')

    def test_create_subscription(self):
        self.node.create_subscription(Primitives, 'chatter', lambda msg: print(msg))
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not contain characters'):
            self.node.create_subscription(Primitives, 'chatter?', lambda msg: print(msg))
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not start with a number'):
            self.node.create_subscription(Primitives, '/chatter/42ish', lambda msg: print(msg))
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_subscription(Primitives, 'foo/{bad_sub}', lambda msg: print(msg))

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

    def test_node_names_and_namespaces(self):
        # test that it doesn't raise
        self.node.get_node_names_and_namespaces()

    def test_count_publishers_subscribers(self):
        short_topic_name = 'chatter'
        fq_topic_name = '%s/%s' % (TEST_NAMESPACE, short_topic_name)

        self.assertEqual(0, self.node.count_publishers(fq_topic_name))
        self.assertEqual(0, self.node.count_subscribers(fq_topic_name))

        self.node.create_publisher(Primitives, short_topic_name)
        self.assertEqual(1, self.node.count_publishers(short_topic_name))
        self.assertEqual(1, self.node.count_publishers(fq_topic_name))

        self.node.create_subscription(Primitives, short_topic_name, lambda msg: print(msg))
        self.assertEqual(1, self.node.count_subscribers(short_topic_name))
        self.assertEqual(1, self.node.count_subscribers(fq_topic_name))

        self.node.create_subscription(Primitives, short_topic_name, lambda msg: print(msg))
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

    def test_initially_no_executor(self):
        node = rclpy.create_node('my_node', context=self.context)
        try:
            assert node.executor is None
        finally:
            node.destroy_node()

    def test_set_executor_adds_node_to_it(self):
        node = rclpy.create_node('my_node', context=self.context)
        executor = Mock()
        executor.add_node.return_value = True
        try:
            node.executor = executor
            assert id(executor) == id(node.executor)
        finally:
            node.destroy_node()
        executor.add_node.assert_called_once_with(node)

    def test_set_executor_removes_node_from_old_executor(self):
        node = rclpy.create_node('my_node', context=self.context)
        old_executor = Mock()
        old_executor.add_node.return_value = True
        new_executor = Mock()
        new_executor.add_node.return_value = True
        try:
            node.executor = old_executor
            assert id(old_executor) == id(node.executor)
            node.executor = new_executor
            assert id(new_executor) == id(node.executor)
        finally:
            node.destroy_node()
        old_executor.remove_node.assert_called_once_with(node)
        new_executor.remove_node.assert_not_called()

    def test_set_executor_clear_executor(self):
        node = rclpy.create_node('my_node', context=self.context)
        executor = Mock()
        executor.add_node.return_value = True
        try:
            node.executor = executor
            assert id(executor) == id(node.executor)
            node.executor = None
            assert node.executor is None
        finally:
            node.destroy_node()

    def test_node_set_parameters(self):
        results = self.node.set_parameters([
            Parameter('foo', Parameter.Type.INTEGER, 42),
            Parameter('bar', Parameter.Type.STRING, 'hello'),
            Parameter('baz', Parameter.Type.DOUBLE, 2.41)
        ])
        self.assertTrue(all(isinstance(result, SetParametersResult) for result in results))
        self.assertTrue(all(result.successful for result in results))
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)

    def test_node_cannot_set_invalid_parameters(self):
        with self.assertRaises(TypeError):
            self.node.set_parameters([42])

    def test_node_set_parameters_atomically(self):
        result = self.node.set_parameters_atomically([
            Parameter('foo', Parameter.Type.INTEGER, 42),
            Parameter('bar', Parameter.Type.STRING, 'hello'),
            Parameter('baz', Parameter.Type.DOUBLE, 2.41)
        ])
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertIsInstance(result, SetParametersResult)
        self.assertTrue(result.successful)

    def test_node_get_parameter(self):
        self.node.set_parameters([Parameter('foo', Parameter.Type.INTEGER, 42)])
        self.assertIsInstance(self.node.get_parameter('foo'), Parameter)
        self.assertEqual(self.node.get_parameter('foo').value, 42)

    def test_node_get_parameter_returns_parameter_not_set(self):
        self.assertIsInstance(self.node.get_parameter('unset'), Parameter)
        self.assertEqual(self.node.get_parameter('unset').type_, Parameter.Type.NOT_SET)

    def test_node_has_parameter_services(self):
        service_names_and_types = self.node.get_service_names_and_types()
        self.assertIn(
            ('/my_ns/my_node/describe_parameters', ['rcl_interfaces/DescribeParameters']),
            service_names_and_types
        )
        self.assertIn(
            ('/my_ns/my_node/get_parameter_types', ['rcl_interfaces/GetParameterTypes']),
            service_names_and_types
        )
        self.assertIn(
            ('/my_ns/my_node/get_parameters', ['rcl_interfaces/GetParameters']),
            service_names_and_types
        )
        self.assertIn(
            ('/my_ns/my_node/list_parameters', ['rcl_interfaces/ListParameters']),
            service_names_and_types
        )
        self.assertIn(
            ('/my_ns/my_node/set_parameters', ['rcl_interfaces/SetParameters']),
            service_names_and_types
        )
        self.assertIn(
            (
                '/my_ns/my_node/set_parameters_atomically',
                ['rcl_interfaces/SetParametersAtomically']
            ), service_names_and_types
        )


class TestCreateNode(unittest.TestCase):

    def test_use_global_arguments(self):
        context = rclpy.context.Context()
        rclpy.init(args=['process_name', '__node:=global_node_name'], context=context)
        try:
            node1 = rclpy.create_node(
                'my_node', namespace='/my_ns', use_global_arguments=True, context=context)
            node2 = rclpy.create_node(
                'my_node', namespace='/my_ns', use_global_arguments=False, context=context)
            self.assertEqual('global_node_name', node1.get_name())
            self.assertEqual('my_node', node2.get_name())
            node1.destroy_node()
            node2.destroy_node()
        finally:
            rclpy.shutdown(context=context)

    def test_node_arguments(self):
        context = rclpy.context.Context()
        rclpy.init(context=context)
        try:
            node = rclpy.create_node(
                'my_node', namespace='/my_ns', cli_args=['__ns:=/foo/bar'], context=context)
            self.assertEqual('/foo/bar', node.get_namespace())
            node.destroy_node()
        finally:
            rclpy.shutdown(context=context)


if __name__ == '__main__':
    unittest.main()
