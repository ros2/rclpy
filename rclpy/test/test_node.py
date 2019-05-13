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
import warnings

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.clock import ClockType
from rclpy.exceptions import InvalidParameterException
from rclpy.exceptions import InvalidParameterValueException
from rclpy.exceptions import InvalidServiceNameException
from rclpy.exceptions import InvalidTopicNameException
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_default
from rclpy.qos import qos_profile_sensor_data
from test_msgs.msg import BasicTypes

TEST_NODE = 'my_node'
TEST_NAMESPACE = '/my_ns'


class TestNodeAllowUndeclaredParameters(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node(
            TEST_NODE, namespace=TEST_NAMESPACE, context=cls.context,
            allow_undeclared_parameters=True)

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
        self.node.create_publisher(BasicTypes, 'chatter')
        self.node.create_publisher(BasicTypes, 'chatter', 0)
        self.node.create_publisher(BasicTypes, 'chatter', 1)
        self.node.create_publisher(BasicTypes, 'chatter', qos_profile_sensor_data)
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not contain characters'):
            self.node.create_publisher(BasicTypes, 'chatter?', 1)
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not start with a number'):
            self.node.create_publisher(BasicTypes, '/chatter/42_is_the_answer', 1)
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_publisher(BasicTypes, 'chatter/{bad_sub}', 1)
        with self.assertRaisesRegex(ValueError, 'must be greater than or equal to zero'):
            self.node.create_publisher(BasicTypes, 'chatter', -1)
        with self.assertRaisesRegex(TypeError, 'Expected QoSProfile or int'):
            self.node.create_publisher(BasicTypes, 'chatter', 'foo')

    def test_create_subscription(self):
        self.node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg))
        self.node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg), 0)
        self.node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg), 1)
        self.node.create_subscription(
            BasicTypes, 'chatter', lambda msg: print(msg), qos_profile_sensor_data)
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not contain characters'):
            self.node.create_subscription(BasicTypes, 'chatter?', lambda msg: print(msg), 1)
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not start with a number'):
            self.node.create_subscription(BasicTypes, '/chatter/42ish', lambda msg: print(msg), 1)
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_subscription(BasicTypes, 'foo/{bad_sub}', lambda msg: print(msg), 1)
        with self.assertRaisesRegex(ValueError, 'must be greater than or equal to zero'):
            self.node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg), -1)
        with self.assertRaisesRegex(TypeError, 'Expected QoSProfile or int'):
            self.node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg), 'foo')

    def raw_subscription_callback(self, msg):
        print('Raw subscription callback: %s length %d' % (msg, len(msg)))
        self.raw_subscription_msg = msg

    def test_create_raw_subscription(self):
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)
        basic_types_pub = self.node.create_publisher(BasicTypes, 'raw_subscription_test', 1)
        self.raw_subscription_msg = None  # None=No result yet
        self.node.create_subscription(
            BasicTypes,
            'raw_subscription_test',
            self.raw_subscription_callback,
            raw=True
        )
        basic_types_msg = BasicTypes()
        cycle_count = 0
        while cycle_count < 5 and self.raw_subscription_msg is None:
            basic_types_pub.publish(basic_types_msg)
            cycle_count += 1
            executor.spin_once(timeout_sec=1)
        self.assertIsNotNone(self.raw_subscription_msg, 'raw subscribe timed out')
        self.assertIs(type(self.raw_subscription_msg), bytes, 'raw subscribe did not return bytes')
        # The length might be implementation dependant, but shouldn't be zero
        # There may be a canonical serialization in the future at which point this can be updated
        self.assertNotEqual(len(self.raw_subscription_msg), 0, 'raw subscribe invalid length')

        executor.shutdown()

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

    def test_deprecation_warnings(self):
        warnings.simplefilter('always')
        with warnings.catch_warnings(record=True) as w:
            self.node.create_publisher(BasicTypes, 'chatter')
            assert len(w) == 1
            assert issubclass(w[0].category, DeprecationWarning)
        with warnings.catch_warnings(record=True) as w:
            self.node.create_publisher(BasicTypes, 'chatter', qos_profile=qos_profile_sensor_data)
            assert len(w) == 1
            assert issubclass(w[0].category, DeprecationWarning)
        with warnings.catch_warnings(record=True) as w:
            self.node.create_publisher(BasicTypes, 'chatter', qos_profile=qos_profile_sensor_data)
            assert len(w) == 1
            assert issubclass(w[0].category, DeprecationWarning)
        with warnings.catch_warnings(record=True) as w:
            self.node.create_publisher(BasicTypes, 'chatter', qos_profile_default)
            assert len(w) == 1
            assert issubclass(w[0].category, DeprecationWarning)
        with warnings.catch_warnings(record=True) as w:
            self.node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg))
            assert len(w) == 1
            assert issubclass(w[0].category, DeprecationWarning)
        with warnings.catch_warnings(record=True) as w:
            self.node.create_subscription(
                BasicTypes, 'chatter', lambda msg: print(msg), qos_profile=qos_profile_sensor_data)
            assert len(w) == 1
            assert issubclass(w[0].category, DeprecationWarning)
        with warnings.catch_warnings(record=True) as w:
            self.node.create_subscription(
                BasicTypes, 'chatter', lambda msg: print(msg), qos_profile=qos_profile_sensor_data)
            assert len(w) == 1
            assert issubclass(w[0].category, DeprecationWarning)
        with warnings.catch_warnings(record=True) as w:
            self.node.create_subscription(
                BasicTypes, 'chatter', lambda msg: print(msg), qos_profile_default)
            assert len(w) == 1
            assert issubclass(w[0].category, DeprecationWarning)
        with warnings.catch_warnings(record=True) as w:
            self.node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg), raw=True)
            assert len(w) == 1
            assert issubclass(w[0].category, DeprecationWarning)
        warnings.simplefilter('default')

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

        self.node.create_publisher(BasicTypes, short_topic_name)
        self.assertEqual(1, self.node.count_publishers(short_topic_name))
        self.assertEqual(1, self.node.count_publishers(fq_topic_name))

        self.node.create_subscription(BasicTypes, short_topic_name, lambda msg: print(msg))
        self.assertEqual(1, self.node.count_subscribers(short_topic_name))
        self.assertEqual(1, self.node.count_subscribers(fq_topic_name))

        self.node.create_subscription(BasicTypes, short_topic_name, lambda msg: print(msg))
        self.assertEqual(2, self.node.count_subscribers(short_topic_name))
        self.assertEqual(2, self.node.count_subscribers(fq_topic_name))

        # error cases
        with self.assertRaisesRegex(TypeError, 'bad argument type for built-in operation'):
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
            ('/my_ns/my_node/describe_parameters', ['rcl_interfaces/srv/DescribeParameters']),
            service_names_and_types
        )
        self.assertIn(
            ('/my_ns/my_node/get_parameter_types', ['rcl_interfaces/srv/GetParameterTypes']),
            service_names_and_types
        )
        self.assertIn(
            ('/my_ns/my_node/get_parameters', ['rcl_interfaces/srv/GetParameters']),
            service_names_and_types
        )
        self.assertIn(
            ('/my_ns/my_node/list_parameters', ['rcl_interfaces/srv/ListParameters']),
            service_names_and_types
        )
        self.assertIn(
            ('/my_ns/my_node/set_parameters', ['rcl_interfaces/srv/SetParameters']),
            service_names_and_types
        )
        self.assertIn(
            (
                '/my_ns/my_node/set_parameters_atomically',
                ['rcl_interfaces/srv/SetParametersAtomically']
            ), service_names_and_types
        )


class TestNode(unittest.TestCase):

    @classmethod
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            TEST_NODE,
            namespace=TEST_NAMESPACE,
            context=self.context,
            initial_parameters=[
                Parameter('initial_foo', Parameter.Type.INTEGER, 4321),
                Parameter('initial_bar', Parameter.Type.STRING, 'init_param'),
                Parameter('initial_baz', Parameter.Type.DOUBLE, 3.14)
            ],
            automatically_declare_initial_parameters=False
        )

    @classmethod
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_declare_parameter(self):
        result_initial_foo = self.node.declare_parameter(
            'initial_foo', ParameterValue(), ParameterDescriptor())
        result_foo = self.node.declare_parameter(
            'foo', ParameterValue(
                type=Parameter.Type.INTEGER.value, integer_value=42), ParameterDescriptor())
        result_bar = self.node.declare_parameter(
            'bar', ParameterValue(
                type=Parameter.Type.STRING.value, string_value='hello'), ParameterDescriptor())
        result_baz = self.node.declare_parameter(
            'baz', ParameterValue(
                type=Parameter.Type.DOUBLE.value, double_value=2.41), ParameterDescriptor())

        # OK cases.
        self.assertIsInstance(result_initial_foo, Parameter)
        self.assertIsInstance(result_foo, Parameter)
        self.assertIsInstance(result_bar, Parameter)
        self.assertIsInstance(result_baz, Parameter)
        self.assertEqual(result_initial_foo.value, 4321)
        self.assertEqual(result_foo.value, 42)
        self.assertEqual(result_bar.value, 'hello')
        self.assertEqual(result_baz.value, 2.41)
        self.assertEqual(self.node.get_parameter('initial_foo').value, 4321)
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)

        # Error cases.
        with self.assertRaises(ParameterAlreadyDeclaredException):
            self.node.declare_parameter(
                'foo', ParameterValue(
                    type=Parameter.Type.STRING.value, string_value='raise'), ParameterDescriptor())
        with self.assertRaises(InvalidParameterException):
            self.node.declare_parameter(
                '123foo', ParameterValue(
                    type=Parameter.Type.STRING.value, string_value='raise'), ParameterDescriptor())
        with self.assertRaises(InvalidParameterException):
            self.node.declare_parameter(
                'foo??', ParameterValue(
                    type=Parameter.Type.STRING.value, string_value='raise'), ParameterDescriptor())

        self.node.set_parameters_callback(self.reject_parameter_callback)
        with self.assertRaises(InvalidParameterValueException):
            self.node.declare_parameter(
                'reject_me', ParameterValue(
                    type=Parameter.Type.STRING.value, string_value='raise'), ParameterDescriptor())

        with self.assertRaises(AssertionError):
            self.node.declare_parameter(
                1,
                ParameterValue(type=Parameter.Type.STRING.value, string_value='wrong_name_type'),
                ParameterDescriptor())

        with self.assertRaises(AssertionError):
            self.node.declare_parameter(
                'wrong_parameter_value_type', 1234, ParameterDescriptor())

        with self.assertRaises(AssertionError):
            self.node.declare_parameter(
                'wrong_parameter_descriptor_type', ParameterValue(), ParameterValue())

    def test_declare_parameters(self):
        parameters = [
            ('foo', ParameterValue(
                type=Parameter.Type.INTEGER.value, integer_value=42), ParameterDescriptor()),
            ('bar', ParameterValue(
                type=Parameter.Type.STRING.value, string_value='hello'), ParameterDescriptor()),
            ('baz', ParameterValue(
                type=Parameter.Type.DOUBLE.value, double_value=2.41), ParameterDescriptor()),
        ]

        result = self.node.declare_parameters('', parameters)

        # OK cases.
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], Parameter)
        self.assertIsInstance(result[1], Parameter)
        self.assertIsInstance(result[2], Parameter)
        self.assertEqual(result[0].value, 42)
        self.assertEqual(result[1].value, 'hello')
        self.assertEqual(result[2].value, 2.41)
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)

        result = self.node.declare_parameters('/namespace/', parameters)

        # OK cases.
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], Parameter)
        self.assertIsInstance(result[1], Parameter)
        self.assertIsInstance(result[2], Parameter)
        self.assertEqual(result[0].value, 42)
        self.assertEqual(result[1].value, 'hello')
        self.assertEqual(result[2].value, 2.41)
        self.assertEqual(self.node.get_parameter('/namespace/foo').value, 42)
        self.assertEqual(self.node.get_parameter('/namespace/bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('/namespace/baz').value, 2.41)

        # Error cases.
        with self.assertRaises(ParameterAlreadyDeclaredException):
            self.node.declare_parameters('', parameters)

        # Declare a new set of parameters; the first one is not already declared,
        # but 2nd and 3rd one are.
        parameters = [
            ('foobar', ParameterValue(
                type=Parameter.Type.INTEGER.value, integer_value=43), ParameterDescriptor()),
            ('bar', ParameterValue(
                type=Parameter.Type.STRING.value, string_value='hello'), ParameterDescriptor()),
            ('baz', ParameterValue(
                type=Parameter.Type.DOUBLE.value, double_value=2.41), ParameterDescriptor()),
        ]
        with self.assertRaises(ParameterAlreadyDeclaredException):
            self.node.declare_parameters('', parameters)

        # Declare a new set; the third one shall fail because of its name.
        parameters = [
            ('foobarbar', ParameterValue(
                type=Parameter.Type.INTEGER.value, integer_value=44), ParameterDescriptor()),
            ('barbarbar', ParameterValue(
                type=Parameter.Type.STRING.value, string_value='world'), ParameterDescriptor()),
            ('baz??wrong_name', ParameterValue(
                type=Parameter.Type.DOUBLE.value, double_value=2.41), ParameterDescriptor()),
        ]
        with self.assertRaises(InvalidParameterException):
            self.node.declare_parameters('', parameters)

        # Declare a new set; the third one shall be rejected by the callback.
        parameters = [
            ('im_ok', ParameterValue(
                type=Parameter.Type.INTEGER.value, integer_value=44), ParameterDescriptor()),
            ('im_also_ok', ParameterValue(
                type=Parameter.Type.STRING.value, string_value='world'), ParameterDescriptor()),
            ('reject_me', ParameterValue(
                type=Parameter.Type.DOUBLE.value, double_value=2.41), ParameterDescriptor()),
        ]
        self.node.set_parameters_callback(self.reject_parameter_callback)
        with self.assertRaises(InvalidParameterValueException):
            self.node.declare_parameters('', parameters)

        with self.assertRaises(AssertionError):
            self.node.declare_parameters(
                '',
                [(
                    1,
                    ParameterValue(
                        type=Parameter.Type.STRING.value, string_value='wrong_name_type'),
                    ParameterDescriptor()
                )]
            )

        with self.assertRaises(AssertionError):
            self.node.declare_parameters(
                '',
                [(
                    'wrong_parameter_value_type',
                    1234,
                    ParameterDescriptor()
                )]
            )

        with self.assertRaises(AssertionError):
            self.node.declare_parameters(
                '',
                [(
                    'wrong_parameter_descriptor_tpye',
                    ParameterValue(),
                    ParameterValue()
                )]
            )

    def reject_parameter_callback(self, parameter_list):
        rejected_parameters = (param for param in parameter_list if 'reject' in param.name)
        return SetParametersResult(successful=(not any(rejected_parameters)))

    def test_node_set_undeclared_parameters(self):
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.set_parameters([
                Parameter('foo', Parameter.Type.INTEGER, 42),
                Parameter('bar', Parameter.Type.STRING, 'hello'),
                Parameter('baz', Parameter.Type.DOUBLE, 2.41)
            ])

    def test_node_set_undeclared_parameters_atomically(self):
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.set_parameters_atomically([
                Parameter('foo', Parameter.Type.INTEGER, 42),
                Parameter('bar', Parameter.Type.STRING, 'hello'),
                Parameter('baz', Parameter.Type.DOUBLE, 2.41)
            ])

    def test_node_get_undeclared_parameter(self):
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.get_parameter('initial_foo')

    def test_node_get_undeclared_parameter_or(self):
        result = self.node.get_parameter_or(
            'initial_foo', Parameter('foo', Parameter.Type.INTEGER, 152))
        self.assertEqual(result.name, 'foo')
        self.assertEqual(result.value, 152)


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
