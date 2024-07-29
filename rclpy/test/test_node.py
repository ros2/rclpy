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
import platform
import time
import unittest
from unittest.mock import Mock
import warnings

import pytest

from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.exceptions import InvalidParameterException
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.exceptions import InvalidParameterValueException
from rclpy.exceptions import InvalidServiceNameException
from rclpy.exceptions import InvalidTopicNameException
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.exceptions import ParameterImmutableException
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.time_source import USE_SIM_TIME_NAME
from rclpy.utilities import get_rmw_implementation_identifier
from test_msgs.msg import BasicTypes

TEST_NODE = 'my_node'
TEST_NAMESPACE = '/my_ns'

TEST_RESOURCES_DIR = pathlib.Path(__file__).resolve().parent / 'resources' / 'test_node'


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
            1,
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

    def dummy_cb(self, msg):
        pass

    @unittest.skipIf(
        get_rmw_implementation_identifier() == 'rmw_connextdds' and platform.system() == 'Windows',
        reason='Source timestamp not implemented for Connext on Windows')
    def test_take(self):
        basic_types_pub = self.node.create_publisher(BasicTypes, 'take_test', 1)
        sub = self.node.create_subscription(
            BasicTypes,
            'take_test',
            self.dummy_cb,
            1)
        basic_types_msg = BasicTypes()
        basic_types_pub.publish(basic_types_msg)
        for i in range(5):
            with sub.handle:
                result = sub.handle.take_message(sub.msg_type, False)
            if result is not None:
                msg, info = result
                self.assertNotEqual(0, info['source_timestamp'])
                return
            else:
                time.sleep(0.2)

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

    def test_service_names_and_types_by_node(self):
        # test that it doesnt raise
        self.node.get_service_names_and_types_by_node(TEST_NODE, TEST_NAMESPACE)

    def test_client_names_and_types_by_node(self):
        # test that it doesnt raise
        self.node.get_client_names_and_types_by_node(TEST_NODE, TEST_NAMESPACE)

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

    def test_node_names_and_namespaces_with_enclaves(self):
        # test that it doesn't raise
        self.node.get_node_names_and_namespaces_with_enclaves()

    def assert_qos_equal(self, expected_qos_profile, actual_qos_profile, *, is_publisher):
        # Depth and history are skipped because they are not retrieved.
        self.assertEqual(
            expected_qos_profile.durability,
            actual_qos_profile.durability,
            'Durability is unequal')
        self.assertEqual(
            expected_qos_profile.reliability,
            actual_qos_profile.reliability,
            'Reliability is unequal')
        if is_publisher:
            self.assertEqual(
                expected_qos_profile.lifespan,
                actual_qos_profile.lifespan,
                'lifespan is unequal')
        self.assertEqual(
            expected_qos_profile.deadline,
            actual_qos_profile.deadline,
            'Deadline is unequal')
        self.assertEqual(
            expected_qos_profile.liveliness,
            actual_qos_profile.liveliness,
            'liveliness is unequal')
        self.assertEqual(
            expected_qos_profile.liveliness_lease_duration,
            actual_qos_profile.liveliness_lease_duration,
            'liveliness_lease_duration is unequal')

    def test_get_publishers_subscriptions_info_by_topic(self):
        topic_name = 'test_topic_endpoint_info'
        fq_topic_name = '{namespace}/{name}'.format(namespace=TEST_NAMESPACE, name=topic_name)
        # Lists should be empty
        self.assertFalse(self.node.get_publishers_info_by_topic(fq_topic_name))
        self.assertFalse(self.node.get_subscriptions_info_by_topic(fq_topic_name))

        # Add a publisher
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_ALL,
            deadline=Duration(seconds=1, nanoseconds=12345),
            lifespan=Duration(seconds=20, nanoseconds=9887665),
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            liveliness_lease_duration=Duration(seconds=5, nanoseconds=23456),
            liveliness=QoSLivelinessPolicy.MANUAL_BY_TOPIC)
        self.node.create_publisher(BasicTypes, topic_name, qos_profile)
        # List should have one item
        publisher_list = self.node.get_publishers_info_by_topic(fq_topic_name)
        self.assertEqual(1, len(publisher_list))
        # Subscription list should be empty
        self.assertFalse(self.node.get_subscriptions_info_by_topic(fq_topic_name))
        # Verify publisher list has the right data
        self.assertEqual(self.node.get_name(), publisher_list[0].node_name)
        self.assertEqual(self.node.get_namespace(), publisher_list[0].node_namespace)
        self.assertEqual('test_msgs/msg/BasicTypes', publisher_list[0].topic_type)
        actual_qos_profile = publisher_list[0].qos_profile
        self.assert_qos_equal(qos_profile, actual_qos_profile, is_publisher=True)

        # Add a subscription
        qos_profile2 = QoSProfile(
            depth=0,
            history=QoSHistoryPolicy.KEEP_LAST,
            deadline=Duration(seconds=15, nanoseconds=1678),
            lifespan=Duration(seconds=29, nanoseconds=2345),
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness_lease_duration=Duration(seconds=5, nanoseconds=23456),
            liveliness=QoSLivelinessPolicy.AUTOMATIC)
        self.node.create_subscription(BasicTypes, topic_name, lambda msg: print(msg), qos_profile2)
        # Both lists should have one item
        publisher_list = self.node.get_publishers_info_by_topic(fq_topic_name)
        subscription_list = self.node.get_subscriptions_info_by_topic(fq_topic_name)
        self.assertEqual(1, len(publisher_list))
        self.assertEqual(1, len(subscription_list))
        # Verify subscription list has the right data
        self.assertEqual(self.node.get_name(), publisher_list[0].node_name)
        self.assertEqual(self.node.get_namespace(), publisher_list[0].node_namespace)
        self.assertEqual('test_msgs/msg/BasicTypes', publisher_list[0].topic_type)
        self.assertEqual('test_msgs/msg/BasicTypes', subscription_list[0].topic_type)
        publisher_qos_profile = publisher_list[0].qos_profile
        subscription_qos_profile = subscription_list[0].qos_profile
        self.assert_qos_equal(qos_profile, publisher_qos_profile, is_publisher=True)
        self.assert_qos_equal(qos_profile2, subscription_qos_profile, is_publisher=False)

        # Error cases
        with self.assertRaises(TypeError):
            self.node.get_subscriptions_info_by_topic(1)
            self.node.get_publishers_info_by_topic(1)
        with self.assertRaisesRegex(ValueError, 'is invalid'):
            self.node.get_subscriptions_info_by_topic('13')
            self.node.get_publishers_info_by_topic('13')

    def test_count_publishers_subscribers(self):
        short_topic_name = 'chatter'
        fq_topic_name = '%s/%s' % (TEST_NAMESPACE, short_topic_name)

        self.assertEqual(0, self.node.count_publishers(fq_topic_name))
        self.assertEqual(0, self.node.count_subscribers(fq_topic_name))

        short_topic_publisher = self.node.create_publisher(BasicTypes, short_topic_name, 1)
        self.assertEqual(1, self.node.count_publishers(short_topic_name))
        self.assertEqual(1, self.node.count_publishers(fq_topic_name))
        self.assertEqual(0, short_topic_publisher.get_subscription_count())

        self.node.create_subscription(BasicTypes, short_topic_name, lambda msg: print(msg), 1)
        self.assertEqual(1, self.node.count_subscribers(short_topic_name))
        self.assertEqual(1, self.node.count_subscribers(fq_topic_name))
        self.assertEqual(1, short_topic_publisher.get_subscription_count())

        self.node.create_subscription(BasicTypes, short_topic_name, lambda msg: print(msg), 1)
        self.assertEqual(2, self.node.count_subscribers(short_topic_name))
        self.assertEqual(2, self.node.count_subscribers(fq_topic_name))
        self.assertEqual(2, short_topic_publisher.get_subscription_count())

        # error cases
        with self.assertRaises(TypeError):
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

    def test_describe_undeclared_parameter(self):
        self.assertFalse(self.node.has_parameter('foo'))

        descriptor = self.node.describe_parameter('foo')
        self.assertEqual(descriptor, ParameterDescriptor())

    def test_describe_undeclared_parameters(self):
        self.assertFalse(self.node.has_parameter('foo'))
        self.assertFalse(self.node.has_parameter('bar'))

        # Check list.
        descriptor_list = self.node.describe_parameters(['foo', 'bar'])
        self.assertIsInstance(descriptor_list, list)
        self.assertEqual(len(descriptor_list), 2)
        self.assertEqual(descriptor_list[0], ParameterDescriptor())
        self.assertEqual(descriptor_list[1], ParameterDescriptor())

    def test_node_get_parameter(self):
        self.node.set_parameters([Parameter('foo', Parameter.Type.INTEGER, 42)])
        self.assertIsInstance(self.node.get_parameter('foo'), Parameter)
        self.assertEqual(self.node.get_parameter('foo').value, 42)

    def test_node_get_parameter_returns_parameter_not_set(self):
        self.assertIsInstance(self.node.get_parameter('unset'), Parameter)
        self.assertEqual(self.node.get_parameter('unset').type_, Parameter.Type.NOT_SET)

    def test_node_declare_static_parameter(self):
        value = self.node.declare_parameter('an_integer', 5)
        self.assertEqual(value.value, 5)
        self.assertFalse(
            self.node.set_parameters([Parameter('an_integer', value='asd')])[0].successful)
        self.assertEqual(self.node.get_parameter('an_integer').value, 5)

    def test_node_undeclared_parameters_are_dynamically_typed(self):
        self.assertTrue(self.node.set_parameters([Parameter('my_param', value=5)])[0].successful)
        self.assertEqual(self.node.get_parameter('my_param').value, 5)
        self.assertTrue(
            self.node.set_parameters([Parameter('my_param', value='asd')])[0].successful)
        self.assertEqual(self.node.get_parameter('my_param').value, 'asd')

    def test_node_cannot_declare_after_set(self):
        self.assertTrue(self.node.set_parameters([Parameter('my_param', value=5)])[0].successful)
        self.assertEqual(self.node.get_parameter('my_param').value, 5)
        with pytest.raises(rclpy.exceptions.ParameterAlreadyDeclaredException):
            self.node.declare_parameter('my_param', 5)

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
            parameter_overrides=[
                Parameter('initial_foo', Parameter.Type.INTEGER, 4321),
                Parameter('initial_bar', Parameter.Type.STRING, 'init_param'),
                Parameter('initial_baz', Parameter.Type.DOUBLE, 3.14),
                Parameter('initial_decl_with_type', Parameter.Type.DOUBLE, 3.14),
                Parameter('initial_decl_wrong_type', Parameter.Type.DOUBLE, 3.14),
            ],
            cli_args=[
                '--ros-args', '-p', 'initial_fizz:=buzz',
                '--params-file', str(TEST_RESOURCES_DIR / 'test_parameters.yaml'),
                '-p', 'initial_buzz:=1.'
            ],
            automatically_declare_parameters_from_overrides=False
        )

    @classmethod
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_declare_parameter(self):
        with pytest.raises(ValueError):
            result_initial_foo = self.node.declare_parameter(
                'initial_foo', ParameterValue(), ParameterDescriptor())
        result_initial_foo = self.node.declare_parameter(
                'initial_foo', ParameterValue(), ParameterDescriptor(dynamic_typing=True))
        result_initial_bar = self.node.declare_parameter(
            'initial_bar', 'ignoring_override', ParameterDescriptor(), ignore_override=True)
        result_initial_fizz = self.node.declare_parameter(
            'initial_fizz', 'default', ParameterDescriptor())
        result_initial_baz = self.node.declare_parameter(
            'initial_baz', 0., ParameterDescriptor())
        result_initial_buzz = self.node.declare_parameter(
            'initial_buzz', 0., ParameterDescriptor())
        result_initial_foobar = self.node.declare_parameter(
            'initial_foobar', True, ParameterDescriptor())

        result_foo = self.node.declare_parameter(
            'foo', 42, ParameterDescriptor())
        result_bar = self.node.declare_parameter(
            'bar', 'hello', ParameterDescriptor())
        result_baz = self.node.declare_parameter(
            'baz', 2.41, ParameterDescriptor())
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter('always', category=UserWarning)
            result_value_not_set = self.node.declare_parameter('value_not_set')
            assert len(w) == 1
            assert issubclass(w[0].category, UserWarning)

        # OK cases.
        self.assertIsInstance(result_initial_foo, Parameter)
        self.assertIsInstance(result_initial_bar, Parameter)
        self.assertIsInstance(result_initial_fizz, Parameter)
        self.assertIsInstance(result_initial_baz, Parameter)
        self.assertIsInstance(result_foo, Parameter)
        self.assertIsInstance(result_bar, Parameter)
        self.assertIsInstance(result_baz, Parameter)
        self.assertIsInstance(result_value_not_set, Parameter)
        # initial_foo and initial_fizz get override values; initial_bar does not.
        self.assertEqual(result_initial_foo.value, 4321)
        self.assertEqual(result_initial_bar.value, 'ignoring_override')
        # provided by CLI, overridden by file
        self.assertEqual(result_initial_fizz.value, 'param_file_override')
        self.assertEqual(result_initial_baz.value, 3.14)  # provided by file, overridden manually
        self.assertEqual(result_initial_buzz.value, 1.)  # provided by CLI
        self.assertEqual(result_initial_foobar.value, False)  # provided by file
        self.assertEqual(result_foo.value, 42)
        self.assertEqual(result_bar.value, 'hello')
        self.assertEqual(result_baz.value, 2.41)
        self.assertIsNone(result_value_not_set.value)
        self.assertEqual(self.node.get_parameter('initial_foo').value, 4321)
        self.assertEqual(self.node.get_parameter('initial_bar').value, 'ignoring_override')
        self.assertEqual(self.node.get_parameter('initial_fizz').value, 'param_file_override')
        self.assertEqual(self.node.get_parameter('initial_baz').value, 3.14)
        self.assertEqual(self.node.get_parameter('initial_buzz').value, 1)
        self.assertEqual(self.node.get_parameter('initial_foobar').value, False)
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)
        self.assertIsNone(self.node.get_parameter('value_not_set').value)
        self.assertTrue(self.node.has_parameter('value_not_set'))

        # Error cases.
        # TODO(@jubeira): add failing test cases with invalid names once name
        # validation is implemented.
        with self.assertRaises(ParameterAlreadyDeclaredException):
            self.node.declare_parameter(
                'foo', 'raise', ParameterDescriptor())
        with self.assertRaises(InvalidParameterException):
            self.node.declare_parameter(
                '', 'raise', ParameterDescriptor())
        with self.assertRaises(InvalidParameterException):
            self.node.declare_parameter(
                '', 'raise', ParameterDescriptor())

        self.node.add_on_set_parameters_callback(self.reject_parameter_callback)
        with self.assertRaises(InvalidParameterValueException):
            self.node.declare_parameter(
                'reject_me', 'raise', ParameterDescriptor())

        with self.assertRaises(TypeError):
            self.node.declare_parameter(
                1,
                'wrong_name_type',
                ParameterDescriptor())

        with self.assertRaises(ValueError):
            self.node.declare_parameter(
                'wrong_parameter_value_type', ParameterValue(), ParameterDescriptor())

        with self.assertRaises(TypeError):
            self.node.declare_parameter(
                'wrong_parameter_descriptor_type', 1, ParameterValue())

        with self.assertRaises(ValueError):
            self.node.declare_parameter(
                'dynamic_typing_and_static_type',
                Parameter.Type.DOUBLE,
                descriptor=ParameterDescriptor(dynamic_typing=True))

    def test_declare_parameters(self):
        parameters = [
            ('initial_foo', 0, ParameterDescriptor()),
            ('foo', 42, ParameterDescriptor()),
            ('bar', 'hello', ParameterDescriptor()),
            ('baz', 2.41),
            ('value_not_set',)
        ]

        # Declare uninitialized parameter
        parameter_type = self.node.declare_parameter('no_override', Parameter.Type.INTEGER).type_
        assert parameter_type == Parameter.Type.NOT_SET

        with pytest.raises(InvalidParameterTypeException):
            self.node.declare_parameter('initial_decl_wrong_type', Parameter.Type.INTEGER)

        self.assertAlmostEqual(
            self.node.declare_parameter('initial_decl_with_type', Parameter.Type.DOUBLE).value,
            3.14)

        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter('always', category=UserWarning)
            result = self.node.declare_parameters('', parameters)
            assert len(w) == 1
            assert issubclass(w[0].category, UserWarning)

        # OK cases - using overrides.
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), len(parameters))
        self.assertIsInstance(result[0], Parameter)
        self.assertIsInstance(result[1], Parameter)
        self.assertIsInstance(result[2], Parameter)
        self.assertIsInstance(result[3], Parameter)
        self.assertIsInstance(result[4], Parameter)
        self.assertEqual(result[0].value, 4321)
        self.assertEqual(result[1].value, 42)
        self.assertEqual(result[2].value, 'hello')
        self.assertEqual(result[3].value, 2.41)
        self.assertIsNone(result[4].value)
        self.assertEqual(self.node.get_parameter('initial_foo').value, 4321)
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)
        self.assertIsNone(self.node.get_parameter('value_not_set').value)
        self.assertTrue(self.node.has_parameter('value_not_set'))

        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter('always', category=UserWarning)
            result = self.node.declare_parameters('namespace', parameters)
            assert len(w) == 1

        # OK cases.
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), len(parameters))
        self.assertIsInstance(result[0], Parameter)
        self.assertIsInstance(result[1], Parameter)
        self.assertIsInstance(result[2], Parameter)
        self.assertIsInstance(result[3], Parameter)
        self.assertIsInstance(result[4], Parameter)
        self.assertEqual(result[0].value, 4321)
        self.assertEqual(result[1].value, 42)
        self.assertEqual(result[2].value, 'hello')
        self.assertEqual(result[3].value, 2.41)
        self.assertIsNone(result[4].value)
        self.assertEqual(self.node.get_parameter('namespace.initial_foo').value, 4321)
        self.assertEqual(self.node.get_parameter('namespace.foo').value, 42)
        self.assertEqual(self.node.get_parameter('namespace.bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('namespace.baz').value, 2.41)
        self.assertIsNone(self.node.get_parameter('namespace.value_not_set').value)
        self.assertTrue(self.node.has_parameter('namespace.value_not_set'))

        parameters = [
            ('initial_bar', 'ignoring_override', ParameterDescriptor()),
            ('initial_baz', 'ignoring_override_again', ParameterDescriptor()),
        ]

        result = self.node.declare_parameters('', parameters, ignore_override=True)

        # OK cases - ignoring overrides.
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), len(parameters))
        self.assertIsInstance(result[0], Parameter)
        self.assertIsInstance(result[1], Parameter)
        self.assertEqual(result[0].value, 'ignoring_override')
        self.assertEqual(result[1].value, 'ignoring_override_again')
        self.assertEqual(self.node.get_parameter('initial_bar').value, 'ignoring_override')
        self.assertEqual(self.node.get_parameter('initial_baz').value, 'ignoring_override_again')

        # Error cases.
        with self.assertRaises(ParameterAlreadyDeclaredException):
            self.node.declare_parameters('', parameters)

        # Declare a new set of parameters; the first one is not already declared,
        # but 2nd and 3rd one are.
        parameters = [
            ('foobar', 43, ParameterDescriptor()),
            ('bar', 'hello', ParameterDescriptor()),
            ('baz', 2.41, ParameterDescriptor()),
        ]
        with self.assertRaises(ParameterAlreadyDeclaredException):
            self.node.declare_parameters('', parameters)

        # Declare a new set; the third one shall fail because of its name.
        parameters = [
            ('foobarbar', 44, ParameterDescriptor()),
            ('barbarbar', 'world', ParameterDescriptor()),
            ('', 2.41, ParameterDescriptor()),
        ]
        with self.assertRaises(InvalidParameterException):
            self.node.declare_parameters('', parameters)

        # Declare a new set; the third one shall be rejected by the callback.
        parameters = [
            ('im_ok', 44, ParameterDescriptor()),
            ('im_also_ok', 'world', ParameterDescriptor()),
            ('reject_me', 2.41, ParameterDescriptor()),
        ]
        self.node.add_on_set_parameters_callback(self.reject_parameter_callback)
        with self.assertRaises(InvalidParameterValueException):
            self.node.declare_parameters('', parameters)

        with self.assertRaises(TypeError):
            self.node.declare_parameters(
                '',
                [(
                    1,
                    'wrong_name_type',
                    ParameterDescriptor()
                )]
            )

        with self.assertRaises(ValueError):
            self.node.declare_parameters(
                '',
                [(
                    'wrong_parameter_value_type',
                    ParameterValue(),
                    ParameterDescriptor()
                )]
            )

        with self.assertRaises(TypeError):
            self.node.declare_parameters(
                '',
                [(
                    'wrong_parameter_descriptor_tpye',
                    ParameterValue(),
                    ParameterValue()
                )]
            )

        # Declare a parameter with parameter type 'Not Set'
        with self.assertRaises(ValueError):
            self.node.declare_parameter(
                'wrong_parameter_value_type_not_set', Parameter.Type.NOT_SET)

    def reject_parameter_callback(self, parameter_list):
        rejected_parameters = (param for param in parameter_list if 'reject' in param.name)
        return SetParametersResult(successful=(not any(rejected_parameters)))

    def reject_parameter_callback_1(self, parameter_list):
        rejected_parameters = (
            param for param in parameter_list if 'refuse' in param.name)
        return SetParametersResult(successful=(not any(rejected_parameters)))

    def test_use_sim_time(self):
        self.assertTrue(self.node.has_parameter(USE_SIM_TIME_NAME))
        self.assertFalse(self.node.get_parameter(USE_SIM_TIME_NAME).value)

        temp_node = rclpy.create_node(
            TEST_NODE + '2',
            namespace=TEST_NAMESPACE,
            context=self.context,
            parameter_overrides=[
                Parameter(USE_SIM_TIME_NAME, value=True),
            ],
            automatically_declare_parameters_from_overrides=False
        )
        # use_sim_time is declared automatically anyways; in this case using override value.
        self.assertTrue(temp_node.has_parameter(USE_SIM_TIME_NAME))
        self.assertTrue(temp_node.get_parameter(USE_SIM_TIME_NAME).value)
        temp_node.destroy_node()

    def test_node_undeclare_parameter_has_parameter(self):
        # Undeclare unexisting parameter.
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.undeclare_parameter('foo')

        # Verify that it doesn't exist.
        self.assertFalse(self.node.has_parameter('foo'))

        # Declare parameter, verify existance, undeclare, and verify again.
        self.node.declare_parameter(
            'foo',
            'hello',
            ParameterDescriptor()
        )
        self.assertTrue(self.node.has_parameter('foo'))
        self.node.undeclare_parameter('foo')
        self.assertFalse(self.node.has_parameter('foo'))

        # Try with a read only parameter.
        self.assertFalse(self.node.has_parameter('immutable_foo'))
        self.node.declare_parameter(
            'immutable_foo',
            'I am immutable',
            ParameterDescriptor(read_only=True)
        )
        with self.assertRaises(ParameterImmutableException):
            self.node.undeclare_parameter('immutable_foo')

        # Verify that it still exists with the same value.
        self.assertTrue(self.node.has_parameter('immutable_foo'))
        self.assertEqual(self.node.get_parameter('immutable_foo').value, 'I am immutable')

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

    def test_node_get_uninitialized_parameter_or(self):
        # Statically typed parameter
        self.node.declare_parameter('uninitialized_foo', Parameter.Type.INTEGER)
        result = self.node.get_parameter_or(
            'uninitialized_foo', Parameter('foo', Parameter.Type.INTEGER, 152))
        self.assertEqual(result.name, 'foo')
        self.assertEqual(result.value, 152)

        # Dynamically typed parameter
        self.node.declare_parameter(
            'uninitialized_bar', None, ParameterDescriptor(dynamic_typing=True))
        result = self.node.get_parameter_or(
            'uninitialized_bar', Parameter('foo', Parameter.Type.INTEGER, 153))
        self.assertEqual(result.name, 'foo')
        self.assertEqual(result.value, 153)

    def test_node_get_parameters_by_prefix(self):
        parameters = [
            ('foo_prefix.foo', 43),
            ('foo_prefix.bar', 'hello'),
            ('foo_prefix.baz', 2.41),
            ('bar_prefix.foo', 1),
            ('bar_prefix.bar', 12.3),
            ('bar_prefix.baz', 'world'),
        ]
        self.node.declare_parameters('', parameters)

        parameters = self.node.get_parameters_by_prefix('foo_prefix')
        self.assertIsInstance(parameters, dict)
        self.assertEqual(len(parameters), 3)
        self.assertDictEqual(
            parameters,
            {
                'foo': self.node.get_parameter('foo_prefix.foo'),
                'bar': self.node.get_parameter('foo_prefix.bar'),
                'baz': self.node.get_parameter('foo_prefix.baz')
            }
        )

        parameters = self.node.get_parameters_by_prefix('bar_prefix')
        self.assertIsInstance(parameters, dict)
        self.assertEqual(len(parameters), 3)
        self.assertDictEqual(
            parameters,
            {
                'foo': self.node.get_parameter('bar_prefix.foo'),
                'bar': self.node.get_parameter('bar_prefix.bar'),
                'baz': self.node.get_parameter('bar_prefix.baz')
            }
        )

        parameters = self.node.get_parameters_by_prefix('')
        self.assertIsInstance(parameters, dict)
        # use_sim_time is automatically declared.
        self.assertDictEqual(
            parameters,
            {
                'foo_prefix.foo': self.node.get_parameter('foo_prefix.foo'),
                'foo_prefix.bar': self.node.get_parameter('foo_prefix.bar'),
                'foo_prefix.baz': self.node.get_parameter('foo_prefix.baz'),
                'bar_prefix.foo': self.node.get_parameter('bar_prefix.foo'),
                'bar_prefix.bar': self.node.get_parameter('bar_prefix.bar'),
                'bar_prefix.baz': self.node.get_parameter('bar_prefix.baz'),
                USE_SIM_TIME_NAME: self.node.get_parameter(USE_SIM_TIME_NAME)
            }
        )

        parameters = self.node.get_parameters_by_prefix('baz')
        self.assertFalse(parameters)
        self.assertIsInstance(parameters, dict)

    def test_node_set_parameters(self):
        integer_value = 42
        string_value = 'hello'
        float_value = 2.41
        parameter_tuples = [
            (
                'foo',
                integer_value,
                ParameterDescriptor()
            ),
            (
                'bar',
                string_value,
                ParameterDescriptor()
            ),
            (
                'baz',
                float_value,
                ParameterDescriptor()
            )
        ]

        # Create rclpy.Parameter list from tuples.
        parameters = [
            Parameter(
                name=parameter_tuples[0][0],
                value=integer_value
            ),
            Parameter(
                name=parameter_tuples[1][0],
                value=string_value
            ),
            Parameter(
                name=parameter_tuples[2][0],
                value=float_value
            ),
        ]

        with self.assertRaises(ParameterNotDeclaredException):
            self.node.set_parameters(parameters)

        self.node.declare_parameters('', parameter_tuples)
        result = self.node.set_parameters(parameters)

        # OK cases: check successful result and parameter value for each parameter set.
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], SetParametersResult)
        self.assertIsInstance(result[1], SetParametersResult)
        self.assertIsInstance(result[2], SetParametersResult)
        self.assertTrue(result[0].successful)
        self.assertTrue(result[1].successful)
        self.assertTrue(result[2].successful)
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)

        # Now we modify the declared parameters, add a new one and set them again.
        integer_value = 24
        string_value = 'bye'
        float_value = 1.42
        extra_value = 2.71
        parameter_tuples.append(
            (
                'foobar',
                extra_value,
                ParameterDescriptor())
            )
        parameters = [
            Parameter(
                name=parameter_tuples[0][0],
                value=integer_value
            ),
            Parameter(
                name=parameter_tuples[1][0],
                value=string_value
            ),
            Parameter(
                name=parameter_tuples[2][0],
                value=float_value
            ),
            Parameter(
                name=parameter_tuples[3][0],
                value=float_value
            ),
        ]
        # The first three parameters should have been set; the fourth one causes the exception.
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.set_parameters(parameters)

        # Validate first three.
        self.assertEqual(self.node.get_parameter('foo').value, 24)
        self.assertEqual(self.node.get_parameter('bar').value, 'bye')
        self.assertEqual(self.node.get_parameter('baz').value, 1.42)

        # Confirm that the fourth one does not exist.
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.get_parameter('foobar')

    def test_node_set_parameters_rejection(self):
        # Declare a new parameter and set a callback so that it's rejected when set.
        reject_parameter_tuple = (
            'reject_me',
            True,
            ParameterDescriptor()
        )
        self.node.declare_parameter(*reject_parameter_tuple)
        self.node.add_on_set_parameters_callback(self.reject_parameter_callback)
        result = self.node.set_parameters(
            [
                Parameter(
                    name=reject_parameter_tuple[0],
                    value=reject_parameter_tuple[1]
                )
            ]
        )
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], SetParametersResult)
        self.assertFalse(result[0].successful)

    def test_node_set_parameters_rejection_list(self):
        # Declare a new parameters and set a list of callbacks so that it's rejected when set.
        reject_list_parameter_tuple = [
            ('reject', True, ParameterDescriptor()),
            ('accept', True, ParameterDescriptor()),
            ('accept', True, ParameterDescriptor())
        ]
        self.node.declare_parameters('', reject_list_parameter_tuple)
        self.node.add_on_set_parameters_callback(self.reject_parameter_callback)
        self.node.add_on_set_parameters_callback(self.reject_parameter_callback_1)

        result = self.node.set_parameters(
            [
                Parameter(
                    name=reject_list_parameter_tuple[0][0],
                    value=reject_list_parameter_tuple[0][1]
                ),
                Parameter(
                    name=reject_list_parameter_tuple[1][0],
                    value=reject_list_parameter_tuple[1][1]
                ),
                Parameter(
                    name=reject_list_parameter_tuple[2][0],
                    value=reject_list_parameter_tuple[2][1]
                )
            ]
        )
        self.assertEqual(3, len(result))
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], SetParametersResult)
        self.assertFalse(result[0].successful)
        self.assertIsInstance(result[1], SetParametersResult)
        self.assertTrue(result[1].successful)
        self.assertIsInstance(result[2], SetParametersResult)
        self.assertTrue(result[2].successful)

    def test_node_add_on_set_parameter_callback(self):
        # Add callbacks to the list of callbacks.
        callbacks = [
            self.reject_parameter_callback,
            self.reject_parameter_callback_1
        ]
        for callback in callbacks:
            self.node.add_on_set_parameters_callback(callback)
        for callback in callbacks:
            self.assertTrue(callback in self.node._parameters_callbacks)
        for callback in callbacks:
            self.node.remove_on_set_parameters_callback(callback)

        # Adding the parameters which will be accepted without any rejections.
        non_reject_parameter_tuple = [
            ('accept_1', True, ParameterDescriptor()),
            ('accept_2', True, ParameterDescriptor())
        ]
        self.node.declare_parameters('', non_reject_parameter_tuple)

        for callback in callbacks:
            self.node.add_on_set_parameters_callback(callback)

        result = self.node.set_parameters(
            [
                Parameter(
                    name=non_reject_parameter_tuple[0][0],
                    value=non_reject_parameter_tuple[0][1]
                ),
                Parameter(
                    name=non_reject_parameter_tuple[1][0],
                    value=non_reject_parameter_tuple[1][1]
                )
            ]
        )
        self.assertEqual(2, len(result))
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], SetParametersResult)
        self.assertTrue(result[0].successful)
        self.assertIsInstance(result[1], SetParametersResult)
        self.assertTrue(result[1].successful)

    def test_node_remove_from_set_callback(self):
        # Remove callbacks from list of callbacks.
        parameter_tuple = (
            'refuse', True, ParameterDescriptor()
        )
        self.node.declare_parameter(*parameter_tuple)

        callbacks = [
            self.reject_parameter_callback_1,
        ]
        # Checking if the callbacks are not already present.
        for callback in callbacks:
            self.assertFalse(callback in self.node._parameters_callbacks)

        for callback in callbacks:
            self.node.add_on_set_parameters_callback(callback)

        result = self.node.set_parameters(
            [
                Parameter(
                    name=parameter_tuple[0],
                    value=parameter_tuple[1]
                )
            ]
        )
        self.assertEqual(1, len(result))
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], SetParametersResult)
        self.assertFalse(result[0].successful)
        # Removing the callback which is causing the rejection.
        self.node.remove_on_set_parameters_callback(self.reject_parameter_callback_1)
        self.assertFalse(self.reject_parameter_callback_1 in self.node._parameters_callbacks)
        # Now the setting its value again.
        result = self.node.set_parameters(
            [
                Parameter(
                    name=parameter_tuple[0],
                    value=parameter_tuple[1]
                )
            ]
        )
        self.assertEqual(1, len(result))
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], SetParametersResult)
        self.assertTrue(result[0].successful)

    def test_node_set_parameters_read_only(self):
        integer_value = 42
        string_value = 'hello'
        float_value = 2.41
        parameter_tuples = [
            (
                'immutable_foo',
                integer_value,
                ParameterDescriptor(read_only=True)
            ),
            (
                'bar',
                string_value,
                ParameterDescriptor()
            ),
            (
                'immutable_baz',
                float_value,
                ParameterDescriptor(read_only=True)
            )
        ]

        # Create rclpy.Parameter list from tuples.
        parameters = [
            Parameter(
                name=parameter_tuples[0][0],
                value=integer_value
            ),
            Parameter(
                name=parameter_tuples[1][0],
                value=string_value
            ),
            Parameter(
                name=parameter_tuples[2][0],
                value=float_value
            ),
        ]

        self.node.declare_parameters('', parameter_tuples)

        # Try setting a different value to the declared parameters.
        integer_value = 24
        string_value = 'bye'

        float_value = 1.42

        # Re-create parameters with modified values.
        parameters = [
            Parameter(
                name=parameter_tuples[0][0],
                value=integer_value
            ),
            Parameter(
                name=parameter_tuples[1][0],
                value=string_value
            ),
            Parameter(
                name=parameter_tuples[2][0],
                value=float_value
            ),
        ]

        result = self.node.set_parameters(parameters)

        # Only the parameter that is not read_only should have succeeded.
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], SetParametersResult)
        self.assertIsInstance(result[1], SetParametersResult)
        self.assertIsInstance(result[2], SetParametersResult)
        self.assertFalse(result[0].successful)
        self.assertTrue(result[1].successful)
        self.assertFalse(result[2].successful)
        self.assertEqual(self.node.get_parameter('immutable_foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'bye')
        self.assertEqual(self.node.get_parameter('immutable_baz').value, 2.41)

    def test_node_set_parameters_implicit_undeclare(self):
        parameter_tuples = [
            (
                'foo',
                42,
                ParameterDescriptor()
            ),
            (
                'bar',
                'hello',
                ParameterDescriptor(dynamic_typing=True)
            ),
            (
                'baz',
                2.41,
                ParameterDescriptor()
            )
        ]

        self.node.declare_parameters('', parameter_tuples)

        # Verify that the parameters are set.
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)

        # Now undeclare one of them implicitly.
        self.node.set_parameters([Parameter('bar', Parameter.Type.NOT_SET, None)])
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertFalse(self.node.has_parameter('bar'))
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)

    def test_node_set_parameters_atomically(self):
        integer_value = 42
        string_value = 'hello'
        float_value = 2.41
        parameter_tuples = [
            (
                'foo',
                integer_value,
                ParameterDescriptor()
            ),
            (
                'bar',
                string_value,
                ParameterDescriptor()
            ),
            (
                'baz',
                float_value,
                ParameterDescriptor()
            )
        ]

        # Create rclpy.Parameter list from tuples.
        parameters = [
            Parameter(
                name=parameter_tuples[0][0],
                value=integer_value
            ),
            Parameter(
                name=parameter_tuples[1][0],
                value=string_value
            ),
            Parameter(
                name=parameter_tuples[2][0],
                value=float_value
            ),
        ]

        with self.assertRaises(ParameterNotDeclaredException):
            self.node.set_parameters_atomically(parameters)

        self.node.declare_parameters('', parameter_tuples)
        result = self.node.set_parameters_atomically(parameters)

        # OK case: check successful aggregated result.
        self.assertIsInstance(result, SetParametersResult)
        self.assertTrue(result.successful)
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)

        # Now we modify the declared parameters, add a new one and set them again.
        integer_value = 24
        string_value = 'bye'
        float_value = 1.42
        extra_value = 2.71
        parameter_tuples.append(
            (
                'foobar',
                extra_value,
                ParameterDescriptor())
            )
        parameters = [
            Parameter(
                name=parameter_tuples[0][0],
                value=integer_value
            ),
            Parameter(
                name=parameter_tuples[1][0],
                value=string_value
            ),
            Parameter(
                name=parameter_tuples[2][0],
                value=float_value
            ),
            Parameter(
                name=parameter_tuples[3][0],
                value=float_value
            ),
        ]

        # The fourth parameter causes the exception, hence none is set.
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.set_parameters_atomically(parameters)

        # Confirm that the first three were not modified.
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)

        # Confirm that the fourth one does not exist.
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.get_parameter('foobar')

    def test_node_set_parameters_atomically_rejection(self):
        # Declare a new parameter and set a callback so that it's rejected when set.
        reject_parameter_tuple = (
            'reject_me',
            True,
            ParameterDescriptor()
        )

        self.node.declare_parameter(*reject_parameter_tuple)
        self.node.add_on_set_parameters_callback(self.reject_parameter_callback)
        result = self.node.set_parameters_atomically(
            [
                Parameter(
                    name=reject_parameter_tuple[0],
                    value=reject_parameter_tuple[1]
                )
            ]
        )
        self.assertIsInstance(result, SetParametersResult)
        self.assertFalse(result.successful)

    def test_node_set_parameters_atomically_read_only(self):
        integer_value = 42
        string_value = 'hello'
        float_value = 2.41
        parameter_tuples = [
            (
                'foo',
                integer_value,
                ParameterDescriptor()
            ),
            (
                'bar',
                string_value,
                ParameterDescriptor()
            ),
            (
                'immutable_baz',
                float_value,
                ParameterDescriptor(read_only=True)
            )
        ]

        # Create rclpy.Parameter list from tuples.
        parameters = [
            Parameter(
                name=parameter_tuples[0][0],
                value=integer_value
            ),
            Parameter(
                name=parameter_tuples[1][0],
                value=string_value
            ),
            Parameter(
                name=parameter_tuples[2][0],
                value=float_value
            ),
        ]

        self.node.declare_parameters('', parameter_tuples)

        # Try setting a different value to the declared parameters.
        integer_value = 24
        string_value = 'bye'
        float_value = 1.42

        parameters = [
            Parameter(
                name=parameter_tuples[0][0],
                value=integer_value
            ),
            Parameter(
                name=parameter_tuples[1][0],
                value=string_value
            ),
            Parameter(
                name=parameter_tuples[2][0],
                value=float_value
            ),
        ]

        result = self.node.set_parameters_atomically(parameters)

        # At least one parameter is read-only, so the overall result should be a failure.
        # All the parameters should have their original value.
        self.assertIsInstance(result, SetParametersResult)
        self.assertFalse(result.successful)
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('immutable_baz').value, 2.41)

    def test_node_set_parameters_atomically_implicit_undeclare(self):
        parameter_tuples = [
            (
                'foo',
                42,
                ParameterDescriptor()
            ),
            (
                'bar',
                'hello',
                ParameterDescriptor(dynamic_typing=True)
            ),
            (
                'baz',
                2.41,
                ParameterDescriptor()
            )
        ]

        self.node.declare_parameters('', parameter_tuples)

        # Verify that the parameters are set.
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertEqual(self.node.get_parameter('bar').value, 'hello')
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)

        # Now undeclare one of them implicitly.
        result = self.node.set_parameters_atomically([
            Parameter('bar', Parameter.Type.NOT_SET, None)])
        self.assertEqual(result.successful, True)
        self.assertEqual(self.node.get_parameter('foo').value, 42)
        self.assertFalse(self.node.has_parameter('bar'))
        self.assertEqual(self.node.get_parameter('baz').value, 2.41)

    def test_describe_parameter(self):
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.describe_parameter('foo')

        # Declare parameter with descriptor.
        self.node.declare_parameter(
            'foo',
            'hello',
            ParameterDescriptor(
                name='foo',
                type=ParameterType.PARAMETER_STRING,
                additional_constraints='some constraints',
                read_only=True,
                floating_point_range=[FloatingPointRange(from_value=-2.0, to_value=2.0, step=0.1)],
                integer_range=[IntegerRange(from_value=-10, to_value=10, step=2)]
            )
        )

        descriptor = self.node.describe_parameter('foo')
        self.assertEqual(descriptor.name, 'foo')
        self.assertEqual(descriptor.type, ParameterType.PARAMETER_STRING)
        self.assertEqual(descriptor.additional_constraints, 'some constraints')
        self.assertEqual(descriptor.read_only, True)
        self.assertEqual(descriptor.floating_point_range[0].from_value, -2.0)
        self.assertEqual(descriptor.floating_point_range[0].to_value, 2.0)
        self.assertEqual(descriptor.floating_point_range[0].step, 0.1)
        self.assertEqual(descriptor.integer_range[0].from_value, -10)
        self.assertEqual(descriptor.integer_range[0].to_value, 10)
        self.assertEqual(descriptor.integer_range[0].step, 2)

    def test_describe_parameters(self):
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.describe_parameter('foo')
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.describe_parameter('bar')

        # Declare parameters with descriptors.
        self.node.declare_parameter(
            'foo',
            'hello',
            ParameterDescriptor(
                name='foo',
                type=ParameterType.PARAMETER_STRING,
                additional_constraints='some constraints',
                read_only=True,
                floating_point_range=[FloatingPointRange(from_value=-2.0, to_value=2.0, step=0.1)],
                integer_range=[IntegerRange(from_value=-10, to_value=10, step=2)]
            )
        )
        self.node.declare_parameter(
            'bar',
            10,
            ParameterDescriptor(
                name='bar',
                type=ParameterType.PARAMETER_DOUBLE,
                additional_constraints='some more constraints',
                read_only=True,
                floating_point_range=[FloatingPointRange(from_value=-3.0, to_value=3.0, step=0.3)],
                integer_range=[IntegerRange(from_value=-20, to_value=20, step=3)]
            )
        )

        # Check list.
        descriptor_list = self.node.describe_parameters(['foo', 'bar'])
        self.assertIsInstance(descriptor_list, list)
        self.assertEqual(len(descriptor_list), 2)

        # Check individual descriptors.
        foo_descriptor = descriptor_list[0]
        self.assertEqual(foo_descriptor.name, 'foo')
        self.assertEqual(foo_descriptor.type, ParameterType.PARAMETER_STRING)
        self.assertEqual(foo_descriptor.additional_constraints, 'some constraints')
        self.assertEqual(foo_descriptor.read_only, True)
        self.assertEqual(foo_descriptor.floating_point_range[0].from_value, -2.0)
        self.assertEqual(foo_descriptor.floating_point_range[0].to_value, 2.0)
        self.assertEqual(foo_descriptor.floating_point_range[0].step, 0.1)
        self.assertEqual(foo_descriptor.integer_range[0].from_value, -10)
        self.assertEqual(foo_descriptor.integer_range[0].to_value, 10)
        self.assertEqual(foo_descriptor.integer_range[0].step, 2)

        # The descriptor gets the type of the parameter.
        bar_descriptor = descriptor_list[1]
        self.assertEqual(bar_descriptor.name, 'bar')
        self.assertEqual(bar_descriptor.type, ParameterType.PARAMETER_INTEGER)
        self.assertEqual(bar_descriptor.additional_constraints, 'some more constraints')
        self.assertEqual(bar_descriptor.read_only, True)
        self.assertEqual(bar_descriptor.floating_point_range[0].from_value, -3.0)
        self.assertEqual(bar_descriptor.floating_point_range[0].to_value, 3.0)
        self.assertEqual(bar_descriptor.floating_point_range[0].step, 0.3)
        self.assertEqual(bar_descriptor.integer_range[0].from_value, -20)
        self.assertEqual(bar_descriptor.integer_range[0].to_value, 20)
        self.assertEqual(bar_descriptor.integer_range[0].step, 3)

    def test_set_descriptor(self):
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.set_descriptor('foo', ParameterDescriptor())

        # Declare parameter with default descriptor.
        # The name and type of the stored descriptor shall match the parameter,
        self.node.declare_parameter(
            'foo',
            'hello',
            ParameterDescriptor()
        )
        self.assertEqual(
            self.node.describe_parameter('foo'),
            ParameterDescriptor(name='foo', type=Parameter.Type.STRING.value)
        )

        # Now modify the descriptor and check again.
        value = self.node.set_descriptor(
            'foo',
            ParameterDescriptor(
                name='this will be ignored',
                type=ParameterType.PARAMETER_INTEGER,  # Type will be ignored too.
                additional_constraints='some constraints',
                read_only=False,
                integer_range=[IntegerRange(from_value=-10, to_value=10, step=2)],
                dynamic_typing=True
            )
        )
        self.assertEqual(value.type, Parameter.Type.STRING.value)
        self.assertEqual(value.string_value, 'hello')

        # Name and type will match the parameter, not the given descriptor.
        descriptor = self.node.describe_parameter('foo')
        self.assertEqual(descriptor.name, 'foo')
        self.assertEqual(descriptor.type, ParameterType.PARAMETER_STRING)
        self.assertEqual(descriptor.additional_constraints, 'some constraints')
        self.assertEqual(descriptor.read_only, False)
        self.assertEqual(descriptor.integer_range[0].from_value, -10)
        self.assertEqual(descriptor.integer_range[0].to_value, 10)
        self.assertEqual(descriptor.integer_range[0].step, 2)

        # A descriptor that is not read-only can be replaced by a read-only one.
        value = self.node.set_descriptor(
            'foo',
            ParameterDescriptor(
                name='bar',
                type=ParameterType.PARAMETER_STRING,
                additional_constraints='some more constraints',
                read_only=True,
                floating_point_range=[FloatingPointRange(from_value=-2.0, to_value=2.0, step=0.1)],
            )
        )
        self.assertEqual(value.type, Parameter.Type.STRING.value)
        self.assertEqual(value.string_value, 'hello')

        descriptor = self.node.describe_parameter('foo')
        self.assertEqual(descriptor.name, 'foo')
        self.assertEqual(descriptor.type, ParameterType.PARAMETER_STRING)
        self.assertEqual(descriptor.additional_constraints, 'some more constraints')
        self.assertEqual(descriptor.read_only, True)
        self.assertEqual(descriptor.floating_point_range[0].from_value, -2.0)
        self.assertEqual(descriptor.floating_point_range[0].to_value, 2.0)
        self.assertEqual(descriptor.floating_point_range[0].step, 0.1)

    def test_set_descriptor_read_only(self):
        with self.assertRaises(ParameterNotDeclaredException):
            self.node.set_descriptor('foo', ParameterDescriptor())

        # Declare parameter with a read_only descriptor.
        self.node.declare_parameter(
            'foo',
            'hello',
            ParameterDescriptor(read_only=True)
        )
        self.assertEqual(
            self.node.describe_parameter('foo'),
            ParameterDescriptor(name='foo', type=Parameter.Type.STRING.value, read_only=True)
        )

        # Try modifying the descriptor.
        with self.assertRaises(ParameterImmutableException):
            self.node.set_descriptor(
                'foo',
                ParameterDescriptor(
                    name='foo',
                    type=ParameterType.PARAMETER_STRING,
                    additional_constraints='some constraints',
                    read_only=False,
                )
            )

    def test_floating_point_range_descriptor(self):
        # OK cases; non-floats are not affected by the range.
        fp_range = FloatingPointRange(from_value=0.0, to_value=10.0, step=0.5)
        parameters = [
            ('from_value', 0.0, ParameterDescriptor(floating_point_range=[fp_range])),
            ('to_value', 10.0, ParameterDescriptor(floating_point_range=[fp_range])),
            ('in_range', 4.5, ParameterDescriptor(floating_point_range=[fp_range])),
            ('str_value', 'I am no float', ParameterDescriptor(floating_point_range=[fp_range])),
            ('int_value', 123, ParameterDescriptor(floating_point_range=[fp_range]))
        ]

        result = self.node.declare_parameters('', parameters)

        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], Parameter)
        self.assertIsInstance(result[1], Parameter)
        self.assertIsInstance(result[2], Parameter)
        self.assertIsInstance(result[3], Parameter)
        self.assertIsInstance(result[4], Parameter)
        self.assertAlmostEqual(result[0].value, 0.0)
        self.assertAlmostEqual(result[1].value, 10.0)
        self.assertAlmostEqual(result[2].value, 4.5)
        self.assertEqual(result[3].value, 'I am no float')
        self.assertEqual(result[4].value, 123)
        self.assertEqual(self.node.get_parameter('from_value').value, 0.0)
        self.assertEqual(self.node.get_parameter('to_value').value, 10.0)
        self.assertEqual(self.node.get_parameter('in_range').value, 4.5)
        self.assertEqual(self.node.get_parameter('str_value').value, 'I am no float')
        self.assertAlmostEqual(self.node.get_parameter('int_value').value, 123)

        # Try to set a parameter out of range.
        result = self.node.set_parameters([Parameter('in_range', value=12.0)])
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], SetParametersResult)
        self.assertFalse(result[0].successful)
        self.assertEqual(self.node.get_parameter('in_range').value, 4.5)

        # Try to set a parameter out of range (bad step).
        result = self.node.set_parameters([Parameter('in_range', value=4.25)])
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], SetParametersResult)
        self.assertFalse(result[0].successful)
        self.assertEqual(self.node.get_parameter('in_range').value, 4.5)

        # From and to are always valid.
        # Parameters that don't comply with the description will raise an exception.
        fp_range = FloatingPointRange(from_value=-10.0, to_value=0.0, step=30.0)
        parameters = [
            ('from_value_2', -10.0, ParameterDescriptor(floating_point_range=[fp_range])),
            ('to_value_2', 0.0, ParameterDescriptor(floating_point_range=[fp_range])),
            ('in_range_bad_step', -4.5, ParameterDescriptor(floating_point_range=[fp_range])),
            ('out_of_range', 30.0, ParameterDescriptor(floating_point_range=[fp_range]))
        ]
        with self.assertRaises(InvalidParameterValueException):
            self.node.declare_parameters('', parameters)

        self.assertAlmostEqual(self.node.get_parameter('from_value_2').value, -10.0)
        self.assertAlmostEqual(self.node.get_parameter('to_value_2').value, 0.0)
        self.assertFalse(self.node.has_parameter('in_range_bad_step'))
        self.assertFalse(self.node.has_parameter('out_of_range'))

        # Try some more parameters with no step.
        fp_range = FloatingPointRange(from_value=-10.0, to_value=10.0, step=0.0)
        parameters = [
            ('from_value_no_step', -10.0, ParameterDescriptor(floating_point_range=[fp_range])),
            ('to_value_no_step', 10.0, ParameterDescriptor(floating_point_range=[fp_range])),
            ('in_range_no_step', 5.37, ParameterDescriptor(floating_point_range=[fp_range])),
        ]

        result = self.node.declare_parameters('', parameters)

        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], Parameter)
        self.assertIsInstance(result[1], Parameter)
        self.assertIsInstance(result[2], Parameter)
        self.assertAlmostEqual(result[0].value, -10.0)
        self.assertAlmostEqual(result[1].value, 10.0)
        self.assertAlmostEqual(result[2].value, 5.37)
        self.assertAlmostEqual(self.node.get_parameter('from_value_no_step').value, -10.0)
        self.assertAlmostEqual(self.node.get_parameter('to_value_no_step').value, 10.0)
        self.assertAlmostEqual(self.node.get_parameter('in_range_no_step').value, 5.37)

    def test_integer_range_descriptor(self):
        # OK cases; non-integers are not affected by the range.
        integer_range = IntegerRange(from_value=0, to_value=10, step=2)
        parameters = [
            ('from_value', 0, ParameterDescriptor(integer_range=[integer_range])),
            ('to_value', 10, ParameterDescriptor(integer_range=[integer_range])),
            ('in_range', 4, ParameterDescriptor(integer_range=[integer_range])),
            ('str_value', 'I am no integer', ParameterDescriptor(integer_range=[integer_range])),
            ('float_value', 123.0, ParameterDescriptor(integer_range=[integer_range]))
        ]

        result = self.node.declare_parameters('', parameters)

        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], Parameter)
        self.assertIsInstance(result[1], Parameter)
        self.assertIsInstance(result[2], Parameter)
        self.assertIsInstance(result[3], Parameter)
        self.assertIsInstance(result[4], Parameter)
        self.assertEqual(result[0].value, 0)
        self.assertEqual(result[1].value, 10)
        self.assertEqual(result[2].value, 4)
        self.assertEqual(result[3].value, 'I am no integer')
        self.assertAlmostEqual(result[4].value, 123.0)
        self.assertEqual(self.node.get_parameter('from_value').value, 0)
        self.assertEqual(self.node.get_parameter('to_value').value, 10)
        self.assertEqual(self.node.get_parameter('in_range').value, 4)
        self.assertEqual(self.node.get_parameter('str_value').value, 'I am no integer')
        self.assertAlmostEqual(self.node.get_parameter('float_value').value, 123.0)

        # Try to set a parameter out of range.
        result = self.node.set_parameters([Parameter('in_range', value=12)])
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], SetParametersResult)
        self.assertFalse(result[0].successful)
        self.assertEqual(self.node.get_parameter('in_range').value, 4)

        # Try to set a parameter out of range (bad step).
        result = self.node.set_parameters([Parameter('in_range', value=5)])
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], SetParametersResult)
        self.assertFalse(result[0].successful)
        self.assertEqual(self.node.get_parameter('in_range').value, 4)

        # From and to are always valid.
        # Parameters that don't comply with the description will raise an exception.
        integer_range = IntegerRange(from_value=-10, to_value=0, step=30)
        parameters = [
            ('from_value_2', -10, ParameterDescriptor(integer_range=[integer_range])),
            ('to_value_2', 0, ParameterDescriptor(integer_range=[integer_range])),
            ('in_range_bad_step', -4, ParameterDescriptor(integer_range=[integer_range])),
            ('out_of_range', 30, ParameterDescriptor(integer_range=[integer_range]))
        ]
        with self.assertRaises(InvalidParameterValueException):
            self.node.declare_parameters('', parameters)

        self.assertEqual(self.node.get_parameter('from_value_2').value, -10)
        self.assertEqual(self.node.get_parameter('to_value_2').value, 0)
        self.assertFalse(self.node.has_parameter('in_range_bad_step'))
        self.assertFalse(self.node.has_parameter('out_of_range'))

        # Try some more parameters with no step.
        integer_range = IntegerRange(from_value=-10, to_value=10, step=0)
        parameters = [
            ('from_value_no_step', -10, ParameterDescriptor(integer_range=[integer_range])),
            ('to_value_no_step', 10, ParameterDescriptor(integer_range=[integer_range])),
            ('in_range_no_step', 5, ParameterDescriptor(integer_range=[integer_range])),
        ]

        result = self.node.declare_parameters('', parameters)

        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], Parameter)
        self.assertIsInstance(result[1], Parameter)
        self.assertIsInstance(result[2], Parameter)
        self.assertEqual(result[0].value, -10)
        self.assertEqual(result[1].value, 10)
        self.assertEqual(result[2].value, 5)
        self.assertEqual(self.node.get_parameter('from_value_no_step').value, -10)
        self.assertEqual(self.node.get_parameter('to_value_no_step').value, 10)
        self.assertEqual(self.node.get_parameter('in_range_no_step').value, 5)

    def test_static_dynamic_typing(self):
        parameters = [
            ('int_param', 0),
            ('int_param_no_default', Parameter.Type.INTEGER),
            ('dynamic_param', None, ParameterDescriptor(dynamic_typing=True)),
        ]
        result = self.node.declare_parameters('', parameters)

        # Try getting parameters before setting values
        int_param = self.node.get_parameter('int_param')
        self.assertEqual(int_param.type_, Parameter.Type.INTEGER)
        self.assertEqual(int_param.value, 0)
        with pytest.raises(ParameterUninitializedException):
            self.node.get_parameter('int_param_no_default')
        self.assertEqual(self.node.get_parameter('dynamic_param').type_, Parameter.Type.NOT_SET)

        result = self.node.set_parameters([Parameter('int_param', value='asd')])[0]
        self.assertFalse(result.successful)
        self.assertTrue(result.reason.startswith('Wrong parameter type'))

        self.assertTrue(self.node.set_parameters([Parameter('int_param', value=3)])[0].successful)

        result = self.node.set_parameters([Parameter('int_param_no_default', value='asd')])[0]
        self.assertFalse(result.successful)
        self.assertTrue(result.reason.startswith('Wrong parameter type'))

        self.assertTrue(
            self.node.set_parameters([Parameter('int_param_no_default', value=3)])[0].successful)
        self.assertEqual(self.node.get_parameter('int_param_no_default').value, 3)

        result = self.node.set_parameters([Parameter('int_param_no_default', value=None)])[0]
        self.assertFalse(result.successful)
        self.assertTrue(result.reason.startswith('Static parameter cannot be undeclared'))

        self.assertTrue(
            self.node.set_parameters([Parameter('dynamic_param', value='asd')])[0].successful)
        self.assertTrue(
            self.node.set_parameters([Parameter('dynamic_param', value=3)])[0].successful)

        result = self.node.set_parameters_atomically([
            Parameter('dynamic_param', value=3), Parameter('int_param', value='asd')])
        self.assertFalse(result.successful)
        self.assertTrue(result.reason.startswith('Wrong parameter type'))

        self.assertTrue(self.node.set_parameters_atomically([
            Parameter('dynamic_param', value=None), Parameter('int_param', value=4)]).successful)
        self.assertEqual(self.node.get_parameter('int_param').value, 4)
        self.assertFalse(self.node.has_parameter('dynamic_param'))


class TestCreateNode(unittest.TestCase):

    def test_use_global_arguments(self):
        context = rclpy.context.Context()
        rclpy.init(
            args=['process_name', '--ros-args', '-r', '__node:=global_node_name'],
            context=context
        )
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
                'my_node',
                namespace='/my_ns',
                cli_args=['--ros-args', '-r', '__ns:=/foo/bar'],
                context=context
            )
            self.assertEqual('/foo/bar', node.get_namespace())
            node.destroy_node()
        finally:
            rclpy.shutdown(context=context)

    def test_bad_node_arguments(self):
        context = rclpy.context.Context()
        rclpy.init(context=context)

        from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

        invalid_ros_args_error_pattern = r'Failed to parse ROS arguments:.*not-a-remap.*'
        with self.assertRaisesRegex(_rclpy.RCLInvalidROSArgsError, invalid_ros_args_error_pattern):
            rclpy.create_node(
                'my_node',
                namespace='/my_ns',
                cli_args=['--ros-args', '-r', 'not-a-remap'],
                context=context)

        unknown_ros_args_error_pattern = r'\[\'--my-custom-flag\'\]'
        with self.assertRaisesRegex(_rclpy.UnknownROSArgsError, unknown_ros_args_error_pattern):
            rclpy.create_node(
                'my_node',
                namespace='/my_ns',
                cli_args=['--ros-args', '--my-custom-flag'],
                context=context)

        rclpy.shutdown(context=context)

    def test_node_get_fully_qualified_name(self):
        context = rclpy.context.Context()
        rclpy.init(context=context)

        ns = '/my_ns'
        name = 'my_node'
        node = rclpy.create_node(name, namespace=ns, context=context)
        assert node.get_fully_qualified_name() == '{}/{}'.format(ns, name)
        node.destroy_node()

        # When ns is not specified, a leading / should be added
        node_without_ns = rclpy.create_node(name, context=context)
        assert node_without_ns.get_fully_qualified_name() == '/' + name
        node_without_ns.destroy_node()

        remapped_ns = '/another_ns'
        remapped_name = 'another_node'
        node_with_remapped_ns = rclpy.create_node(
            name,
            namespace=ns,
            context=context,
            cli_args=['--ros-args', '-r', '__ns:=' + remapped_ns]
        )
        expected_name = '{}/{}'.format(remapped_ns, name)
        assert node_with_remapped_ns.get_fully_qualified_name() == expected_name
        node_with_remapped_ns.destroy_node()

        node_with_remapped_name = rclpy.create_node(
            name,
            namespace=ns,
            context=context,
            cli_args=['--ros-args', '-r', '__node:=' + remapped_name]
        )
        expected_name = '{}/{}'.format(ns, remapped_name)
        assert node_with_remapped_name.get_fully_qualified_name() == expected_name
        node_with_remapped_name.destroy_node()

        node_with_remapped_ns_name = rclpy.create_node(
            name,
            namespace=ns,
            context=context,
            cli_args=['--ros-args', '-r', '__node:=' + remapped_name, '-r', '__ns:=' + remapped_ns]
        )
        expected_name = '{}/{}'.format(remapped_ns, remapped_name)
        assert node_with_remapped_ns_name.get_fully_qualified_name() == expected_name
        node_with_remapped_ns_name.destroy_node()

        rclpy.shutdown(context=context)

        g_context = rclpy.context.Context()
        global_remap_name = 'global_node_name'
        rclpy.init(
            args=['--ros-args', '-r', '__node:=' + global_remap_name],
            context=g_context,
        )
        node_with_global_arguments = rclpy.create_node(
            name,
            namespace=ns,
            context=g_context,
        )
        expected_name = '{}/{}'.format(ns, global_remap_name)
        assert node_with_global_arguments.get_fully_qualified_name() == expected_name
        node_with_global_arguments.destroy_node()

        node_skip_global_params = rclpy.create_node(
            name,
            namespace=ns,
            context=g_context,
            use_global_arguments=False
        )
        assert node_skip_global_params.get_fully_qualified_name() == '{}/{}'.format(ns, name)
        node_skip_global_params.destroy_node()

        rclpy.shutdown(context=g_context)


def test_node_resolve_name():
    context = rclpy.Context()
    rclpy.init(
        args=['--ros-args', '-r', 'foo:=bar'],
        context=context,
    )
    node = rclpy.create_node('test_rclpy_node_resolve_name', namespace='/my_ns', context=context)
    assert node.resolve_topic_name('foo') == '/my_ns/bar'
    assert node.resolve_topic_name('/abs') == '/abs'
    assert node.resolve_topic_name('foo', only_expand=True) == '/my_ns/foo'
    assert node.resolve_service_name('foo') == '/my_ns/bar'
    assert node.resolve_service_name('/abs') == '/abs'
    assert node.resolve_service_name('foo', only_expand=True) == '/my_ns/foo'
    rclpy.shutdown(context=context)


class TestNodeParamsFile(unittest.TestCase):

    @classmethod
    def setUp(self):
        rclpy.init()

    @classmethod
    def tearDown(self):
        rclpy.shutdown()

    def test_node_ns_params_file_with_wildcards(self):
        node = rclpy.create_node(
            'node2',
            namespace='/ns',
            cli_args=[
                '--ros-args',
                '--params-file', str(TEST_RESOURCES_DIR / 'wildcards.yaml')
            ],
            automatically_declare_parameters_from_overrides=True)
        self.assertEqual('full_wild', node.get_parameter('full_wild').value)
        self.assertEqual('namespace_wild', node.get_parameter('namespace_wild').value)
        self.assertEqual(
          'namespace_wild_another', node.get_parameter('namespace_wild_another').value)
        self.assertEqual(
          'namespace_wild_one_star', node.get_parameter('namespace_wild_one_star').value)
        self.assertEqual('node_wild_in_ns', node.get_parameter('node_wild_in_ns').value)
        self.assertEqual(
          'node_wild_in_ns_another', node.get_parameter('node_wild_in_ns_another').value)
        self.assertEqual('explicit_in_ns', node.get_parameter('explicit_in_ns').value)
        with self.assertRaises(ParameterNotDeclaredException):
            node.get_parameter('node_wild_no_ns')
        with self.assertRaises(ParameterNotDeclaredException):
            node.get_parameter('explicit_no_ns')
        with self.assertRaises(ParameterNotDeclaredException):
            node.get_parameter('should_not_appear')

    def test_node_params_file_with_wildcards(self):
        node = rclpy.create_node(
            'node2',
            cli_args=[
                '--ros-args',
                '--params-file', str(TEST_RESOURCES_DIR / 'wildcards.yaml')
            ],
            automatically_declare_parameters_from_overrides=True)
        self.assertEqual('full_wild', node.get_parameter('full_wild').value)
        self.assertEqual('namespace_wild', node.get_parameter('namespace_wild').value)
        self.assertEqual(
          'namespace_wild_another', node.get_parameter('namespace_wild_another').value)
        with self.assertRaises(ParameterNotDeclaredException):
            node.get_parameter('namespace_wild_one_star')
        with self.assertRaises(ParameterNotDeclaredException):
            node.get_parameter('node_wild_in_ns')
        with self.assertRaises(ParameterNotDeclaredException):
            node.get_parameter('node_wild_in_ns_another')
        with self.assertRaises(ParameterNotDeclaredException):
            node.get_parameter('explicit_in_ns')
        self.assertEqual('node_wild_no_ns', node.get_parameter('node_wild_no_ns').value)
        self.assertEqual('explicit_no_ns', node.get_parameter('explicit_no_ns').value)
        with self.assertRaises(ParameterNotDeclaredException):
            node.get_parameter('should_not_appear')

    def test_node_ns_params_file_by_order(self):
        node = rclpy.create_node(
            'node2',
            namespace='/ns',
            cli_args=[
                '--ros-args',
                '--params-file', str(TEST_RESOURCES_DIR / 'params_by_order.yaml')
            ],
            automatically_declare_parameters_from_overrides=True)
        self.assertEqual('last_one_win', node.get_parameter('a_value').value)
        self.assertEqual('foo', node.get_parameter('foo').value)
        self.assertEqual('bar', node.get_parameter('bar').value)

    def test_node_ns_params_file_with_complicated_wildcards(self):
        # regex matched: /**/foo/*/bar
        node = rclpy.create_node(
            'node2',
            namespace='/a/b/c/foo/d/bar',
            cli_args=[
                '--ros-args',
                '--params-file', str(TEST_RESOURCES_DIR / 'complicated_wildcards.yaml')
            ],
            automatically_declare_parameters_from_overrides=True)
        self.assertEqual('foo', node.get_parameter('foo').value)
        self.assertEqual('bar', node.get_parameter('bar').value)


if __name__ == '__main__':
    unittest.main()
