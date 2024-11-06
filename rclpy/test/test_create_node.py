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

import rclpy

from rclpy.duration import Duration
from rclpy.exceptions import InvalidNamespaceException
from rclpy.exceptions import InvalidNodeNameException
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

class TestCreateNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown(context=cls.context)

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

    def test_create_node(self) -> None:
        node_name = 'create_node_test'
        rclpy.create_node(node_name, context=self.context).destroy_node()

    def test_create_node_with_namespace(self) -> None:
        node_name = 'create_node_test'
        namespace = '/ns'
        rclpy.create_node(node_name, namespace=namespace, context=self.context).destroy_node()

    def test_create_node_with_empty_namespace(self) -> None:
        node_name = 'create_node_test'
        namespace = ''
        node = rclpy.create_node(node_name, namespace=namespace, context=self.context)
        self.assertEqual('/', node.get_namespace())
        node.destroy_node()

    def test_create_node_with_relative_namespace(self) -> None:
        node_name = 'create_node_test'
        namespace = 'ns'
        node = rclpy.create_node(node_name, namespace=namespace, context=self.context)
        self.assertEqual('/ns', node.get_namespace())
        node.destroy_node()

    def test_create_node_invalid_name(self) -> None:
        node_name = 'create_node_test_invalid_name?'
        with self.assertRaisesRegex(InvalidNodeNameException, 'must not contain characters'):
            rclpy.create_node(node_name, context=self.context)

    def test_create_node_invalid_relative_namespace(self) -> None:
        node_name = 'create_node_test_invalid_namespace'
        namespace = 'invalid_namespace?'
        with self.assertRaisesRegex(InvalidNamespaceException, 'must not contain characters'):
            rclpy.create_node(node_name, namespace=namespace, context=self.context)

    def test_create_node_invalid_namespace(self) -> None:
        node_name = 'create_node_test_invalid_namespace'
        namespace = '/invalid_namespace?'
        with self.assertRaisesRegex(InvalidNamespaceException, 'must not contain characters'):
            rclpy.create_node(node_name, namespace=namespace, context=self.context)

    def test_create_node_with_parameter_overrides(self) -> None:
        node_name = 'create_node_with_parameter_overrides_test'
        rclpy.create_node(
            node_name, context=self.context,
            automatically_declare_parameters_from_overrides=True,
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        ).destroy_node()

    def test_create_node_disable_rosout(self):
        node_name = 'create_node_test_disable_rosout'
        namespace = '/ns'
        node = rclpy.create_node(
            node_name, namespace=namespace, context=self.context, enable_rosout=False)
        # topic /rosout publisher should not exist
        self.assertFalse(node.get_publishers_info_by_topic('/rosout'))
        node.destroy_node()

    def test_create_node_rosout_qos_profile(self):
        test_qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_ALL,
            deadline=Duration(seconds=1, nanoseconds=12345),
            lifespan=Duration(seconds=20, nanoseconds=9887665),
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            liveliness_lease_duration=Duration(seconds=5, nanoseconds=23456),
            liveliness=QoSLivelinessPolicy.MANUAL_BY_TOPIC)
        node_name = 'create_node_test_rosout_qos_profile'
        namespace = '/ns'
        node = rclpy.create_node(
            node_name, namespace=namespace, context=self.context, enable_rosout=True,
            rosout_qos_profile=test_qos_profile)
        publisher_list = node.get_publishers_info_by_topic('/rosout')
        # only test node /rosout topic publisher should exist
        self.assertEqual(1, len(publisher_list))
        self.assertEqual(node.get_name(), publisher_list[0].node_name)
        self.assertEqual(node.get_namespace(), publisher_list[0].node_namespace)
        actual_qos_profile = publisher_list[0].qos_profile
        # QoS should match except depth and history cz that are not retrieved from rmw.
        self.assert_qos_equal(test_qos_profile, actual_qos_profile, is_publisher=True)
        node.destroy_node()


if __name__ == '__main__':
    unittest.main()
