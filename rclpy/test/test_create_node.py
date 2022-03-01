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

from rclpy.exceptions import InvalidNamespaceException
from rclpy.exceptions import InvalidNodeNameException
from rclpy.parameter import Parameter


class TestCreateNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown(context=cls.context)

    def test_create_node(self):
        node_name = 'create_node_test'
        rclpy.create_node(node_name, context=self.context).destroy_node()

    def test_create_node_with_namespace(self):
        node_name = 'create_node_test'
        namespace = '/ns'
        rclpy.create_node(node_name, namespace=namespace, context=self.context).destroy_node()

    def test_create_node_with_empty_namespace(self):
        node_name = 'create_node_test'
        namespace = ''
        node = rclpy.create_node(node_name, namespace=namespace, context=self.context)
        self.assertEqual('/', node.get_namespace())
        node.destroy_node()

    def test_create_node_with_relative_namespace(self):
        node_name = 'create_node_test'
        namespace = 'ns'
        node = rclpy.create_node(node_name, namespace=namespace, context=self.context)
        self.assertEqual('/ns', node.get_namespace())
        node.destroy_node()

    def test_create_node_invalid_name(self):
        node_name = 'create_node_test_invalid_name?'
        with self.assertRaisesRegex(InvalidNodeNameException, 'must not contain characters'):
            rclpy.create_node(node_name, context=self.context)

    def test_create_node_invalid_relative_namespace(self):
        node_name = 'create_node_test_invalid_namespace'
        namespace = 'invalid_namespace?'
        with self.assertRaisesRegex(InvalidNamespaceException, 'must not contain characters'):
            rclpy.create_node(node_name, namespace=namespace, context=self.context)

    def test_create_node_invalid_namespace(self):
        node_name = 'create_node_test_invalid_namespace'
        namespace = '/invalid_namespace?'
        with self.assertRaisesRegex(InvalidNamespaceException, 'must not contain characters'):
            rclpy.create_node(node_name, namespace=namespace, context=self.context)

    def test_create_node_with_parameter_overrides(self):
        node_name = 'create_node_with_parameter_overrides_test'
        rclpy.create_node(
            node_name, context=self.context,
            automatically_declare_parameters_from_overrides=True,
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        ).destroy_node()


if __name__ == '__main__':
    unittest.main()
