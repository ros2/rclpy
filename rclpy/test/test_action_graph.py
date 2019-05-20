# Copyright 2019 Open Source Robotics Foundation, Inc.
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
import unittest

import rclpy
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.action import get_action_client_names_and_types_by_node
from rclpy.action import get_action_names_and_types
from rclpy.action import get_action_server_names_and_types_by_node

from test_msgs.action import Fibonacci

TEST_ACTION0 = 'foo_action'
TEST_ACTION1 = 'bar_action'
TEST_NODE0 = 'foo_node'
TEST_NAMESPACE0 = '/foo_ns'
TEST_NODE1 = 'bar_node'
TEST_NAMESPACE1 = '/bar_ns'
TEST_NODE2 = 'baz_node'
TEST_NAMESPACE2 = '/baz_ns'


class TestActionGraph(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node0 = rclpy.create_node(TEST_NODE0, namespace=TEST_NAMESPACE0, context=cls.context)
        cls.node1 = rclpy.create_node(TEST_NODE1, namespace=TEST_NAMESPACE1, context=cls.context)
        cls.node2 = rclpy.create_node(TEST_NODE2, namespace=TEST_NAMESPACE2, context=cls.context)

        cls.action_client10 = ActionClient(cls.node1, Fibonacci, TEST_ACTION0)
        cls.action_server10 = ActionServer(cls.node1, Fibonacci, TEST_ACTION0, lambda: None)
        cls.action_client20 = ActionClient(cls.node2, Fibonacci, TEST_ACTION0)
        cls.action_client21 = ActionClient(cls.node2, Fibonacci, TEST_ACTION1)
        cls.action_server20 = ActionServer(cls.node2, Fibonacci, TEST_ACTION0, lambda: None)
        cls.action_server21 = ActionServer(cls.node2, Fibonacci, TEST_ACTION1, lambda: None)

        assert cls.wait_for_node(node=cls.node1, remote_node=cls.node0, timeout=2)
        assert cls.wait_for_node(node=cls.node1, remote_node=cls.node2, timeout=2)
        assert cls.wait_for_node(node=cls.node2, remote_node=cls.node0, timeout=2)
        assert cls.wait_for_node(node=cls.node2, remote_node=cls.node1, timeout=2)

    @staticmethod
    def wait_for_node(*, node, remote_node, timeout):
        remote_node_identifier = (remote_node.get_name(), remote_node.get_namespace())
        start = time.time()
        while remote_node_identifier not in node.get_node_names_and_namespaces():
            if time.time() - start > timeout:
                return False
            time.sleep(0.1)
        return True

    @classmethod
    def tearDownClass(cls):
        cls.action_client10.destroy()
        cls.action_server10.destroy()
        cls.action_client20.destroy()
        cls.action_client21.destroy()
        cls.action_server20.destroy()
        cls.action_server21.destroy()
        cls.node0.destroy_node()
        cls.node1.destroy_node()
        cls.node2.destroy_node()
        rclpy.shutdown(context=cls.context)

    def get_names_and_types(
        self,
        get_names_and_types_func,
        *args,
        expected_num_names,
        timeout=5.0
    ):
        # Since it can take some time for the ROS graph to update
        # keep trying to get names and types until we get the desired number or timeout
        start = time.monotonic()
        end = start
        while (end - start) < timeout:
            nat = get_names_and_types_func(*args)
            if len(nat) == expected_num_names:
                return nat
            end = time.monotonic()
        assert len(nat) == expected_num_names

    def test_get_action_client_names_and_types_by_node(self):
        self.get_names_and_types(
            get_action_client_names_and_types_by_node,
            self.node1,
            TEST_NODE0,
            TEST_NAMESPACE0,
            expected_num_names=0)

        names_and_types1 = self.get_names_and_types(
            get_action_client_names_and_types_by_node,
            self.node0,
            TEST_NODE1,
            TEST_NAMESPACE1,
            expected_num_names=1)
        assert isinstance(names_and_types1[0], tuple)
        name1, types1 = names_and_types1[0]
        assert name1 == TEST_NAMESPACE1 + '/' + TEST_ACTION0
        assert isinstance(types1, list)
        assert len(types1) == 1
        assert types1[0] == 'test_msgs/action/Fibonacci'

        names_and_types2 = self.get_names_and_types(
            get_action_client_names_and_types_by_node,
            self.node0,
            TEST_NODE2,
            TEST_NAMESPACE2,
            expected_num_names=2)
        assert isinstance(names_and_types2[0], tuple)
        name20, types20 = names_and_types2[0]
        assert isinstance(types20, list)
        assert len(types20) == 1
        assert types20[0] == 'test_msgs/action/Fibonacci'
        assert isinstance(names_and_types2[1], tuple)
        name21, types21 = names_and_types2[1]
        assert isinstance(types21, list)
        assert len(types21) == 1
        assert types21[0] == 'test_msgs/action/Fibonacci'
        # Not assuming the order of names in the list
        assert TEST_NAMESPACE2 + '/' + TEST_ACTION0 in [name20, name21]
        assert TEST_NAMESPACE2 + '/' + TEST_ACTION1 in [name20, name21]

    def test_get_action_server_names_and_types_by_node(self):
        self.get_names_and_types(
            get_action_server_names_and_types_by_node,
            self.node1,
            TEST_NODE0,
            TEST_NAMESPACE0,
            expected_num_names=0)

        names_and_types1 = self.get_names_and_types(
            get_action_server_names_and_types_by_node,
            self.node0,
            TEST_NODE1,
            TEST_NAMESPACE1,
            expected_num_names=1)
        assert isinstance(names_and_types1[0], tuple)
        name1, types1 = names_and_types1[0]
        assert name1 == TEST_NAMESPACE1 + '/' + TEST_ACTION0
        assert isinstance(types1, list)
        assert len(types1) == 1
        assert types1[0] == 'test_msgs/action/Fibonacci'

        names_and_types2 = self.get_names_and_types(
            get_action_server_names_and_types_by_node,
            self.node0,
            TEST_NODE2,
            TEST_NAMESPACE2,
            expected_num_names=2)
        assert len(names_and_types2) == 2
        assert isinstance(names_and_types2[0], tuple)
        name20, types20 = names_and_types2[0]
        assert isinstance(types20, list)
        assert len(types20) == 1
        assert types20[0] == 'test_msgs/action/Fibonacci'
        assert isinstance(names_and_types2[1], tuple)
        name21, types21 = names_and_types2[1]
        assert isinstance(types21, list)
        assert len(types21) == 1
        assert types21[0] == 'test_msgs/action/Fibonacci'
        # Not assuming the order of names in the list
        assert TEST_NAMESPACE2 + '/' + TEST_ACTION0 in [name20, name21]
        assert TEST_NAMESPACE2 + '/' + TEST_ACTION1 in [name20, name21]

    def test_get_action_names_and_types(self):
        names_and_types = self.get_names_and_types(
            get_action_names_and_types,
            self.node0,
            expected_num_names=3)
        assert isinstance(names_and_types[0], tuple)
        name0, types0 = names_and_types[0]
        assert isinstance(types0, list)
        assert len(types0) == 1
        assert types0[0] == 'test_msgs/action/Fibonacci'
        assert isinstance(names_and_types[1], tuple)
        name1, types1 = names_and_types[1]
        assert isinstance(types1, list)
        assert len(types1) == 1
        assert types1[0] == 'test_msgs/action/Fibonacci'
        assert isinstance(names_and_types[2], tuple)
        name2, types2 = names_and_types[2]
        assert isinstance(types2, list)
        assert len(types2) == 1
        assert types2[0] == 'test_msgs/action/Fibonacci'
        # Not assuming the order of names in the list
        assert TEST_NAMESPACE1 + '/' + TEST_ACTION0 in [name0, name1, name2]
        assert TEST_NAMESPACE2 + '/' + TEST_ACTION0 in [name0, name1, name2]
        assert TEST_NAMESPACE2 + '/' + TEST_ACTION1 in [name0, name1, name2]


if __name__ == '__main__':
    unittest.main()
