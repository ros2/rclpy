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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from test_msgs.msg import Primitives


class TestCallbackGroup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('TestCallbackGroup', namespace='/rclpy', context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_reentrant_group(self):
        self.assertIsNotNone(self.node.handle)
        group = ReentrantCallbackGroup()
        t1 = self.node.create_timer(1.0, lambda: None, callback_group=group)
        t2 = self.node.create_timer(1.0, lambda: None, callback_group=group)

        self.assertTrue(group.can_execute(t1))
        self.assertTrue(group.can_execute(t2))
        self.assertTrue(group.beginning_execution(t1))
        self.assertTrue(group.beginning_execution(t2))

    def test_mutually_exclusive_group(self):
        self.assertIsNotNone(self.node.handle)
        group = MutuallyExclusiveCallbackGroup()
        t1 = self.node.create_timer(1.0, lambda: None, callback_group=group)
        t2 = self.node.create_timer(1.0, lambda: None, callback_group=group)

        self.assertTrue(group.can_execute(t1))
        self.assertTrue(group.can_execute(t2))

        self.assertTrue(group.beginning_execution(t1))
        self.assertFalse(group.can_execute(t2))
        self.assertFalse(group.beginning_execution(t2))

        group.ending_execution(t1)
        self.assertTrue(group.can_execute(t2))
        self.assertTrue(group.beginning_execution(t2))

    def test_create_timer_with_group(self):
        tmr1 = self.node.create_timer(1.0, lambda: None)
        group = ReentrantCallbackGroup()
        tmr2 = self.node.create_timer(1.0, lambda: None, callback_group=group)

        self.assertFalse(group.has_entity(tmr1))
        self.assertTrue(group.has_entity(tmr2))

    def test_create_subscription_with_group(self):
        sub1 = self.node.create_subscription(Primitives, 'chatter', lambda msg: print(msg))
        group = ReentrantCallbackGroup()
        sub2 = self.node.create_subscription(
            Primitives, 'chatter', lambda msg: print(msg), callback_group=group)

        self.assertFalse(group.has_entity(sub1))
        self.assertTrue(group.has_entity(sub2))

    def test_create_client_with_group(self):
        cli1 = self.node.create_client(GetParameters, 'get/parameters')
        group = ReentrantCallbackGroup()
        cli2 = self.node.create_client(GetParameters, 'get/parameters', callback_group=group)

        self.assertFalse(group.has_entity(cli1))
        self.assertTrue(group.has_entity(cli2))

    def test_create_service_with_group(self):
        srv1 = self.node.create_service(GetParameters, 'get/parameters', lambda req: None)
        group = ReentrantCallbackGroup()
        srv2 = self.node.create_service(
            GetParameters, 'get/parameters', lambda req: None, callback_group=group)

        self.assertFalse(group.has_entity(srv1))
        self.assertTrue(group.has_entity(srv2))


if __name__ == '__main__':
    unittest.main()
