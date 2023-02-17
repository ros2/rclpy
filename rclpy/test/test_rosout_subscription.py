# Copyright 2023 Sony Group Corporation.
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

from rcl_interfaces.msg import Log
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future


class TestRosoutSubscription(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('test_rosout_subscription', context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def setUp(self):
        # create subscriber of 'rosout' topic
        self.sub = self.node.create_subscription(
            Log,
            '/rosout',
            self._rosout_subscription_callback,
            1
        )
        self.fut = Future()
        self.rosout_msg_name = None

    def _rosout_subscription_callback(self, msg):
        if msg.name == self.rosout_msg_name:
            self.fut.set_result(None)

    def test_parent_log(self):
        self.rosout_msg_name = 'test_rosout_subscription'
        logger = self.node.get_logger()
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

    def test_child_log(self):
        self.rosout_msg_name = 'test_rosout_subscription.child1'
        logger = self.node.get_logger()
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertFalse(self.fut.done())

        logger = self.node.get_logger().get_child('child1')
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())
        self.fut = Future()

        logger = self.node.get_logger().get_child('child2')
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertFalse(self.fut.done())

        self.rosout_msg_name = 'test_rosout_subscription.child2'
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

    def test_child_hierarchy(self):
        self.rosout_msg_name = 'test_rosout_subscription.child.grandchild'
        logger = self.node.get_logger().get_child('child').get_child('grandchild')
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

    def test_first_child_removed(self):
        self.rosout_msg_name = 'test_rosout_subscription.child'
        logger = self.node.get_logger().get_child('child')
        logger2 = self.node.get_logger().get_child('child')
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())
        logger = None
        logger2.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

    def test_logger_parameter(self):
        self.rosout_msg_name = 'test_rosout_subscription.child'
        logger = self.node.get_logger().get_child('child')

        def call_logger(logger):
            logger1 = logger
            logger1.info('test')
        call_logger(logger)
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

    def test_node_logger_not_exist(self):
        node = rclpy.create_node('test_extra_node', context=self.context)
        logger = node.get_logger()
        node = None
        with self.assertRaises(RuntimeError):
            logger.get_child('child')


if __name__ == '__main__':
    unittest.main()
