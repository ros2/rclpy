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
from rclpy.executors import SingleThreadedExecutor


class TestGuardCondition(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node(
            'TestGuardCondition', namespace='/rclpy/test', context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_trigger(self):
        called = False

        def func():
            nonlocal called
            called = True

        gc = self.node.create_guard_condition(func)

        self.executor.spin_once(timeout_sec=0)
        self.assertFalse(called)

        gc.trigger()
        self.executor.spin_once(timeout_sec=0)
        self.assertTrue(called)

        self.node.destroy_guard_condition(gc)

    def test_double_trigger(self):
        called1 = False
        called2 = False

        def func1():
            nonlocal called1
            called1 = True

        def func2():
            nonlocal called2
            called2 = True

        gc1 = self.node.create_guard_condition(func1)
        gc2 = self.node.create_guard_condition(func2)

        self.executor.spin_once(timeout_sec=0)
        self.assertFalse(called1)
        self.assertFalse(called2)

        gc1.trigger()
        gc2.trigger()
        self.executor.spin_once(timeout_sec=0)
        self.executor.spin_once(timeout_sec=0)
        self.assertTrue(called1)
        self.assertTrue(called2)

        called1 = False
        called2 = False
        self.executor.spin_once(timeout_sec=0)
        self.assertFalse(called1)
        self.assertFalse(called2)

        self.node.destroy_guard_condition(gc1)
        self.node.destroy_guard_condition(gc2)


if __name__ == '__main__':
    unittest.main()
