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

import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor


class AwaitableClass:

    def __await__(self):
        yield


class TestExecutor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('TestExecutor', namespace='/rclpy')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def func_execution(self, executor):
        got_callback = False

        def timer_callback():
            nonlocal got_callback
            got_callback = True

        tmr = self.node.create_timer(0.1, timer_callback)

        executor.add_node(self.node)
        executor.spin_once(timeout_sec=1.23)

        self.node.destroy_timer(tmr)
        return got_callback

    def test_single_threaded_executor_executes(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor()
        try:
            self.assertTrue(self.func_execution(executor))
        finally:
            executor.shutdown()

    def test_multi_threaded_executor_executes(self):
        self.assertIsNotNone(self.node.handle)
        executor = MultiThreadedExecutor()
        try:
            self.assertTrue(self.func_execution(executor))
        finally:
            executor.shutdown()

    def test_add_node_to_executor(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        self.assertIn(self.node, executor.get_nodes())

    def test_executor_spin_non_blocking(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        start = time.monotonic()
        executor.spin_once(timeout_sec=0)
        end = time.monotonic()
        self.assertLess(start - end, 0.001)

    def test_executor_coroutine(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)

        called1 = False
        called2 = False

        async def timer_coroutine():
            nonlocal called1
            nonlocal called2
            called1 = True
            await AwaitableClass()
            called2 = True

        tmr = self.node.create_timer(0.1, timer_coroutine)
        try:
            executor.spin_once(timeout_sec=1.23)
            self.assertTrue(called1)
            self.assertFalse(called2)

            called1 = False
            executor.spin_once(timeout_sec=0)
            self.assertFalse(called1)
            self.assertTrue(called2)
        finally:
            self.node.destroy_timer(tmr)


if __name__ == '__main__':
    unittest.main()
