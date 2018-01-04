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

import asyncio
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor


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
        # TODO(sloretz) redesign test, sleeping to workaround race condition between test cleanup
        # and MultiThreadedExecutor thread pool
        time.sleep(0.1)

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

    def test_execute_coroutine_timer(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)

        called1 = False
        called2 = False

        async def coroutine():
            nonlocal called1
            nonlocal called2
            called1 = True
            await asyncio.sleep(0)
            called2 = True

        tmr = self.node.create_timer(0.1, coroutine)
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

    def test_execute_coroutine_guard_condition(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)

        called1 = False
        called2 = False

        async def coroutine():
            nonlocal called1
            nonlocal called2
            called1 = True
            await asyncio.sleep(0)
            called2 = True

        gc = self.node.create_guard_condition(coroutine)
        try:
            gc.trigger()
            executor.spin_once(timeout_sec=0)
            self.assertTrue(called1)
            self.assertFalse(called2)

            called1 = False
            executor.spin_once(timeout_sec=1)
            self.assertFalse(called1)
            self.assertTrue(called2)
        finally:
            self.node.destroy_guard_condition(gc)

    def test_create_task_coroutine(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)

        async def coroutine():
            return 'Sentinel Result'

        future = executor.create_task(coroutine)
        self.assertFalse(future.done())

        executor.spin_once(timeout_sec=0)
        self.assertTrue(future.done())
        self.assertEqual('Sentinel Result', future.result())

    def test_create_task_normal_function(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)

        def func():
            return 'Sentinel Result'

        future = executor.create_task(func)
        self.assertFalse(future.done())

        executor.spin_once(timeout_sec=0)
        self.assertTrue(future.done())
        self.assertEqual('Sentinel Result', future.result())

    def test_create_task_dependent_coroutines(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)

        async def coro1():
            return 'Sentinel Result 1'

        future1 = executor.create_task(coro1)

        async def coro2():
            nonlocal future1
            await future1
            return 'Sentinel Result 2'

        future2 = executor.create_task(coro2)

        # Coro2 is newest task, so it gets to await future1 in this spin
        executor.spin_once(timeout_sec=0)
        # Coro1 execs in this spin
        executor.spin_once(timeout_sec=0)
        self.assertTrue(future1.done())
        self.assertEqual('Sentinel Result 1', future1.result())
        self.assertFalse(future2.done())

        # Coro2 passes the await step here (timeout change forces new generator)
        executor.spin_once(timeout_sec=1)
        self.assertTrue(future2.done())
        self.assertEqual('Sentinel Result 2', future2.result())


if __name__ == '__main__':
    unittest.main()
