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
import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor


class TestExecutor(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('TestExecutor', namespace='/rclpy', context=self.context)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def func_execution(self, executor):
        got_callback = False

        def timer_callback():
            nonlocal got_callback
            got_callback = True

        tmr = self.node.create_timer(0.1, timer_callback)

        assert executor.add_node(self.node)
        executor.spin_once(timeout_sec=1.23)
        # TODO(sloretz) redesign test, sleeping to workaround race condition between test cleanup
        # and MultiThreadedExecutor thread pool
        time.sleep(0.1)

        self.node.destroy_timer(tmr)
        return got_callback

    def test_single_threaded_executor_executes(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor(context=self.context)
        try:
            self.assertTrue(self.func_execution(executor))
        finally:
            executor.shutdown()

    def test_remove_node(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor(context=self.context)

        got_callback = False

        def timer_callback():
            nonlocal got_callback
            got_callback = True

        try:
            tmr = self.node.create_timer(0.1, timer_callback)
            try:
                executor.add_node(self.node)
                executor.remove_node(self.node)
                executor.spin_once(timeout_sec=0.2)
            finally:
                self.node.destroy_timer(tmr)
        finally:
            executor.shutdown()

        assert not got_callback

    def test_multi_threaded_executor_executes(self):
        self.assertIsNotNone(self.node.handle)
        executor = MultiThreadedExecutor(context=self.context)
        try:
            self.assertTrue(self.func_execution(executor))
        finally:
            executor.shutdown()

    def test_add_node_to_executor(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)
        self.assertIn(self.node, executor.get_nodes())

    def test_executor_spin_non_blocking(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)
        start = time.monotonic()
        executor.spin_once(timeout_sec=0)
        end = time.monotonic()
        self.assertLess(start - end, 0.001)

    def test_execute_coroutine_timer(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor(context=self.context)
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
        executor = SingleThreadedExecutor(context=self.context)
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
        executor = SingleThreadedExecutor(context=self.context)
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
        executor = SingleThreadedExecutor(context=self.context)
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
        executor = SingleThreadedExecutor(context=self.context)
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

    def test_create_task_during_spin(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)

        future = None

        def spin_until_task_done(executor):
            nonlocal future
            while future is None or future.done():
                try:
                    executor.spin_once()
                finally:
                    executor.shutdown()
                    break

        # Start spinning in a separate thread
        thr = threading.Thread(target=spin_until_task_done, args=(executor, ), daemon=True)
        thr.start()

        def func():
            return 'Sentinel Result'

        # Create a task
        future = executor.create_task(func)

        thr.join(timeout=0.5)
        # If the join timed out, remove the node to cause the spin thread to stop
        if thr.is_alive():
            executor.remove_node(self.node)

        self.assertTrue(future.done())
        self.assertEqual('Sentinel Result', future.result())

    def test_global_executor_completes_async_task(self):
        self.assertIsNotNone(self.node.handle)

        class TriggerAwait:

            def __init__(self):
                self.do_yield = True

            def __await__(self):
                while self.do_yield:
                    yield
                return

        trigger = TriggerAwait()
        did_callback = False
        did_return = False

        async def timer_callback():
            nonlocal trigger, did_callback, did_return
            did_callback = True
            await trigger
            did_return = True

        timer = self.node.create_timer(0.1, timer_callback)

        executor = SingleThreadedExecutor(context=self.context)
        rclpy.spin_once(self.node, timeout_sec=0.5, executor=executor)
        self.assertTrue(did_callback)

        timer.cancel()
        trigger.do_yield = False
        rclpy.spin_once(self.node, timeout_sec=0, executor=executor)
        self.assertTrue(did_return)

    def test_executor_add_node(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor(context=self.context)
        assert executor.add_node(self.node)
        assert not executor.add_node(self.node)


if __name__ == '__main__':
    unittest.main()
