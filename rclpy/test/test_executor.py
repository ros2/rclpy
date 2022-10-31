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
import os
import threading
import time
import unittest
import warnings

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future


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

    def test_executor_immediate_shutdown(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor(context=self.context)
        try:
            got_callback = False

            def timer_callback():
                nonlocal got_callback
                got_callback = True

            timer_period = 1
            tmr = self.node.create_timer(timer_period, timer_callback)

            self.assertTrue(executor.add_node(self.node))
            t = threading.Thread(target=executor.spin, daemon=True)
            start_time = time.monotonic()
            t.start()
            executor.shutdown()
            t.join()
            end_time = time.monotonic()

            self.node.destroy_timer(tmr)
            self.assertLess(end_time - start_time, timer_period / 2)
            self.assertFalse(got_callback)
        finally:
            executor.shutdown()

    def test_shutdown_executor_before_waiting_for_callbacks(self):
        self.assertIsNotNone(self.node.handle)
        for cls in [SingleThreadedExecutor, MultiThreadedExecutor]:
            executor = cls(context=self.context)
            executor.shutdown()
            with self.assertRaises(ShutdownException):
                executor.wait_for_ready_callbacks()

    def test_shutdown_exception_from_callback_generator(self):
        self.assertIsNotNone(self.node.handle)
        for cls in [SingleThreadedExecutor, MultiThreadedExecutor]:
            executor = cls(context=self.context)
            cb_generator = executor._wait_for_ready_callbacks()
            executor.shutdown()
            with self.assertRaises(ShutdownException):
                next(cb_generator)

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

    def test_multi_threaded_executor_num_threads(self):
        self.assertIsNotNone(self.node.handle)

        # check default behavior, either platform configuration or defaults to 2
        executor = MultiThreadedExecutor(context=self.context)
        if hasattr(os, 'sched_getaffinity'):
            platform_threads = len(os.sched_getaffinity(0))
        else:
            platform_threads = os.cpu_count()
        self.assertEqual(platform_threads, executor._executor._max_workers)
        executor.shutdown()

        # check specified thread number w/o warning
        executor = MultiThreadedExecutor(num_threads=3, context=self.context)
        self.assertEqual(3, executor._executor._max_workers)
        executor.shutdown()

        # check specified thread number = 1, expecting UserWarning
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter('always', category=UserWarning)
            executor = MultiThreadedExecutor(num_threads=1, context=self.context)
            self.assertEqual(1, executor._executor._max_workers)
            executor.shutdown()
            assert len(w) == 1
            assert issubclass(w[0].category, UserWarning)

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
            while future is None or not future.done():
                try:
                    executor.spin_once()
                finally:
                    executor.shutdown()
                    break

        # Start spinning in a separate thread
        thr = threading.Thread(target=spin_until_task_done, args=(executor, ), daemon=True)
        thr.start()

        # Sleep in this thread to give the executor a chance to reach the loop in
        # '_wait_for_ready_callbacks()'
        time.sleep(1)

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
        assert id(executor) == id(self.node.executor)
        assert not executor.add_node(self.node)
        assert id(executor) == id(self.node.executor)

    def test_executor_spin_until_future_complete_timeout(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)

        def timer_callback():
            pass
        timer = self.node.create_timer(0.003, timer_callback)

        # Timeout
        future = Future()
        self.assertFalse(future.done())
        start = time.monotonic()
        executor.spin_until_future_complete(future=future, timeout_sec=0.1)
        end = time.monotonic()
        # Nothing is ever setting the future, so this should have waited
        # at least 0.1 seconds.
        self.assertGreaterEqual(end - start, 0.1)
        self.assertFalse(future.done())

        timer.cancel()

    def test_executor_spin_until_future_complete_future_done(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)

        def timer_callback():
            pass
        timer = self.node.create_timer(0.003, timer_callback)

        def set_future_result(future):
            future.set_result('finished')

        # Future complete timeout_sec > 0
        future = Future()
        self.assertFalse(future.done())
        t = threading.Thread(target=lambda: set_future_result(future))
        t.start()
        executor.spin_until_future_complete(future=future, timeout_sec=0.2)
        self.assertTrue(future.done())
        self.assertEqual(future.result(), 'finished')

        # Future complete timeout_sec = None
        future = Future()
        self.assertFalse(future.done())
        t = threading.Thread(target=lambda: set_future_result(future))
        t.start()
        executor.spin_until_future_complete(future=future, timeout_sec=None)
        self.assertTrue(future.done())
        self.assertEqual(future.result(), 'finished')

        # Future complete timeout < 0
        future = Future()
        self.assertFalse(future.done())
        t = threading.Thread(target=lambda: set_future_result(future))
        t.start()
        executor.spin_until_future_complete(future=future, timeout_sec=-1)
        self.assertTrue(future.done())
        self.assertEqual(future.result(), 'finished')

        timer.cancel()

    def test_executor_spin_until_future_complete_do_not_wait(self):
        self.assertIsNotNone(self.node.handle)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)

        def timer_callback():
            pass
        timer = self.node.create_timer(0.003, timer_callback)

        # Do not wait timeout_sec = 0
        future = Future()
        self.assertFalse(future.done())
        executor.spin_until_future_complete(future=future, timeout_sec=0)
        self.assertFalse(future.done())

        timer.cancel()

    def test_executor_add_node_wakes_executor(self):
        self.assertIsNotNone(self.node.handle)
        got_callback = False

        def timer_callback():
            nonlocal got_callback
            got_callback = True

        timer_period = 0.1
        tmr = self.node.create_timer(timer_period, timer_callback)

        executor = SingleThreadedExecutor(context=self.context)
        try:
            # spin in background
            t = threading.Thread(target=executor.spin_once, daemon=True)
            t.start()
            # sleep to make sure executor is blocked in rcl_wait
            time.sleep(0.5)

            self.assertTrue(executor.add_node(self.node))
            # Make sure timer has time to trigger
            time.sleep(timer_period)

            self.assertTrue(got_callback)
        finally:
            executor.shutdown()
            self.node.destroy_timer(tmr)

    def shutdown_executor_from_callback(self):
        """https://github.com/ros2/rclpy/issues/944: allow for executor shutdown from callback."""
        self.assertIsNotNone(self.node.handle)
        timer_period = 0.1
        executor = SingleThreadedExecutor(context=self.context)
        shutdown_event = threading.Event()

        def timer_callback():
            nonlocal shutdown_event, executor
            executor.shutdown()
            shutdown_event.set()

        tmr = self.node.create_timer(timer_period, timer_callback)
        executor.add_node(self.node)
        t = threading.Thread(target=executor.spin, daemon=True)
        t.start()
        self.assertTrue(shutdown_event.wait(120))
        self.node.destroy_timer(tmr)


if __name__ == '__main__':
    unittest.main()
