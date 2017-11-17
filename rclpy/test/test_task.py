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

from rclpy.future import Task


class AwaitableClass:

    def __await__(self):
        yield


class TestTask(unittest.TestCase):

    def test_task_normal_callable(self):
        called = False

        def func():
            nonlocal called
            called = True

        t = Task(func)
        t()
        self.assertTrue(called)
        self.assertTrue(t.done())

    def test_task_lambda(self):
        called = False

        def func():
            nonlocal called
            called = True

        t = Task(lambda: func())
        t()
        self.assertTrue(called)
        self.assertTrue(t.done())

    def test_coroutine(self):
        called1 = False
        called2 = False

        async def coro():
            nonlocal called1
            nonlocal called2
            called1 = True
            await AwaitableClass()
            called2 = True

        t = Task(coro)
        t()
        self.assertTrue(called1)
        self.assertFalse(called2)

        called1 = False
        t()
        self.assertFalse(called1)
        self.assertTrue(called2)

    def test_done_callback(self):
        called = False

        def func():
            nonlocal called
            called = True

        t = Task(lambda: None)
        t.add_done_callback(func)
        t()
        self.assertTrue(called)
        self.assertTrue(t.done())

    def test_done_callback_is_coroutine(self):
        called1 = False
        called2 = False

        async def coro():
            nonlocal called1
            nonlocal called2
            called1 = True
            await AwaitableClass()
            called2 = True

        t = Task(lambda: None)
        t.add_done_callback(coro)
        t()
        self.assertTrue(called1)
        self.assertFalse(called2)

        called1 = False
        t()
        self.assertFalse(called1)
        self.assertTrue(called2)


if __name__ == '__main__':
    unittest.main()
