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
        rclpy.init()
        cls.node = rclpy.create_node('TestGuardCondition', namespace='/rclpy/test')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

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


if __name__ == '__main__':
    unittest.main()
