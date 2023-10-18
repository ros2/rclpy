# Copyright 2022 Open Source Robotics Foundation, Inc.
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
import rclpy.executors


class TestTimerInTask(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('TestTimerInTask', context=cls.context)
        cls.executor = rclpy.executors.SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_timer_in_task(self):
        fut = rclpy.Future()

        async def work():
            self.node.create_timer(0.1, lambda: fut.set_result(None))
            await fut

        task = self.executor.create_task(work())
        self.executor.spin_until_future_complete(task, 1)
        self.assertTrue(task.done())


if __name__ == '__main__':
    unittest.main()
