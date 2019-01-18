# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from test_msgs.action import Fibonacci
import rclpy
from rclpy.action_client import ActionClient


# TODO(jacobperron) Reduce fudge once wait_for_service uses node graph events
TIME_FUDGE = 0.3


class TestActionClient(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('TestActionClient', context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_constructor_defaults(self):
        # Defaults
        ac = ActionClient(self.node, Fibonacci, 'fibonacci')
        ac.destroy()

    def test_constructor_no_defaults(self):
        ac = ActionClient(
            self.node,
            Fibonacci,
            'fibonacci',
            goal_service_qos_profile=rclpy.qos.qos_profile_default,
            result_service_qos_profile=rclpy.qos.qos_profile_default,
            cancel_service_qos_profile=rclpy.qos.qos_profile_default,
            feedback_sub_qos_profile=rclpy.qos.qos_profile_default,
            status_sub_qos_profile=rclpy.qos.qos_profile_default
        )
        ac.destroy()

    def test_wait_for_server_nowait(self):
        ac = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            start = time.monotonic()
            self.assertFalse(ac.wait_for_server(timeout_sec=0.0))
            end = time.monotonic()
            self.assertGreater(0.0, end - start - TIME_FUDGE)
            self.assertLess(0.0, end - start + TIME_FUDGE)
        finally:
            self.node.destroy_client(ac)

    def test_wait_for_server_timeout(self):
        ac = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            start = time.monotonic()
            self.assertFalse(ac.wait_for_server(timeout_sec=5.0))
            end = time.monotonic()
            self.assertGreater(5.0, end - start - TIME_FUDGE)
            self.assertLess(5.0, end - start + TIME_FUDGE)
        finally:
            self.node.destroy_client(ac)


if __name__ == '__main__':
    unittest.main()
