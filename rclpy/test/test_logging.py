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
from rclpy.logging import LoggingSeverity


class TestLogging(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.logging.initialize()

    def test_set_get_severity_threshold(self):
        original_severity = rclpy.logging.get_severity_threshold()
        for severity in LoggingSeverity:
            rclpy.logging.set_severity_threshold(severity)
            self.assertEqual(severity, rclpy.logging.get_severity_threshold())
        rclpy.logging.set_severity_threshold(original_severity)


if __name__ == '__main__':
    unittest.main()
