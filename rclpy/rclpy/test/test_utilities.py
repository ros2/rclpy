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

from rclpy.constants import S_TO_NS
import rclpy.utilities


class TestValidateRemoveRosArgs(unittest.TestCase):

    def test_remove_ros_args(self):
        args = [
            'process_name',
            '-d',
            '--ros-args',
            '-r', '__ns:=/foo/bar',
            '-r', '__ns:=/fiz/buz',
            '--',
            '--foo=bar',
            '--baz'
        ]
        stripped_args = rclpy.utilities.remove_ros_args(args=args)
        self.assertEqual(4, len(stripped_args))
        self.assertEqual('process_name', stripped_args[0])
        self.assertEqual('-d', stripped_args[1])
        self.assertEqual('--foo=bar', stripped_args[2])
        self.assertEqual('--baz', stripped_args[3])


class TestUtilities(unittest.TestCase):

    def test_timeout_sec_to_nsec(self):
        self.assertGreater(0, rclpy.utilities.timeout_sec_to_nsec(None))
        self.assertGreater(0, rclpy.utilities.timeout_sec_to_nsec(-1))
        self.assertEqual(0, rclpy.utilities.timeout_sec_to_nsec(0))
        self.assertEqual(0, rclpy.utilities.timeout_sec_to_nsec(0.5 / S_TO_NS))
        self.assertEqual(int(1.5 * S_TO_NS), rclpy.utilities.timeout_sec_to_nsec(1.5))


if __name__ == '__main__':
    unittest.main()
