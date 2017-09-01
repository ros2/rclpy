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

    def test_severity_threshold(self):
        original_severity = rclpy.logging.get_severity_threshold()
        for severity in LoggingSeverity:
            rclpy.logging.set_severity_threshold(severity)
            self.assertEqual(severity, rclpy.logging.get_severity_threshold())
        rclpy.logging.set_severity_threshold(original_severity)

    def test_log(self):
        rclpy.logging.logdebug('message')
        rclpy.logging.loginfo('message')
        rclpy.logging.logwarn('message')
        rclpy.logging.logerr('message')
        rclpy.logging.logfatal('message')

        for severity in reversed(LoggingSeverity):
            # TODO(dhood): tests that logging features work as expected
            print('\n')
            print(severity)
            print('logging once')
            rclpy.logging.log(
                'message', severity,
                name='my_name',
                once=True,
            )
            print('logging throttled')
            rclpy.logging.log(
                'message', severity,
                throttle_duration=1000,
                name='my_name',
                throttle_time_source_type='RCUTILS_STEADY_TIME',
            )
            # Check half-specified feature
            with self.assertRaisesRegex(RuntimeError, 'required parameter .* not specified'):
                rclpy.logging.log(
                    'message', severity,
                    throttle_time_source_type='asdf',
                )

            # Check unused kwarg is not allowed
            with self.assertRaisesRegex(RuntimeError, 'parameter .* is not one of the recognized'):
                rclpy.logging.log(
                    'message', severity,
                    name='my_name',
                    skip_first=True,
                    unused_kwarg='unused_kwarg',
                )

            # Check changing log call parameters is not allowed
            with self.assertRaisesRegex(ValueError, 'parameters cannot be changed between'):
                # Start at 1 because a throttle_duration of 0 causes the filter to be ignored.
                for i in range(1, 3):
                    rclpy.logging.log(
                        'message', severity,
                        throttle_duration=i,
                    )

            with self.assertRaisesRegex(ValueError, 'name cannot be changed between'):
                for i in range(2):
                    rclpy.logging.log(
                        'message', severity,
                        name=str(i),
                    )


if __name__ == '__main__':
    unittest.main()
