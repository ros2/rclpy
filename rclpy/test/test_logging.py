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

import inspect
import time
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

    def test_log_threshold(self):
        rclpy.logging.set_severity_threshold(LoggingSeverity.INFO)

        # Logging below threshold not expected to be logged
        self.assertFalse(rclpy.logging.logdebug('message_debug'))

        # Logging at or above threshold expected to be logged
        self.assertTrue(rclpy.logging.loginfo('message_info'))
        self.assertTrue(rclpy.logging.logwarn('message_warn'))
        self.assertTrue(rclpy.logging.logerr('message_err'))
        self.assertTrue(rclpy.logging.logfatal('message_fatal'))

    def test_log_once(self):
        message_was_logged = []
        for i in range(5):
            message_was_logged.append(rclpy.logging.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                once=True,
            ))
        self.assertEqual(message_was_logged, [True] + [False] * 4)

        # If the argument is specified as false it shouldn't impact the logging.
        message_was_logged = []
        for i in range(5):
            message_was_logged.append(rclpy.logging.log(
                'message2_' + inspect.stack()[0][3] + '_false_' + str(i),
                LoggingSeverity.INFO,
                once=False,
            ))
        self.assertEqual(message_was_logged, [True] * 5)

    def test_log_throttle(self):
        message_was_logged = []
        for i in range(5):
            message_was_logged.append(rclpy.logging.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                throttle_duration_sec=1,
                throttle_time_source_type='RCUTILS_STEADY_TIME',
            ))
            time.sleep(0.3)
        self.assertEqual(message_was_logged, [True] + [False] * 3 + [True])

    def test_log_skip_first(self):
        message_was_logged = []
        for i in range(5):
            message_was_logged.append(rclpy.logging.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                skip_first=True,
            ))
        self.assertEqual(message_was_logged, [False] + [True] * 4)

    def test_log_skip_first_throttle(self):
        # Because of the ordering of supported_filters, first the throttle condition will be
        # evaluated/updated, then the skip_first condition
        message_was_logged = []
        for i in range(5):
            message_was_logged.append(rclpy.logging.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                skip_first=True,
                throttle_duration_sec=1,
                throttle_time_source_type='RCUTILS_STEADY_TIME',
            ))
            time.sleep(0.3)
        self.assertEqual(message_was_logged, [False] * 4 + [True])

    def test_log_skip_first_once(self):
        # Because of the ordering of supported_filters, first the skip_first condition will be
        # evaluated/updated, then the once condition
        message_was_logged = []
        for i in range(5):
            message_was_logged.append(rclpy.logging.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                once=True,
                skip_first=True,
            ))
            time.sleep(0.3)
        self.assertEqual(message_was_logged, [False, True] + [False] * 3)

    def test_log_arguments(self):
        # Check half-specified filter not allowed if a required parameter is missing
        with self.assertRaisesRegex(TypeError, 'required parameter .* not specified'):
            rclpy.logging.log(
                'message',
                LoggingSeverity.INFO,
                throttle_time_source_type='RCUTILS_STEADY_TIME',
            )

        # Check half-specified filter is allowed if an optional parameter is missing
        rclpy.logging.log(
            'message',
            LoggingSeverity.INFO,
            throttle_duration_sec=0.1,
        )

        # Check unused kwarg is not allowed
        with self.assertRaisesRegex(TypeError, 'parameter .* is not one of the recognized'):
            rclpy.logging.log(
                'message',
                LoggingSeverity.INFO,
                name='my_name',
                skip_first=True,
                unused_kwarg='unused_kwarg',
            )

    def test_log_parameters_changing(self):
        # Check changing log call parameters is not allowed
        with self.assertRaisesRegex(ValueError, 'parameters cannot be changed between'):
            # Start at 1 because a throttle_duration_sec of 0 causes the filter to be ignored.
            for i in range(1, 3):
                rclpy.logging.log(
                    'message_' + inspect.stack()[0][3] + '_' + str(i),
                    LoggingSeverity.INFO,
                    throttle_duration_sec=i,
                )

        with self.assertRaisesRegex(ValueError, 'name cannot be changed between'):
            for i in range(2):
                rclpy.logging.log(
                    'message_' + inspect.stack()[0][3] + '_' + str(i),
                    LoggingSeverity.INFO,
                    name='name_' + str(i),
                )

        with self.assertRaisesRegex(ValueError, 'severity cannot be changed between'):
            for severity in LoggingSeverity:
                rclpy.logging.log(
                    'message_' + inspect.stack()[0][3] + '_' + str(severity),
                    severity,
                )

    def test_named_logger(self):
        my_logger = rclpy.logging.get_named_logger('my_logger')

        rclpy.logging.set_severity_threshold(LoggingSeverity.INFO)
        # Test convenience functions

        # Logging below threshold not expected to be logged
        self.assertFalse(my_logger.debug('message_debug'))

        # Logging at or above threshold expected to be logged
        self.assertTrue(my_logger.warn('message_warn'))
        self.assertTrue(my_logger.error('message_err'))
        self.assertTrue(my_logger.fatal('message_fatal'))

        # Check that specifying a different severity isn't allowed
        with self.assertRaisesRegex(TypeError, "got multiple values for argument 'severity'"):
            my_logger.fatal(
                'message_fatal',
                severity=LoggingSeverity.DEBUG)

        # Check that this logger's context is independent of the root logger's context
        loggers = [my_logger, rclpy.logging.root_logger]
        for logger in loggers:
            message_was_logged = []
            for i in range(5):
                message_was_logged.append(logger.log(
                    'message_' + inspect.stack()[0][3] + '_' + str(i),
                    LoggingSeverity.INFO,
                    once=True,
                ))
            self.assertEqual(message_was_logged, [True] + [False] * 4)


if __name__ == '__main__':
    unittest.main()
