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
import os
from pathlib import Path
import time
import unittest

import rclpy
from rclpy.clock import Clock, ROSClock
from rclpy.logging import LoggingSeverity
from rclpy.time import Time
from rclpy.time_source import TimeSource


class TestLogging(unittest.TestCase):

    def test_root_logger_level(self):
        original_severity = rclpy.logging._root_logger.get_effective_level()
        for severity in LoggingSeverity:
            rclpy.logging._root_logger.set_level(severity)
            self.assertEqual(
                severity, rclpy.logging._root_logger.get_effective_level())
        rclpy.logging._root_logger.set_level(original_severity)

    def test_logger_level(self):
        # We should be able to set the threshold of a nonexistent logger / one that doesn't
        # correspond to a python object, e.g. an RMW internal logger.
        name = 'my_internal_logger_name'
        original_severity = rclpy.logging.get_logger_effective_level(name)
        for severity in LoggingSeverity:
            if severity is not LoggingSeverity.UNSET:  # unset causes the hierarchy to be traversed
                rclpy.logging.set_logger_level(name, severity)
                self.assertEqual(
                    severity, rclpy.logging.get_logger_effective_level(name))
        rclpy.logging.set_logger_level(name, original_severity)

    def test_logger_object_level(self):
        logger = rclpy.logging.get_logger('test_logger')
        for severity in LoggingSeverity:
            if severity is not LoggingSeverity.UNSET:  # unset causes the hierarchy to be traversed
                logger.set_level(severity)
                self.assertEqual(severity, logger.get_effective_level())

    def test_logger_effective_level(self):
        name = 'my_nonexistent_logger_name'
        self.assertEqual(
            rclpy.logging._root_logger.get_effective_level(),
            rclpy.logging.get_logger_effective_level(name))

        # Check that the effective threshold for a logger with manually unset severity is default
        rclpy.logging.set_logger_level(name, LoggingSeverity.UNSET)
        self.assertEqual(
            rclpy.logging._root_logger.get_effective_level(),
            rclpy.logging.get_logger_effective_level(name))
        # Check that the effective threshold for a logger with set severity
        rclpy.logging.set_logger_level(name, LoggingSeverity.ERROR)
        self.assertEqual(
            LoggingSeverity.ERROR,
            rclpy.logging.get_logger_effective_level(name))

    def test_log_threshold(self):
        rclpy.logging._root_logger.set_level(LoggingSeverity.INFO)

        # Logging below threshold not expected to be logged
        self.assertFalse(rclpy.logging._root_logger.debug('message_debug'))

        # Logging at or above threshold expected to be logged
        self.assertTrue(rclpy.logging._root_logger.info('message_info'))
        self.assertTrue(rclpy.logging._root_logger.warning('message_warn'))
        self.assertTrue(rclpy.logging._root_logger.error('message_error'))
        self.assertTrue(rclpy.logging._root_logger.fatal('message_fatal'))

    def test_log_once(self):
        message_was_logged = []
        for i in range(5):
            message_was_logged.append(rclpy.logging._root_logger.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                once=True,
            ))
        self.assertEqual(message_was_logged, [True] + [False] * 4)

        # If the argument is specified as false it shouldn't impact the logging.
        message_was_logged = []
        for i in range(5):
            message_was_logged.append(rclpy.logging._root_logger.log(
                'message2_' + inspect.stack()[0][3] + '_false_' + str(i),
                LoggingSeverity.INFO,
                once=False,
            ))
        self.assertEqual(message_was_logged, [True] * 5)

    def test_log_throttle(self):
        message_was_logged = []
        system_clock = Clock()
        for i in range(5):
            message_was_logged.append(rclpy.logging._root_logger.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                throttle_duration_sec=1,
                throttle_time_source_type=system_clock,
            ))
            time.sleep(0.4)
        self.assertEqual(
            message_was_logged, [
                True,  # t=0, not throttled
                False,  # t=0.4, throttled
                False,  # t=0.8, throttled
                True,  # t=1.2, not throttled
                False  # t=1.6, throttled
            ])

    def test_log_throttle_ros_clock(self):
        message_was_logged = []
        ros_clock = ROSClock()
        time_source = TimeSource()
        time_source.attach_clock(ros_clock)
        time_source.ros_time_is_active = True
        for i in range(5):
            message_was_logged.append(rclpy.logging._root_logger.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                throttle_duration_sec=1,
                throttle_time_source_type=ros_clock,
            ))
            time.sleep(0.4)
        self.assertEqual(
            message_was_logged, [
                False,  # t=0, throttled
                False,  # t=0.4, throttled
                False,  # t=0.8, throttled
                False,  # t=1.2, throttled
                False  # t=1.6, throttled
            ])
        message_was_logged = []
        for i in range(5):
            message_was_logged.append(rclpy.logging._root_logger.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                throttle_duration_sec=2,
                throttle_time_source_type=ros_clock,
            ))
            ros_clock.set_ros_time_override(Time(
                seconds=i + 1,
                nanoseconds=0,
                clock_type=ros_clock.clock_type,
            ))
        self.assertEqual(
            message_was_logged, [
                False,  # t=0, throttled
                False,  # t=1.0, throttled
                True,  # t=2.0, not throttled
                False,  # t=3.0, throttled
                True  # t=4.0, not throttled
            ])

    def test_log_skip_first(self):
        message_was_logged = []
        for i in range(5):
            message_was_logged.append(rclpy.logging._root_logger.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                skip_first=True,
            ))
        self.assertEqual(message_was_logged, [False] + [True] * 4)

    def test_log_skip_first_throttle(self):
        # Because of the ordering of supported_filters, first the throttle condition will be
        # evaluated/updated, then the skip_first condition
        message_was_logged = []
        system_clock = Clock()
        for i in range(5):
            message_was_logged.append(rclpy.logging._root_logger.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                skip_first=True,
                throttle_duration_sec=1,
                throttle_time_source_type=system_clock,
            ))
            time.sleep(0.4)
        self.assertEqual(
            message_was_logged, [
                False,  # t=0, not throttled, but skipped because first
                False,  # t=0.4, throttled
                False,  # t=0.8, throttled
                True,  # t=1.2, not throttled
                False  # t=1.6, throttled
            ])

    def test_log_skip_first_once(self):
        # Because of the ordering of supported_filters, first the skip_first condition will be
        # evaluated/updated, then the once condition
        message_was_logged = []
        for i in range(5):
            message_was_logged.append(rclpy.logging._root_logger.log(
                'message_' + inspect.stack()[0][3] + '_' + str(i),
                LoggingSeverity.INFO,
                once=True,
                skip_first=True,
            ))
            time.sleep(0.3)
        self.assertEqual(message_was_logged, [False, True] + [False] * 3)

    def test_log_arguments(self):
        system_clock = Clock()
        # Check half-specified filter not allowed if a required parameter is missing
        with self.assertRaisesRegex(TypeError, 'required parameter .* not specified'):
            rclpy.logging._root_logger.log(
                'message',
                LoggingSeverity.INFO,
                throttle_time_source_type=system_clock,
            )

        # Check half-specified filter is allowed if an optional parameter is missing
        rclpy.logging._root_logger.log(
            'message',
            LoggingSeverity.INFO,
            throttle_duration_sec=0.1,
        )

        # Check unused kwarg is not allowed
        with self.assertRaisesRegex(TypeError, 'parameter .* is not one of the recognized'):
            rclpy.logging._root_logger.log(
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
                rclpy.logging._root_logger.log(
                    'message_' + inspect.stack()[0][3] + '_' + str(i),
                    LoggingSeverity.INFO,
                    throttle_duration_sec=i,
                )

        with self.assertRaisesRegex(ValueError, 'name cannot be changed between'):
            for i in range(2):
                rclpy.logging._root_logger.log(
                    'message_' + inspect.stack()[0][3] + '_' + str(i),
                    LoggingSeverity.INFO,
                    name='name_' + str(i),
                )

        with self.assertRaisesRegex(ValueError, 'severity cannot be changed between'):
            for severity in LoggingSeverity:
                rclpy.logging._root_logger.log(
                    'message_' + inspect.stack()[0][3] + '_' + str(severity),
                    severity,
                )

    def test_named_logger(self):
        my_logger = rclpy.logging.get_logger('my_logger')

        my_logger.set_level(LoggingSeverity.INFO)
        # Test convenience functions

        # Logging below threshold not expected to be logged
        self.assertFalse(my_logger.debug('message_debug'))

        # Logging at or above threshold expected to be logged
        self.assertTrue(my_logger.warning('message_warn'))
        self.assertTrue(my_logger.error('message_err'))
        self.assertTrue(my_logger.fatal('message_fatal'))

        # Check that specifying a different severity isn't allowed
        with self.assertRaisesRegex(TypeError, "got multiple values for argument 'severity'"):
            my_logger.fatal(
                'message_fatal',
                severity=LoggingSeverity.DEBUG)

        # Check that this logger's context is independent of the root logger's context
        loggers = [my_logger, rclpy.logging._root_logger]
        for logger in loggers:
            message_was_logged = []
            for i in range(5):
                message_was_logged.append(logger.log(
                    'message_' + inspect.stack()[0][3] + '_' + str(i),
                    LoggingSeverity.INFO,
                    once=True,
                ))
            self.assertEqual(message_was_logged, [True] + [False] * 4)

    def test_named_logger_hierarchy(self):
        # Create a logger that implicitly is a child of the un-named root logger
        with self.assertRaisesRegex(ValueError, 'Logger name must not be empty'):
            my_logger = rclpy.logging.get_logger('')

        my_logger = rclpy.logging.get_logger('my_logger')
        self.assertEqual('my_logger', my_logger.name)

        # Check that any logger gets the level of the root logger by default
        self.assertEqual(
            rclpy.logging._root_logger.get_effective_level(),
            my_logger.get_effective_level())

        with self.assertRaisesRegex(ValueError, 'Child logger name must not be empty'):
            my_logger_child = my_logger.get_child('')

        with self.assertRaisesRegex(ValueError, 'Child logger name must not be empty'):
            my_logger_child = my_logger.get_child(None)

        my_logger_child = my_logger.get_child('child')
        self.assertEqual(my_logger.name + '.child', my_logger_child.name)

        original_severity = rclpy.logging._root_logger.get_effective_level()
        default_severity = LoggingSeverity.INFO
        rclpy.logging._root_logger.set_level(default_severity)

        # Check that children get the default severity if parent's threshold is unset
        self.assertEqual(default_severity, my_logger.get_effective_level())
        self.assertEqual(default_severity, my_logger_child.get_effective_level())

        # Check that children inherit their parent's threshold
        my_logger_severity = LoggingSeverity.ERROR
        my_logger.set_level(my_logger_severity)
        self.assertEqual(my_logger_severity, my_logger.get_effective_level())
        self.assertEqual(my_logger_severity, my_logger_child.get_effective_level())

        # Check that child's threshold has preference over their parent's, if set
        my_logger_child_severity = LoggingSeverity.DEBUG
        my_logger_child.set_level(my_logger_child_severity)
        self.assertEqual(my_logger_severity, my_logger.get_effective_level())
        self.assertEqual(
            my_logger_child_severity,
            my_logger_child.get_effective_level())

        # Check that severity inheritance returns if the child's threshold is cleared
        my_logger_child_severity = LoggingSeverity.UNSET
        my_logger_child.set_level(my_logger_child_severity)
        self.assertEqual(my_logger_severity, my_logger.get_effective_level())
        self.assertEqual(my_logger_severity, my_logger_child.get_effective_level())

        rclpy.logging._root_logger.set_level(original_severity)

    def test_clear_config(self):
        my_logger = rclpy.logging.get_logger('my_temp_logger')
        my_logger.set_level(LoggingSeverity.WARN)
        self.assertEqual(LoggingSeverity.WARN, my_logger.get_effective_level())
        rclpy.logging.clear_config()
        self.assertNotEqual(LoggingSeverity.WARN, my_logger.get_effective_level())
        self.assertEqual(
            rclpy.logging._root_logger.get_effective_level(),
            my_logger.get_effective_level())

    def test_logging_severity_from_string(self):
        for severity in rclpy.logging.LoggingSeverity:
            self.assertEqual(
                rclpy.logging.get_logging_severity_from_string(severity.name), severity)

    def test_nonexistent_logging_severity_from_string(self):
        with self.assertRaises(RuntimeError):
            rclpy.logging.get_logging_severity_from_string('non_existent_severity')

    def test_get_logging_directory(self):
        os.environ['HOME'] = '/fake_home_dir'
        os.environ.pop('USERPROFILE', None)
        os.environ.pop('ROS_LOG_DIR', None)
        os.environ.pop('ROS_HOME', None)
        log_dir = rclpy.logging.get_logging_directory()
        assert isinstance(log_dir, Path)
        assert log_dir == Path('/fake_home_dir') / '.ros' / 'log'


if __name__ == '__main__':
    unittest.main()
