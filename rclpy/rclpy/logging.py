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


from enum import IntEnum

from rclpy.impl.implementation_singleton import rclpy_logging_implementation as _rclpy_logging
import rclpy.impl.rcutils_logger


class LoggingSeverity(IntEnum):
    """
    Enum for logging severity levels.

    This enum must match the one defined in rcutils/logging.h
    """

    UNSET = 0
    DEBUG = 10
    INFO = 20
    WARN = 30
    ERROR = 40
    FATAL = 50


root_logger = rclpy.impl.rcutils_logger.RcutilsLogger('ros.rclpy')


def get_named_logger(name):
    return root_logger.get_child(name)


def get_default_severity_threshold():
    return _rclpy_logging.rclpy_logging_get_default_severity_threshold()


def set_default_severity_threshold(severity):
    severity = LoggingSeverity(severity)
    return _rclpy_logging.rclpy_logging_set_default_severity_threshold(severity)


def get_logger_severity_threshold(name):
    severity = _rclpy_logging.rclpy_logging_get_logger_severity_threshold(name)
    return LoggingSeverity(severity)


def set_logger_severity_threshold(name, severity):
    severity = LoggingSeverity(severity)
    return _rclpy_logging.rclpy_logging_set_logger_severity_threshold(name, severity)


def get_logger_effective_severity_threshold(name):
    severity = _rclpy_logging.rclpy_logging_get_logger_effective_severity_threshold(name)
    return LoggingSeverity(severity)


def logdebug(message, **kwargs):
    """Log a message with `DEBUG` severity via :py:classmethod:RcutilsLogger.log:."""
    return root_logger.log(message, severity=LoggingSeverity.DEBUG, **kwargs)


def loginfo(message, **kwargs):
    """Log a message with `INFO` severity via :py:classmethod:RcutilsLogger.log:."""
    return root_logger.log(message, severity=LoggingSeverity.INFO, **kwargs)


def logwarn(message, **kwargs):
    """Log a message with `WARN` severity via :py:classmethod:RcutilsLogger.log:."""
    return root_logger.log(message, severity=LoggingSeverity.WARN, **kwargs)


def logerr(message, **kwargs):
    """Log a message with `ERROR` severity via :py:classmethod:RcutilsLogger.log:."""
    return root_logger.log(message, severity=LoggingSeverity.ERROR, **kwargs)


def logfatal(message, **kwargs):
    """Log a message with `FATAL` severity via :py:classmethod:RcutilsLogger.log:."""
    return root_logger.log(message, severity=LoggingSeverity.FATAL, **kwargs)


def log(message, severity, **kwargs):
    """Log a message with the specified severity via :py:classmethod:RcutilsLogger.log:."""
    severity = LoggingSeverity(severity)
    return root_logger.log(message, severity, **kwargs)
