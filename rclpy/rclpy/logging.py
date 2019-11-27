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

    UNSET = _rclpy_logging.rclpy_get_unset_logging_severity()
    DEBUG = _rclpy_logging.rclpy_get_debug_logging_severity()
    INFO = _rclpy_logging.rclpy_get_info_logging_severity()
    WARN = _rclpy_logging.rclpy_get_warn_logging_severity()
    ERROR = _rclpy_logging.rclpy_get_error_logging_severity()
    FATAL = _rclpy_logging.rclpy_get_fatal_logging_severity()


_root_logger = rclpy.impl.rcutils_logger.RcutilsLogger()


def get_logger(name):
    if not name:
        raise ValueError('Logger name must not be empty.')
    return _root_logger.get_child(name)


def initialize():
    return _rclpy_logging.rclpy_logging_initialize()


def shutdown():
    return _rclpy_logging.rclpy_logging_shutdown()


def clear_config():
    """Clear the configuration of the logging system, e.g. logger levels."""
    shutdown()
    initialize()


def set_logger_level(name, level):
    level = LoggingSeverity(level)
    return _rclpy_logging.rclpy_logging_set_logger_level(name, level)


def get_logger_effective_level(name):
    logger_level = _rclpy_logging.rclpy_logging_get_logger_effective_level(name)
    return LoggingSeverity(logger_level)


def get_logging_severity_from_string(log_severity):
    return LoggingSeverity(
        _rclpy_logging.rclpy_logging_severity_level_from_string(log_severity))
