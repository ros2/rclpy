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
import inspect

import rclpy.impl.logging_rcutils
from rclpy.impl.logging_rcutils import _frame_to_caller_id


class LoggingSeverity(IntEnum):
    """
    Enum for logging severity levels.

    This enum matches the one defined in rcutils/logging.h
    """

    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3
    FATAL = 4


def get_named_logger(name):
    return rclpy.impl.logging_rcutils.RcutilsLogger(name)

_rclpy_logger = get_named_logger('rclpy.internal')


def get_severity_threshold():
    return LoggingSeverity(_rclpy_logger.get_severity_threshold())


def set_severity_threshold(severity):
    assert isinstance(severity, LoggingSeverity) or isinstance(severity, int)
    return _rclpy_logger.set_severity_threshold(severity)


def logdebug(message, **kwargs):
    _rclpy_logger.log(
        message, severity=LoggingSeverity.DEBUG,
        caller_id=_frame_to_caller_id(inspect.currentframe().f_back.f_back),
        **kwargs)


def loginfo(message, **kwargs):
    _rclpy_logger.log(
        message, severity=LoggingSeverity.INFO,
        caller_id=_frame_to_caller_id(inspect.currentframe().f_back.f_back.f_back),
        **kwargs)


def logwarn(message, **kwargs):
    _rclpy_logger.log(
        message, severity=LoggingSeverity.WARN,
        caller_id=_frame_to_caller_id(inspect.currentframe().f_back.f_back),
        **kwargs)


def logerr(message, **kwargs):
    _rclpy_logger.log(
        message, severity=LoggingSeverity.ERROR,
        caller_id=_frame_to_caller_id(inspect.currentframe().f_back.f_back),
        **kwargs)


def logfatal(message, **kwargs):
    _rclpy_logger.log(
        message, severity=LoggingSeverity.FATAL,
        caller_id=_frame_to_caller_id(inspect.currentframe().f_back.f_back),
        **kwargs)


# TODO(dhood): document the supported features
def log(message, severity, **kwargs):
    assert isinstance(severity, LoggingSeverity) or isinstance(severity, int)
    severity = LoggingSeverity(severity)
    _rclpy_logger.log(
        message, severity,
        **kwargs)
