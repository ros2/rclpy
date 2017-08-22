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

import rclpy.impl.logging_rcutils


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


def initialize():
    return rclpy.impl.logging_rcutils.initialize()


def get_severity_threshold():
    return LoggingSeverity(rclpy.impl.logging_rcutils.get_severity_threshold())


def set_severity_threshold(severity):
    assert isinstance(severity, LoggingSeverity) or isinstance(severity, int)
    return rclpy.impl.logging_rcutils.set_severity_threshold(severity)


def logdebug(message):
    return log(message, severity=LoggingSeverity.DEBUG)


def loginfo(message):
    return log(message, severity=LoggingSeverity.INFO)


def logwarn(message):
    return log(message, severity=LoggingSeverity.WARN)


def logerr(message):
    return log(message, severity=LoggingSeverity.ERROR)


def logfatal(message):
    return log(message, severity=LoggingSeverity.FATAL)


def log(message, severity, **kwargs):
    assert isinstance(severity, LoggingSeverity) or isinstance(severity, int)
    severity = LoggingSeverity(severity)

    return rclpy.impl.logging_rcutils.log(message, severity, **kwargs)
