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

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class LoggingSeverity(IntEnum):
    """
    Enum for logging severity levels.

    This enum must match the one defined in rcutils/logging.h
    """

    UNSET = _rclpy.RCUTILS_LOG_SEVERITY.RCUTILS_LOG_SEVERITY_UNSET
    DEBUG = _rclpy.RCUTILS_LOG_SEVERITY.RCUTILS_LOG_SEVERITY_DEBUG
    INFO = _rclpy.RCUTILS_LOG_SEVERITY.RCUTILS_LOG_SEVERITY_INFO
    WARN = _rclpy.RCUTILS_LOG_SEVERITY.RCUTILS_LOG_SEVERITY_WARN
    ERROR = _rclpy.RCUTILS_LOG_SEVERITY.RCUTILS_LOG_SEVERITY_ERROR
    FATAL = _rclpy.RCUTILS_LOG_SEVERITY.RCUTILS_LOG_SEVERITY_FATAL
