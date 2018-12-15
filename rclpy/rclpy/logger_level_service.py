# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from rcl_interfaces.srv import GetLoggerLevel, SetLoggerLevel
from rclpy.logging import get_logger_effective_level
from rclpy.qos import qos_profile_parameters
from rclpy.validate_topic_name import TOPIC_SEPARATOR_STRING


class LoggerLevelService:

    def __init__(self, node):
        self._node = node
        nodename = node.get_name()

        get_logger_level_service_name = \
            TOPIC_SEPARATOR_STRING.join((nodename, 'get_logger_level'))
        node.create_service(
            GetLoggerLevel, get_logger_level_service_name,
            self._get_logger_level_callback, qos_profile=qos_profile_parameters
        )
        set_logger_level_service_name = TOPIC_SEPARATOR_STRING.join((nodename, 'set_logger_level'))
        node.create_service(
            SetLoggerLevel, set_logger_level_service_name, self._set_logger_level_callback,
            qos_profile=qos_profile_parameters
        )

    def _get_logger_level_callback(self, request, response):
        response.logger_level = get_logger_effective_level(request.name)
        return response

    def _set_logger_level_callback(self, request, response):
        self._node.get_logger().set_level(request.logger_level)
        return response
