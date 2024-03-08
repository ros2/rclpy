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

from typing import Literal

from rclpy.exceptions import InvalidServiceNameException
from rclpy.exceptions import InvalidTopicNameException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


def validate_full_topic_name(name: str, *, is_service: bool = False) -> Literal[True]:
    """
    Validate a given topic or service name, and raise an exception if invalid.

    The name must be fully-qualified and already expanded.

    If the name is invalid then rclpy.exceptions.InvalidTopicNameException
    will be raised.

    :param name: topic or service name to be validated
    :param is_service: if true, InvalidServiceNameException is raised
    :returns: True when it is valid
    :raises: InvalidTopicNameException: when the name is invalid
    """
    result = _rclpy.rclpy_get_validation_error_for_full_topic_name(name)
    if result is None:
        return True
    error_msg, invalid_index = result
    if is_service:
        raise InvalidServiceNameException(name, error_msg, invalid_index)
    else:
        raise InvalidTopicNameException(name, error_msg, invalid_index)
