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

from rclpy.exceptions import InvalidNodeNameException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


def validate_node_name(node_name: str) -> Literal[True]:
    """
    Validate a given node_name, and raise an exception if it is invalid.

    If the node_name is invalid then rclpy.exceptions.InvalidNodeNameException
    will be raised.

    :param node_name: node_name to be validated
    :returns: True when it is valid
    :raises: InvalidNodeNameException: when the node_name is invalid
    """
    result = _rclpy.rclpy_get_validation_error_for_node_name(node_name)
    if result is None:
        return True
    error_msg, invalid_index = result
    raise InvalidNodeNameException(node_name, error_msg, invalid_index)
