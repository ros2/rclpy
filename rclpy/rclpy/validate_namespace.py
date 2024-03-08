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

from rclpy.exceptions import InvalidNamespaceException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


def validate_namespace(namespace: str) -> Literal[True]:
    """
    Validate a given namespace, and raise an exception if it is invalid.

    Unlike the node constructor, which allows namespaces without a leading '/'
    and empty namespaces "", this function requires that the namespace be
    absolute and non-empty.

    If the namespace is invalid then rclpy.exceptions.InvalidNamespaceException
    will be raised.

    :param namespace: namespace to be validated
    :returns: True when it is valid
    :raises: InvalidNamespaceException: when the namespace is invalid
    """
    result = _rclpy.rclpy_get_validation_error_for_namespace(namespace)
    if result is None:
        return True
    error_msg, invalid_index = result
    raise InvalidNamespaceException(namespace, error_msg, invalid_index)
