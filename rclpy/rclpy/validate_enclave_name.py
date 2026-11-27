# Copyright 2026 Open Source Robotics Foundation, Inc.
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

from rclpy.exceptions import InvalidEnclaveNameException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


def validate_enclave_name(enclave_name: str) -> Literal[True]:
    """
    Validate a given enclave name, and raise an exception if it is invalid.

    Enclave names follow the same rules as namespaces: they must be absolute,
    non-empty, and made up of valid name tokens separated by a single '/'.

    If the enclave name is invalid then rclpy.exceptions.InvalidEnclaveNameException
    will be raised.

    :param enclave_name: enclave name to be validated
    :returns: True when it is valid
    :raises: InvalidEnclaveNameException: when the enclave name is invalid
    """
    result = _rclpy.rclpy_get_validation_error_for_enclave_name(enclave_name)
    if result is None:
        return True
    error_msg, invalid_index = result
    raise InvalidEnclaveNameException(enclave_name, error_msg, invalid_index)
