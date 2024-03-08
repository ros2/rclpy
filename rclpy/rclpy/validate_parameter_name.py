# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from rclpy.exceptions import InvalidParameterException


def validate_parameter_name(name: str) -> Literal[True]:
    """
    Validate a given parameter name, and raise an exception if invalid.

    The name does not have to be fully-qualified and is not expanded.

    If the name is invalid then rclpy.exceptions.InvalidParameterException
    will be raised.

    :param name: parameter name to be validated.
    :raises: InvalidParameterException: when the name is invalid.
    """
    # TODO(jubeira): add parameter name check to be implemented at RCL level.
    if not name:
        raise InvalidParameterException(name)

    return True
