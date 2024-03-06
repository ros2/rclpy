# Copyright 2016 Open Source Robotics Foundation, Inc.
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
from typing import List, Optional, TYPE_CHECKING, Union

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class NotInitializedException(Exception):
    """Raised when the rclpy implementation is accessed before rclpy.init()."""

    def __init__(self, message: Optional[str] = None) -> None:
        if message:
            Exception.__init__(self, f'rclpy.init() has not been called. msg:{message}')
        else:
            Exception.__init__(self, 'rclpy.init() has not been called.')


class NoTypeSupportImportedException(Exception):
    """Raised when there is no type support imported."""

    def __init__(self) -> None:
        Exception.__init__(self, 'no type_support imported')


class NameValidationException(Exception):
    """Raised when a topic name, node name, or namespace are invalid."""

    def __init__(self, name_type: str, name: str, error_msg: str, invalid_index: int) -> None:
        msg = """\
Invalid {name_type}: {error_msg}:
  '{name}'
   {indent}^\
""".format(name_type=name_type, name=name, error_msg=error_msg, indent=' ' * invalid_index)
        Exception.__init__(self, msg)


InvalidHandle = _rclpy.InvalidHandle


class InvalidNamespaceException(NameValidationException):
    """Raised when a namespace is invalid."""

    def __init__(self, name: str, error_msg: str, invalid_index: int) -> None:
        NameValidationException.__init__(self, 'namespace', name, error_msg, invalid_index)


class InvalidNodeNameException(NameValidationException):
    """Raised when a node name is invalid."""

    def __init__(self, name: str, error_msg: str, invalid_index: int) -> None:
        NameValidationException.__init__(self, 'node name', name, error_msg, invalid_index)


class InvalidTopicNameException(NameValidationException):
    """Raised when a topic name is invalid."""

    def __init__(self, name: str, error_msg: str, invalid_index: int) -> None:
        NameValidationException.__init__(self, 'topic name', name, error_msg, invalid_index)


class InvalidServiceNameException(NameValidationException):
    """Raised when a service name is invalid."""

    def __init__(self, name: str, error_msg: str, invalid_index: int) -> None:
        NameValidationException.__init__(self, 'service name', name, error_msg, invalid_index)


class ParameterException(Exception):
    """Base exception for parameter-related errors."""

    def __init__(self, error_msg: str, parameters: Union[str, List[str]]) -> None:
        Exception.__init__(self, f'{error_msg}: {parameters}')


class ParameterNotDeclaredException(ParameterException):
    """Raised when handling an undeclared parameter when it is not allowed."""

    def __init__(self, parameters: Union[str, List[str]]) -> None:
        ParameterException.__init__(self, 'Invalid access to undeclared parameter(s)', parameters)


class ParameterAlreadyDeclaredException(ParameterException):
    """Raised when declaring a parameter that had been declared before."""

    def __init__(self, parameters: Union[str, List[str]]) -> None:
        ParameterException.__init__(self, 'Parameter(s) already declared', parameters)


class InvalidParameterException(ParameterException):
    """Raised when a parameter to be declared has an invalid name."""

    def __init__(self, parameter_name: str):
        ParameterException.__init__(self, 'Invalid parameter name', parameter_name)


class InvalidParameterTypeException(ParameterException):
    """Raised when a parameter is rejected for having an invalid type."""

    from rclpy.parameter import Parameter

    def __init__(self, desired_parameter: Parameter, expected_type: Parameter.Type) -> None:
        from rclpy.parameter import Parameter
        ParameterException.__init__(
            self,
            f"Trying to set parameter '{desired_parameter._name}' to '{desired_parameter._value}'"
            f" of type '{Parameter.Type.from_parameter_value(desired_parameter._value).name}'"
            f", expecting type '{expected_type}'",
            desired_parameter.name)


class InvalidParameterValueException(ParameterException):
    """Raised when a parameter is rejected by a user callback or when applying a descriptor."""

    if TYPE_CHECKING:
        from rclpy.parameter import AllowableParameterValue

    def __init__(self, parameter: str, value: 'AllowableParameterValue', reason: str) -> None:
        ParameterException.__init__(
            self,
            'Invalid parameter value ({value}) for parameter. Reason: {reason}'.format(
                value=value, reason=reason), parameter)


class ParameterImmutableException(ParameterException):
    """Raised when a read-only parameter is modified."""

    def __init__(self, parameter_name: str):
        ParameterException.__init__(self, 'Attempted to modify read-only parameter',
                                    parameter_name)


class ParameterUninitializedException(ParameterException):
    """Raised when an uninitialized parameter is accessed."""

    def __init__(self, parameter_name: str):
        ParameterException.__init__(
            self,
            f"The parameter '{parameter_name}' is not initialized",
            parameter_name)


class ROSInterruptException(Exception):
    """Raised when an operation is canceled by rclpy shutting down."""

    def __init__(self) -> None:
        Exception.__init__(self, 'rclpy.shutdown() has been called')
