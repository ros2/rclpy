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


class NotInitializedException(Exception):
    """Raised when the rclpy implementation is accessed before rclpy.init()."""

    def __init__(self, *args):
        Exception.__init__(self, 'rclpy.init() has not been called', *args)


class NoTypeSupportImportedException(Exception):
    """Raised when there is no type support imported."""

    def __init__(self, *args):
        Exception.__init__(self, 'no type_support imported')


class NameValidationException(Exception):
    """Raised when a topic name, node name, or namespace are invalid."""

    def __init__(self, name_type, name, error_msg, invalid_index, *args):
        msg = """\
Invalid {name_type}: {error_msg}:
  '{name}'
   {indent}^\
""".format(name_type=name_type, name=name, error_msg=error_msg, indent=' ' * invalid_index)
        Exception.__init__(self, msg)


class InvalidNamespaceException(NameValidationException):
    """Raised when a namespace is invalid."""

    def __init__(self, name, error_msg, invalid_index, *args):
        NameValidationException.__init__(self, 'namespace', name, error_msg, invalid_index)


class InvalidNodeNameException(NameValidationException):
    """Raised when a node name is invalid."""

    def __init__(self, name, error_msg, invalid_index, *args):
        NameValidationException.__init__(self, 'node name', name, error_msg, invalid_index)


class InvalidTopicNameException(NameValidationException):
    """Raised when a topic name is invalid."""

    def __init__(self, name, error_msg, invalid_index, *args):
        NameValidationException.__init__(self, 'topic name', name, error_msg, invalid_index)


class InvalidServiceNameException(NameValidationException):
    """Raised when a service name is invalid."""

    def __init__(self, name, error_msg, invalid_index, *args):
        NameValidationException.__init__(self, 'service name', name, error_msg, invalid_index)


class ParameterException(Exception):
    """Base exception for parameter-related errors."""

    def __init__(self, error_msg, parameters, *args):
        Exception.__init__(self, f'{error_msg}: {parameters}')


class ParameterNotDeclaredException(ParameterException):
    """Raised when handling an undeclared parameter when it is not allowed."""

    def __init__(self, parameters, *args):
        Exception.__init__(self, 'Invalid access to undeclared parameter(s)', parameters, *args)


class ParameterAlreadyDeclaredException(ParameterException):
    """Raised when declaring a parameter that had been declared before."""

    def __init__(self, parameters, *args):
        Exception.__init__(self, 'Parameter(s) already declared', parameters, *args)


class InvalidParameterException(ParameterException):
    """Raised when a parameter to be declared has an invalid name."""

    def __init__(self, parameter, *args):
        Exception.__init__(self, 'Invalid parameter name', parameter, *args)


class InvalidParameterValueException(ParameterException):
    """Raised when a parameter is rejected by a user callback or when applying a descriptor."""

    def __init__(self, parameter, value, reason, *args):
        Exception.__init__(
            self,
            'Invalid parameter value ({value}) for parameter. Reason: {reason}'.format(
                value=value, reason=reason), parameter, *args)


class ParameterImmutableException(ParameterException):
    """Raised when a read-only parameter is modified."""

    def __init__(self, parameter, *args):
        Exception.__init__(self, 'Attempted to modify read-only parameter', parameter, *args)


class ROSInterruptException(Exception):
    """Raised when an operation is canceled by rclpy shutting down."""

    def __init__(self, *args):
        Exception.__init__(self, 'rclpy.shutdown() has been called', *args)
