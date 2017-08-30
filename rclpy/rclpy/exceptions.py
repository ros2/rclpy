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
