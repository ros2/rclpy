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


class ImplementationAlreadyImportedException(Exception):
    """Raised on select_rmw_implemenation() after import_rmw_implementation() has been called."""

    def __init__(self, *args):
        Exception.__init__(self, 'rmw implementation already imported', *args)


class InvalidRCLPYImplementation(Exception):
    """Raised when an invalid RCLPYImplementation is requested."""

    def __init__(self, *args):
        Exception.__init__(self, 'requested invalid rmw implementation', *args)


class NotInitializedException(Exception):
    """Raised when the rclpy implementation is accessed before rclpy.init()."""

    def __init__(self, *args):
        Exception.__init__(self, 'rclpy.init() has not been called', *args)


class NoImplementationAvailableException(Exception):
    """Raised when there is no rmw implementation with a Python extension available."""

    def __init__(self, *args):
        Exception.__init__(self, 'no rmw implementation with a Python extension available')


class NoTypeSupportImportedException(Exception):
    """Raised when there is no type support imported."""

    def __init__(self, *args):
        Exception.__init__(self, 'no type_support imported')
