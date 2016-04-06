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

import importlib
import logging

import ament_index_python

from rclpy.exceptions import ImplementationAlreadyImportedException
from rclpy.exceptions import InvalidRCLPYImplementation

__rmw_implementations = None
__selected_rmw_implementation = None
__rmw_implementation_module = None


def reload_rmw_implementations():
    """(Re)Load the available rmw implementations by inspecting the ament index."""
    global __rmw_implementations
    __rmw_implementations = sorted(ament_index_python.get_resources('rmw_implementation').keys())

    # Remove implementations that are being filtered for in the rclpy CMakeLists so
    # that they cannot be selected
    __rmw_implementations = [
        rmw_impl for rmw_impl in __rmw_implementations
        if rmw_impl not in ['rmw_connext_dynamic_cpp', 'rmw_fastrtps_cpp']]
    return __rmw_implementations


def get_rmw_implementations():
    """Return a list of available rmw implementations by name."""
    if __rmw_implementations is None:
        reload_rmw_implementations()
    return __rmw_implementations


def select_rmw_implementation(rmw_implementation):
    """Set the rmw implementation to be imported by name.

    Can be called multiple times, but only before calling :py:func:`rclpy.init`
    and/or :py:func:`import_rmw_implementation`.

    If given an rmw implementation that is not in the list provided by
    :py:func:`get_rmw_implementations` then the
    :py:class:`InvalidRCLPYImplementation` exception will be raised.

    :raises: :py:class:`ImplementationAlreadyImportedException` if
        :py:func:`import_rmw_implementation` has already been called and the
        module already imported.
    """
    global __selected_rmw_implementation
    if __rmw_implementation_module is not None:
        raise ImplementationAlreadyImportedException()
    if rmw_implementation not in get_rmw_implementations():
        raise InvalidRCLPYImplementation()
    __selected_rmw_implementation = rmw_implementation


def import_rmw_implementation():
    """Import and return the selected or default rmw implementation.

    This function can be called multiple times, but the rmw implementation is
    only imported the first time.
    Subsequent calls will return a cached rmw implementation module.

    If :py:func:`select_rmw_implementation` has not previously been called then
    a default implementation will be selected implicitly before loading.
    """
    global __rmw_implementation_module
    if __rmw_implementation_module is not None:
        return __rmw_implementation_module
    if __selected_rmw_implementation is None:
        logger = logging.getLogger('rclpy')
        select_rmw_implementation(get_rmw_implementations()[0])  # select the first one
        logger.debug("Implicitly selecting the '{0}' rmw implementation."
                     .format(__selected_rmw_implementation))
    module_name = '._rclpy__{rmw_implementation}'.format(
        rmw_implementation=__selected_rmw_implementation,
    )
    __rmw_implementation_module = importlib.import_module(module_name, package='rclpy')
    return __rmw_implementation_module
