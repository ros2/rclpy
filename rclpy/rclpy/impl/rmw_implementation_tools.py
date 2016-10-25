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
from rclpy.exceptions import NoImplementationAvailableException
from rclpy.impl.excepthook import add_unhandled_exception_addendum

__rmw_implementations = None
__selected_rmw_implementation = None
__rmw_implementation_module = None

AMENT_INDEX_NAME = 'rmw_python_extension'
RCLPY_IMPLEMENTATION_ENV_NAME = 'RCLPY_IMPLEMENTATION'


def reload_rmw_implementations():
    """(Re)Load the available rmw implementations by inspecting the ament index."""
    global __rmw_implementations
    __rmw_implementations = sorted(ament_index_python.get_resources(AMENT_INDEX_NAME).keys())
    return __rmw_implementations


def get_rmw_implementations():
    """Return a list of available rmw implementations by name."""
    if __rmw_implementations is None:
        reload_rmw_implementations()
    return __rmw_implementations


def select_rmw_implementation(rmw_implementation):
    """
    Set the rmw implementation to be imported by name.

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
    """
    Import and return the selected or default rmw implementation.

    This function can be called multiple times, but the rmw implementation is
    only imported the first time.
    Subsequent calls will return a cached rmw implementation module.

    If :py:func:`select_rmw_implementation` has not previously been called then
    a default implementation will be selected implicitly before loading.

    :raises: :py:class:`NoImplementationAvailableException` if there are no
        implementations available.
    """
    global __rmw_implementation_module
    if __rmw_implementation_module is not None:
        return __rmw_implementation_module
    available_implementations = get_rmw_implementations()
    if __selected_rmw_implementation is None:
        logger = logging.getLogger('rclpy')
        # prefer FastRTPS, otherwise first in alphabetical order
        # the same logic is implemented in
        # rmw_implementation_cmake/cmake/get_default_rmw_implementation.cmake
        default_impl = 'rmw_fastrtps_cpp'
        if default_impl in available_implementations:
            select_rmw_implementation(default_impl)
        elif available_implementations:
            select_rmw_implementation(available_implementations[0])  # select the first one
        else:
            raise NoImplementationAvailableException()
        logger.debug("Implicitly selecting the '{0}' rmw implementation."
                     .format(__selected_rmw_implementation))
    module_name = '._rclpy__{rmw_implementation}'.format(
        rmw_implementation=__selected_rmw_implementation,
    )
    try:
        __rmw_implementation_module = importlib.import_module(module_name, package='rclpy')
    except ImportError as exc:
        if "No module named 'rclpy.{0}".format(module_name) in str(exc):
            if available_implementations:
                rmw_implementations_msg = '\n'.join(
                    ['  - {0}'.format(x) for x in available_implementations]
                )
            else:
                rmw_implementations_msg = '  No rmw implementation has a Python extension.'
            log_args = [
                __selected_rmw_implementation,
                RCLPY_IMPLEMENTATION_ENV_NAME,
                rmw_implementations_msg,
            ]
            # This message will only be printed if this exception goes unhandled.
            add_unhandled_exception_addendum(
                exc,
                "\n"
                "Failed to import the Python extension for the '{0}' rmw implementation.\n"
                "A different rmw implementation can be selected using the '{1}' env variable.\n"
                "These are the available rmw implementations:\n"
                "{2}\n".format(*log_args)
            )
        raise
    return __rmw_implementation_module
