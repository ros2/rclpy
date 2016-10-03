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

import logging
import sys

__original_excepthook = None
__unhandled_exception_addendums = {}


def rclpy_excepthook(exc_type, value, traceback):
    """
    The rclpy custom except hook for unhandled exceptions.

    This excepthook will check for any unhandled exception addendum's,
    log them if they exist, and then call the original excepthook.
    The original excepthook is what ever was in ``sys.excepthook`` when
    :py:func:`install_rclpy_excepthook` was called.

    Any addendum found during this excepthook will be removed after logging.
    """
    if value in __unhandled_exception_addendums:
        logger = logging.getLogger('rclpy')
        logger.error(__unhandled_exception_addendums[value])
        del __unhandled_exception_addendums[value]
    return __original_excepthook(exc_type, value, traceback)


def install_rclpy_excepthook():
    """
    Replace the system excepthook with the custom rclpy excepthook.

    Repeated calls to this function will return without any side effects.
    """
    global __original_excepthook
    if __original_excepthook is not None:
        # The rclpy excepthook has already been installed.
        return
    __original_excepthook = sys.excepthook
    sys.excepthook = rclpy_excepthook


def add_unhandled_exception_addendum(exception, message):
    """
    Add an addendum for the given exception.

    If the exception is never handled, then the addendum will be logged before
    passing the exception to the original excepthook.
    This only works if the custom rclpy excepthook has been installed with
    :py:func:`install_rclpy_excepthook`.
    """
    __unhandled_exception_addendums[exception] = message
