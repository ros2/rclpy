# Copyright 2016-2019 Open Source Robotics Foundation, Inc.
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

"""
Provide singleton access to the rclpy C modules.

For example, you might use it like this:

.. code::

    from rclpy.impl.implementation_singleton import get_rclpy_implementation

    get_rclpy_implementation().rclpy_init()
    while get_rclpy_implementation().rclpy_ok():
        # ...
"""

from rclpy.impl import _import


def get_rclpy_implementation():
    """
    Get the rclpy C extension.

    This function shouldn't be called from module level.
    """
    return _import('._rclpy')


def get_rclpy_action_implementation():
    """
    Get the rclpy action C extension.

    This function shouldn't be called from module level.
    """
    return _import('._rclpy_action')


def get_rclpy_logging_implementation():
    """
    Get the rclpy logging C extension.

    This function shouldn't be called from module level.
    """
    return _import('._rclpy_logging')


def get_rclpy_signal_handler_implementation():
    """
    Get the rclpy C extension.

    This function shouldn't be called from module level.
    """
    return _import('._rclpy_signal_handler')


def get_rclpy_pycapsule_implementation():
    """
    Get the rclpy pycapsule C extension.

    This function shouldn't be called from module level.
    """
    return _import('._rclpy_pycapsule')
