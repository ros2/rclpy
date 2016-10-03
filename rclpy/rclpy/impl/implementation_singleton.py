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

"""
Provide singleton access to the rclpy implementation.

The singleton is called ``rclpy_implementation`` and is in this module.
For example, you might use it like this:

.. code::

    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

    _rclpy.rclpy_init()
    while _rclpy.rclpy_ok():
        # ...

Before you call :py:func:`rclpy.init` this singleton will point to an instance
of the :py:class:`ImplementationPlaceholder` class, which will raise a
:py:class:`rclpy.exceptions.NotInitializedException` when ever accessed.
After initialization it will point to the rclpy implementation C module.
"""

from rclpy.exceptions import NotInitializedException
from rclpy.impl.object_proxy import ObjectProxy


class ImplementationPlaceholder:
    """
    Placeholder for the rclpy implementation module.

    This class will raise a :py:class:`NotInitializedException` when used.
    """

    def __getattr__(self, key):
        if key in ['__repr__']:
            return object.__getattr__(key)
        raise NotInitializedException()

rclpy_implementation = ObjectProxy(ImplementationPlaceholder())


def set_rclpy_implementation(implementation):
    """Set the rclpy implementation singleton."""
    # Update the ObjectProxy to point to the rmw implementation specific module.
    rclpy_implementation.__actual__ = implementation


def rclpy_implementation_is_placeholder(implementation=None):
    """
    Return True if the implementation is a placeholder, else False.

    :param implementation: implementation to check, defaults to the singleton
    :returns: bool
    """
    implementation = rclpy_implementation if implementation is None else implementation
    return isinstance(implementation, ImplementationPlaceholder)
