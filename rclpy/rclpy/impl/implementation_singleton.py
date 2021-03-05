# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

    _rclpy.rclpy_init()
    while _rclpy.rclpy_ok():
        # ...
"""

from rpyutils import import_c_library
package = 'rclpy'

rclpy_implementation = import_c_library('._rclpy', package)
rclpy_action_implementation = import_c_library('._rclpy_action', package)
rclpy_logging_implementation = import_c_library('._rclpy_logging', package)
rclpy_signal_handler_implementation = import_c_library('._rclpy_signal_handler', package)
rclpy_handle_implementation = import_c_library('._rclpy_handle', package)
rclpy_pycapsule_implementation = import_c_library('._rclpy_pycapsule', package)


# Temporary code for converting giant _rclpy module to pybind11
def _combine_split_modules():
    global rclpy_implementation
    module = import_c_library('._rclpy_pybind11', package)
    for attr in dir(module):
        thing = getattr(module, attr)
        if attr.startswith('rclpy_'):
            # It's a wrapped C function
            setattr(rclpy_implementation, attr, thing)
        elif isinstance(thing, type):
            # It's a custom type
            setattr(rclpy_implementation, attr, thing)


_combine_split_modules()
del _combine_split_modules
