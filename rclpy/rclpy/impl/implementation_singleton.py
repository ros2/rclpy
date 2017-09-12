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
Provide singleton access to the rclpy C module.

For example, you might use it like this:

.. code::

    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

    _rclpy.rclpy_init()
    while _rclpy.rclpy_ok():
        # ...
"""

import importlib
import os

try:
    rclpy_implementation = importlib.import_module('._rclpy', package='rclpy')
except ImportError as e:
    if os.path.isfile(e.path):
        e.msg += \
            "\nThe C extension '%s' failed to be imported while being present on the system." \
            " Please refer to '%s' for possible solutions" % \
            (e.path, 'https://github.com/ros2/ros2/wiki/Rclpy-Import-error-hint')
    raise
