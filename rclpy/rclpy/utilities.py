# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import threading

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

g_shutdown_lock = threading.Lock()


def ok():
    with g_shutdown_lock:
        return _rclpy.rclpy_ok()


def shutdown():
    with g_shutdown_lock:
        return _rclpy.rclpy_shutdown()


def try_shutdown():
    """Shutdown rclpy if not already shutdown."""
    with g_shutdown_lock:
        if _rclpy.rclpy_ok():
            return _rclpy.rclpy_shutdown()


def get_rmw_implementation_identifier():
    return _rclpy.rclpy_get_rmw_implementation_identifier()
