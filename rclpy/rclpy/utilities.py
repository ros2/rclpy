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

from rclpy.constants import S_TO_NS
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


def timeout_sec_to_nsec(timeout_sec):
    """
    Convert timeout in seconds to rcl compatible timeout in nanoseconds.

    Python tends to use floating point numbers in seconds for timeouts. This utility converts a
    python-style timeout to an integer in nanoseconds that can be used by rcl_wait.

    :param timeout_sec: Seconds to wait. Block forever if None or negative. Don't wait if < 1ns
    :type timeout_sec: float or None
    :rtype: int
    :returns: rcl_wait compatible timeout in nanoseconds
    """
    if timeout_sec is None or timeout_sec < 0:
        # Block forever
        return -1
    else:
        # wait for given time
        return int(float(timeout_sec) * S_TO_NS)
