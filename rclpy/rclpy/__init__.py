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

import sys

from rclpy.utilities import get_rmw_implementation_identifier  # noqa
from rclpy.utilities import ok
from rclpy.utilities import shutdown  # noqa
from rclpy.utilities import try_shutdown  # noqa


def init(*, args=None):
    # imported locally to avoid loading extensions on module import
    from rclpy.impl.implementation_singleton import rclpy_implementation
    return rclpy_implementation.rclpy_init(
        args if args is not None else sys.argv)


def create_node(node_name, *, namespace=None):
    # imported locally to avoid loading extensions on module import
    from rclpy.node import Node
    return Node(node_name, namespace=namespace)


def spin_once(node, *, timeout_sec=None):
    # imported locally to avoid loading extensions on module import
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin_once(timeout_sec=timeout_sec)
    finally:
        executor.shutdown()


def spin(node):
    # imported locally to avoid loading extensions on module import
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    try:
        executor.add_node(node)
        while ok():
            executor.spin_once()
    finally:
        executor.shutdown()


def spin_until_future_complete(node, future):
    """
    Execute work until the future is complete.

    Callbacks and other work will be executed in a SingleThreadedExecutor until future.done()
    returns True or rclpy is shutdown.

    :param future: The future object to wait on.
    :type future: rclpy.task.Future
    """
    # imported locally to avoid loading extensions on module import
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin_until_future_complete(future)
    finally:
        executor.shutdown()
