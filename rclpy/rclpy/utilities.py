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

import os
import sys
import threading
from typing import List
from typing import Optional
from typing import Sequence
from typing import Set

import ament_index_python

from rclpy.constants import S_TO_NS
from rclpy.context import Context

g_default_context = None
g_context_lock = threading.Lock()


def get_default_context(*, shutting_down=False) -> Context:
    """Return the global default context singleton."""
    global g_context_lock
    with g_context_lock:
        global g_default_context
        if g_default_context is None:
            g_default_context = Context()
        if shutting_down:
            old_context = g_default_context
            g_default_context = None
            return old_context
        return g_default_context


def remove_ros_args(args: Optional[Sequence[str]] = None) -> List[str]:
    """
    Return a list of only the non-ROS command line arguments.

    :param args: A list of command line arguments to filter from. If None then
        ``sys.argv`` is used instead.
    :returns: A list of all command line arguments that are not used by ROS.
    """
    # imported locally to avoid loading extensions on module import
    from rclpy.impl.implementation_singleton import rclpy_implementation
    return rclpy_implementation.rclpy_remove_ros_args(
        args if args is not None else sys.argv)


def ok(*, context: Optional[Context] = None) -> bool:
    """
    Return ``True`` if the given ``Context`` is initialized and not shut down.

    :param context: a ``Context`` to check, else the global default context is
        used.
    :return: ``True`` if the context is valid.
    """
    if context is None:
        context = get_default_context()
    return context.ok()


def shutdown(*, context=None) -> None:
    """
    Shutdown the given ``Context``.

    :param context: a ``Context`` to check, else the global default context is
        used.
    """
    if context is None:
        context = get_default_context(shutting_down=True)
    context.shutdown()


def try_shutdown(*, context=None) -> None:
    """
    Shutdown the given ``Context`` if not already shutdown.

    :param context: a ``Context`` to check, else the global default context is
        used.
    """
    global g_context_lock
    global g_default_context
    if context is None:
        # Replace the default context with a new one if shutdown was successful
        # or if the context was already shutdown.
        with g_context_lock:
            if g_default_context is None:
                g_default_context = Context()
            g_default_context.try_shutdown()
            if not g_default_context.ok():
                g_default_context = None
    else:
        context.try_shutdown()


def get_rmw_implementation_identifier() -> str:
    """Return the identifier of the current RMW implementation."""
    # imported locally to avoid loading extensions on module import
    from rclpy.impl.implementation_singleton import rclpy_implementation
    return rclpy_implementation.rclpy_get_rmw_implementation_identifier()


def get_available_rmw_implementations() -> Set[str]:
    """
    Return the set of all available RMW implementations as registered in the ament index.

    The result can be overridden by setting an environment variable named
    ``RMW_IMPLEMENTATIONS``.
    The variable can contain RMW implementation names separated by the platform
    specific path separator.

    :raises RuntimeError: if the environment variable includes a missing RMW implementation.
    """
    available_rmw_implementations = ament_index_python.get_resources(
        'rmw_typesupport')
    available_rmw_implementations = {
        name for name in available_rmw_implementations
        if name != 'rmw_implementation'}

    # filter by implementations in environment variable if provided
    rmw_implementations = os.environ.get('RMW_IMPLEMENTATIONS')
    if rmw_implementations:
        rmw_implementations = rmw_implementations.split(os.pathsep)
        missing_rmw_implementations = set(rmw_implementations) - \
            available_rmw_implementations
        if missing_rmw_implementations:
            # TODO(sloretz) function name suggets to me it would return available ones even
            # if some were missing.
            raise RuntimeError(
                f'The RMW implementations {missing_rmw_implementations} '
                "specified in 'RMW_IMPLEMENTATIONS' are not available (" +
                ', '.join(sorted(available_rmw_implementations)) + ')')
        available_rmw_implementations = {
            name for name in available_rmw_implementations
            if name in rmw_implementations}

    return available_rmw_implementations


def timeout_sec_to_nsec(timeout_sec: Optional[float]) -> int:
    """
    Convert timeout in seconds to rcl compatible timeout in nanoseconds.

    Python tends to use floating point numbers in seconds for timeouts. This utility converts a
    python-style timeout to an integer in nanoseconds that can be used by ``rcl_wait``.

    :param timeout_sec: Seconds to wait. Block forever if None or negative. Don't wait if < 1ns
    :returns: rcl_wait compatible timeout in nanoseconds
    """
    if timeout_sec is None or timeout_sec < 0:
        # Block forever
        return -1
    else:
        # wait for given time
        return int(float(timeout_sec) * S_TO_NS)
