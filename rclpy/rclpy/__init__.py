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

from rclpy.utilities import get_default_context
from rclpy.utilities import get_rmw_implementation_identifier  # noqa
from rclpy.utilities import ok  # noqa: forwarding to this module
from rclpy.utilities import shutdown as _shutdown
from rclpy.utilities import try_shutdown  # noqa


def init(*, args=None, context=None):
    context = get_default_context() if context is None else context
    # imported locally to avoid loading extensions on module import
    from rclpy.impl.implementation_singleton import rclpy_implementation
    return rclpy_implementation.rclpy_init(args if args is not None else sys.argv, context.handle)


# The global spin functions need an executor to do the work
# A persistent executor can re-run async tasks that yielded in rclpy.spin*().
__executor = None


def get_global_executor():
    global __executor
    if __executor is None:
        # imported locally to avoid loading extensions on module import
        from rclpy.executors import SingleThreadedExecutor
        __executor = SingleThreadedExecutor()
    return __executor


def shutdown(*, context=None):
    global __executor
    if __executor is not None:
        __executor.shutdown()
        __executor = None
    _shutdown(context=context)


def create_node(
    node_name, *, context=None, cli_args=None, namespace=None, use_global_arguments=True,
    start_parameter_services=True, initial_parameters=None
):
    """
    Create an instance of :class:`rclpy.node.Node`.

    :param node_name: A unique name to give to this node.
    :param context: The context to be associated with, or None for the default global context.
    :param cli_args: A list of strings of command line args to be used only by this node.
    :param namespace: The namespace to which relative topic and service names will be prefixed.
    :param use_global_arguments: False if the node should ignore process-wide command line args.
    :param start_parameter_services: False if the node should not create parameter services.
    :param initial_parameters: A list of rclpy.parameter.Parameters to be set during node creation.
    :return: An instance of a node
    :rtype: :class:`rclpy.node.Node`
    """
    # imported locally to avoid loading extensions on module import
    from rclpy.node import Node
    return Node(
        node_name, context=context, cli_args=cli_args, namespace=namespace,
        use_global_arguments=use_global_arguments,
        start_parameter_services=start_parameter_services,
        initial_parameters=initial_parameters)


def spin_once(node, *, executor=None, timeout_sec=None):
    """
    Execute one item of work or wait until timeout expires.

    One callback will be executed in a SingleThreadedExecutor as long as that
    callback is ready before the timeout expires.

    It is possible the work done may be for a node other than the one passed to this method
    if the global executor has a partially completed coroutine.

    :param node: A node to add to the executor to check for work.
    :param executor: The executor to use, or the global executor if None is passed.
    :param timeout_sec: Seconds to wait. Block forever if None or negative. Don't wait if 0
    :return: Always returns None regardless whether work executes or timeout expires.
    :rtype: None
    """
    executor = get_global_executor() if executor is None else executor
    try:
        executor.add_node(node)
        executor.spin_once(timeout_sec=timeout_sec)
    finally:
        executor.remove_node(node)


def spin(node, executor=None):
    """
    Execute work blocking until the library is shutdown.

    Callbacks will be executed in a SingleThreadedExecutor until shutdown() is called.
    This method blocks.

    :param node: A node to add to the executor to check for work.
    :param executor: The executor to use, or the global executor if None is passed.
    :return: Always returns None regardless whether work executes or timeout expires.
    :rtype: None
    """
    executor = get_global_executor() if executor is None else executor
    try:
        executor.add_node(node)
        while executor.context.ok():
            executor.spin_once()
    finally:
        executor.remove_node(node)


def spin_until_future_complete(node, future, executor=None):
    """
    Execute work until the future is complete.

    Callbacks and other work will be executed in a SingleThreadedExecutor until future.done()
    returns True or rclpy is shutdown.

    :param node: A node to add to the executor to check for work.
    :param future: The future object to wait on.
    :param executor: The executor to use, or the global executor if None is passed.
    :type future: rclpy.task.Future
    """
    executor = get_global_executor() if executor is None else executor
    try:
        executor.add_node(node)
        executor.spin_until_future_complete(future)
    finally:
        executor.remove_node(node)
