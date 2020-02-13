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
A collection of functions for writing a ROS program.

A typical ROS program consists of the following operations:

#. Initialization
#. Create one or more ROS nodes
#. Process node callbacks
#. Shutdown

Inititalization is done by calling :func:`init` for a particular :class:`.Context`.
This must be done before any ROS nodes can be created.

Creating a ROS node is done by calling :func:`create_node` or by instantiating a
:class:`.Node`.
A node can be used to create common ROS entities like publishers, subscriptions, services,
and actions.

After a node is created, items of work can be done (e.g. subscription callbacks) by *spinning* on
the node.
The following functions can be used to process work that is waiting to be executed: :func:`spin`,
:func:`spin_once`, and :func:`spin_until_future_complete`.

When finished with a previously initialized :class:`.Context` (ie. done using
all ROS nodes associated with the context), the :func:`shutdown` function should be called.
This will invalidate all entities derived from the context.
"""

from typing import List
from typing import Optional
from typing import TYPE_CHECKING

from rclpy.context import Context
from rclpy.parameter import Parameter
from rclpy.task import Future
from rclpy.utilities import get_default_context
from rclpy.utilities import get_rmw_implementation_identifier  # noqa: F401
from rclpy.utilities import ok  # noqa: F401 forwarding to this module
from rclpy.utilities import shutdown as _shutdown
from rclpy.utilities import try_shutdown  # noqa: F401

# Avoid loading extensions on module import
if TYPE_CHECKING:
    from rclpy.executors import Executor  # noqa: F401
    from rclpy.node import Node  # noqa: F401


def init(*, args: Optional[List[str]] = None, context: Context = None) -> None:
    """
    Initialize ROS communications for a given context.

    :param args: List of command line arguments.
    :param context: The context to initialize. If ``None``, then the default context is used
        (see :func:`.get_default_context`).
    """
    context = get_default_context() if context is None else context
    return context.init(args)


# The global spin functions need an executor to do the work
# A persistent executor can re-run async tasks that yielded in rclpy.spin*().
__executor = None


def get_global_executor() -> 'Executor':
    global __executor
    if __executor is None:
        # imported locally to avoid loading extensions on module import
        from rclpy.executors import SingleThreadedExecutor
        __executor = SingleThreadedExecutor()
    return __executor


def shutdown(*, context: Context = None) -> None:
    """
    Shutdown a previously initialized context.

    This will also shutdown the global executor.

    :param context: The context to invalidate. If ``None``, then the default context is used
        (see :func:`.get_default_context`).
    """
    global __executor
    if __executor is not None:
        __executor.shutdown()
        __executor = None
    _shutdown(context=context)


def create_node(
    node_name: str,
    *,
    context: Context = None,
    cli_args: List[str] = None,
    namespace: str = None,
    use_global_arguments: bool = True,
    enable_rosout: bool = True,
    start_parameter_services: bool = True,
    parameter_overrides: List[Parameter] = None,
    allow_undeclared_parameters: bool = False,
    automatically_declare_parameters_from_overrides: bool = False
) -> 'Node':
    """
    Create an instance of :class:`.Node`.

    :param node_name: A name to give to the node.
    :param context: The context to associated with the node, or ``None`` for the default global
        context.
    :param cli_args: Command line arguments to be used by the node. Being specific to a ROS node,
        an implicit `--ros-args` scope flag always precedes these arguments.
    :param namespace: The namespace prefix to apply to entities associated with the node
        (node name, topics, etc).
    :param use_global_arguments: ``False`` if the node should ignore process-wide command line
        arguments.
    :param enable_rosout: ``False`` if the node should ignore rosout logging.
    :param start_parameter_services: ``False`` if the node should not create parameter services.
    :param parameter_overrides: A list of :class:`.Parameter` which are used to override the
        initial values of parameters declared on this node.
     :param allow_undeclared_parameters: if True undeclared parameters are allowed, default False.
        This option doesn't affect `parameter_overrides`.
    :param automatically_declare_parameters_from_overrides: If True, the "parameter overrides" will
        be used to implicitly declare parameters on the node during creation, default False.
    :return: An instance of the newly created node.
    """
    # imported locally to avoid loading extensions on module import
    from rclpy.node import Node  # noqa: F811
    return Node(
        node_name, context=context, cli_args=cli_args, namespace=namespace,
        use_global_arguments=use_global_arguments,
        enable_rosout=enable_rosout,
        start_parameter_services=start_parameter_services,
        parameter_overrides=parameter_overrides,
        allow_undeclared_parameters=allow_undeclared_parameters,
        automatically_declare_parameters_from_overrides=(
            automatically_declare_parameters_from_overrides
        ))


def spin_once(node: 'Node', *, executor: 'Executor' = None, timeout_sec: float = None) -> None:
    """
    Execute one item of work or wait until a timeout expires.

    One callback will be executed by the provided executor as long as that callback is ready
    before the timeout expires.

    If no executor is provided (ie. ``None``), then the global executor is used.
    It is possible the work done is for a node other than the one provided if the global executor
    has a partially completed coroutine.

    :param node: A node to add to the executor to check for work.
    :param executor: The executor to use, or the global executor if ``None``.
    :param timeout_sec: Seconds to wait. Block forever if ``None`` or negative. Don't wait if 0.
    """
    executor = get_global_executor() if executor is None else executor
    try:
        executor.add_node(node)
        executor.spin_once(timeout_sec=timeout_sec)
    finally:
        executor.remove_node(node)


def spin(node: 'Node', executor: 'Executor' = None) -> None:
    """
    Execute work and block until the context associated with the executor is shutdown.

    Callbacks will be executed by the provided executor.

    This function blocks.

    :param node: A node to add to the executor to check for work.
    :param executor: The executor to use, or the global executor if ``None``.
    """
    executor = get_global_executor() if executor is None else executor
    try:
        executor.add_node(node)
        while executor.context.ok():
            executor.spin_once()
    finally:
        executor.remove_node(node)


def spin_until_future_complete(
    node: 'Node',
    future: Future,
    executor: 'Executor' = None,
    timeout_sec: float = None
) -> None:
    """
    Execute work until the future is complete.

    Callbacks and other work will be executed by the provided executor until ``future.done()``
    returns ``True`` or the context associated with the executor is shutdown.

    :param node: A node to add to the executor to check for work.
    :param future: The future object to wait on.
    :param executor: The executor to use, or the global executor if ``None``.
    :param timeout_sec: Seconds to wait. Block until the future is complete
        if ``None`` or negative. Don't wait if 0.
    """
    executor = get_global_executor() if executor is None else executor
    try:
        executor.add_node(node)
        executor.spin_until_future_complete(future, timeout_sec)
    finally:
        executor.remove_node(node)
