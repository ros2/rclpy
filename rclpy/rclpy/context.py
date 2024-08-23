# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from inspect import ismethod
import sys
import threading
from types import MethodType, TracebackType
from typing import Callable
from typing import ContextManager
from typing import List
from typing import Optional
from typing import Type
from typing import TYPE_CHECKING
from typing import Union
import warnings
import weakref

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

if TYPE_CHECKING:
    from rclpy.node import Node

g_logging_configure_lock = threading.Lock()
g_logging_ref_count: int = 0


class Context(ContextManager['Context']):
    """
    Encapsulates the lifecycle of init and shutdown.

    Context objects should not be reused, and are finalized in their destructor.
    Wraps the `rcl_context_t` type.

    :Example:
        >>> from rclpy.context import Context
        >>> with Context() as context:
        >>>     context.ok()
        True

    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._callbacks: List[Union['weakref.WeakMethod[MethodType]', Callable[[], None]]] = []
        self._logging_initialized = False
        self.__context: Optional[_rclpy.Context] = None
        self.__node_weak_ref_list: List[weakref.ReferenceType['Node']] = []

    @property
    def handle(self) -> Optional[_rclpy.Context]:
        return self.__context

    def destroy(self) -> None:
        if self.__context:
            self.__context.destroy_when_not_in_use()

    def init(self,
             args: Optional[List[str]] = None,
             *,
             initialize_logging: bool = True,
             domain_id: Optional[int] = None) -> None:
        """
        Initialize ROS communications for a given context.

        :param args: List of command line arguments.
        :param initialize_logging: Whether to initialize logging for the whole process.
            The default is to initialize logging.
        :param domain_id: Which domain ID to use for this context.
            If None (the default), domain ID 0 is used.
        """
        # imported locally to avoid loading extensions on module import
        from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

        global g_logging_ref_count
        with self._lock:
            if domain_id is not None and domain_id < 0:
                raise RuntimeError(
                    'Domain id ({}) should not be lower than zero.'
                    .format(domain_id))

            if self.__context is not None:
                raise RuntimeError('Context.init() must only be called once')

            self.__context = _rclpy.Context(
                args if args is not None else sys.argv,
                domain_id if domain_id is not None else _rclpy.RCL_DEFAULT_DOMAIN_ID)
            if initialize_logging and not self._logging_initialized:
                with g_logging_configure_lock:
                    g_logging_ref_count += 1
                    if g_logging_ref_count == 1:
                        _rclpy.rclpy_logging_configure(self.__context)
                self._logging_initialized = True

    def track_node(self, node: 'Node') -> None:
        """
        Track a Node associated with this Context.

        When the Context is destroyed, it will destroy every Node it tracks.

        :param node: The node to take a weak reference to.
        """
        with self._lock:
            self.__node_weak_ref_list.append(weakref.ref(node))

    def untrack_node(self, node: 'Node') -> None:
        """
        Stop tracking a Node associated with this Context.

        If a Node is destroyed before the context, we no longer need to track it for destruction of
        the Context, so remove it here.
        """
        with self._lock:
            for index, weak_node in enumerate(self.__node_weak_ref_list):
                node_in_list = weak_node()
                if node_in_list is node:
                    found_index = index
                    break
            else:
                # Odd that we didn't find the node in the list, but just get out
                return

            del self.__node_weak_ref_list[found_index]

    def ok(self) -> bool:
        """Check if context hasn't been shut down."""
        with self._lock:
            if self.__context is None:
                return False
            with self.__context:
                return self.__context.ok()

    def _call_on_shutdown_callbacks(self) -> None:
        for weak_method in self._callbacks:
            callback = weak_method()
            if callback is not None:
                callback()
        self._callbacks = []

    def _cleanup(self) -> None:
        for weak_node in self.__node_weak_ref_list:
            node = weak_node()
            if node is not None:
                node.destroy_node()

        self.__context.shutdown()
        self._call_on_shutdown_callbacks()
        self._logging_fini()

    def shutdown(self) -> None:
        """Shutdown this context."""
        if self.__context is None:
            raise RuntimeError('Context must be initialized before it can be shutdown')
        with self.__context, self._lock:
            self._cleanup()

    def try_shutdown(self) -> None:
        """Shutdown this context, if not already shutdown."""
        if self.__context is None:
            return
        with self.__context, self._lock:
            if self.__context.ok():
                self._cleanup()

    def _remove_callback(self, weak_method: 'weakref.WeakMethod[MethodType]') -> None:
        self._callbacks.remove(weak_method)

    def on_shutdown(self, callback: Callable[[], None]) -> None:
        """Add a callback to be called on shutdown."""
        if not callable(callback):
            raise TypeError('callback should be a callable, got {}', type(callback))

        if self.__context is None:
            with self._lock:
                if ismethod(callback):
                    self._callbacks.append(weakref.WeakMethod(callback, self._remove_callback))
                else:
                    self._callbacks.append(callback)
            return

        with self.__context, self._lock:
            if not self.__context.ok():
                callback()
            else:
                if ismethod(callback):
                    self._callbacks.append(weakref.WeakMethod(callback, self._remove_callback))
                else:
                    self._callbacks.append(callback)

    def _logging_fini(self) -> None:
        # This function must be called with self._lock held.
        from rclpy.impl.implementation_singleton import rclpy_implementation
        global g_logging_ref_count
        if self._logging_initialized:
            with g_logging_configure_lock:
                g_logging_ref_count -= 1
                if g_logging_ref_count == 0:
                    rclpy_implementation.rclpy_logging_fini()
                if g_logging_ref_count < 0:
                    raise RuntimeError(
                        'Unexpected error: logger ref count should never be lower that zero')
            self._logging_initialized = False

    def get_domain_id(self) -> int:
        """Get domain id of context."""
        if self.__context is None:
            raise RuntimeError('Context must be initialized before it can have a domain id')
        with self.__context, self._lock:
            return self.__context.get_domain_id()

    def __enter__(self) -> 'Context':
        if self.__context is None:
            # init() hasn't been called yet; for backwards compatibility, initialize and warn
            warnings.warn('init() must be called on a Context before using it in a Python context '
                          'manager. Calling init() with no arguments, this usage is deprecated')
            self.init()

        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> None:
        self.try_shutdown()
