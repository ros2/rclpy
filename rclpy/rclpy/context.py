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

import sys
import threading
from typing import Callable
from typing import List
from typing import Optional
import weakref


g_logging_configure_lock = threading.Lock()
g_logging_ref_count = 0


class Context:
    """
    Encapsulates the lifecycle of init and shutdown.

    Context objects should not be reused, and are finalized in their destructor.

    Wraps the `rcl_context_t` type.
    """

    def __init__(self):
        from rclpy.impl.implementation_singleton import rclpy_implementation
        from .handle import Handle
        self._handle = Handle(rclpy_implementation.rclpy_create_context())
        self._lock = threading.Lock()
        self._callbacks = []
        self._callbacks_lock = threading.Lock()
        self._logging_initialized = False

    @property
    def handle(self):
        return self._handle

    def init(self, args: Optional[List[str]] = None, *, initialize_logging: bool = True):
        """
        Initialize ROS communications for a given context.

        :param args: List of command line arguments.
        """
        # imported locally to avoid loading extensions on module import
        from rclpy.impl.implementation_singleton import rclpy_implementation
        global g_logging_ref_count
        with self._handle as capsule, self._lock:
            rclpy_implementation.rclpy_init(args if args is not None else sys.argv, capsule)
            if initialize_logging and not self._logging_initialized:
                with g_logging_configure_lock:
                    g_logging_ref_count += 1
                    if g_logging_ref_count == 1:
                        rclpy_implementation.rclpy_logging_configure(capsule)
                self._logging_initialized = True

    def ok(self):
        """Check if context hasn't been shut down."""
        # imported locally to avoid loading extensions on module import
        from rclpy.impl.implementation_singleton import rclpy_implementation
        with self._handle as capsule, self._lock:
            return rclpy_implementation.rclpy_ok(capsule)

    def _call_on_shutdown_callbacks(self):
        with self._callbacks_lock:
            for weak_method in self._callbacks:
                callback = weak_method()
                callback()
            self._callbacks = []

    def shutdown(self):
        """Shutdown this context."""
        # imported locally to avoid loading extensions on module import
        from rclpy.impl.implementation_singleton import rclpy_implementation
        with self._handle as capsule, self._lock:
            rclpy_implementation.rclpy_shutdown(capsule)
        self._call_on_shutdown_callbacks()
        self._logging_fini()

    def try_shutdown(self):
        """Shutdown this context, if not already shutdown."""
        # imported locally to avoid loading extensions on module import
        from rclpy.impl.implementation_singleton import rclpy_implementation
        with self._handle as capsule, self._lock:
            if rclpy_implementation.rclpy_ok(capsule):
                rclpy_implementation.rclpy_shutdown(capsule)
                self._call_on_shutdown_callbacks()

    def _remove_callback(self, weak_method):
        self._callbacks.remove(weak_method)

    def on_shutdown(self, callback: Callable[[], None]):
        """Add a callback to be called on shutdown."""
        if not callable(callback):
            raise TypeError('callback should be a callable, got {}', type(callback))
        with self._callbacks_lock:
            if not self.ok():
                callback()
            else:
                self._callbacks.append(weakref.WeakMethod(callback, self._remove_callback))

    def _logging_fini(self):
        from rclpy.impl.implementation_singleton import rclpy_implementation
        global g_logging_ref_count
        with self._lock:
            if self._logging_initialized:
                with g_logging_configure_lock:
                    g_logging_ref_count -= 1
                    if g_logging_ref_count == 0:
                        rclpy_implementation.rclpy_logging_fini()
                    if g_logging_ref_count < 0:
                        raise RuntimeError(
                            'Unexpected error: logger ref count should never be lower that zero')
                self._logging_initialized = False
