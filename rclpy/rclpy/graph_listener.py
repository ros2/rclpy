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
import traceback

from rclpy.constants import S_TO_NS
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
import rclpy.logging
from rclpy.timer import WallTimer as _WallTimer
from rclpy.utilities import ok as _ok
from rclpy.wait_set import WaitSet as _WaitSet


class GraphListenerSingleton:
    """Manage a thread to listen for graph events."""

    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, '__singleton'):
            setattr(cls, '__singleton', super().__new__(cls, *args, **kwargs))
        return getattr(cls, '__singleton')

    def __init__(self):
        # Maps guard_condition pointers to a list of subscriber callbacks
        self._callbacks = {}
        # Maps timer instances to timer callbacks
        self._timers = {}
        self._gc_handle, self._gc_pointer = _rclpy.rclpy_create_guard_condition()
        self._thread = None
        self._lock = threading.RLock()
        self._wait_set = _WaitSet()
        self._wait_set.add_guard_condition(self._gc_handle, self._gc_pointer)

    def __del__(self):
        self.destroy()

    @classmethod
    def destroy(cls):
        self = getattr(cls, '__singleton')
        if self is not None:
            with self._lock:
                setattr(cls, '__singleton', None)
                self._thread = None
            _rclpy.rclpy_destroy_entity(self._gc_handle)

    def _try_start_thread(self):
        # Assumes lock is already held
        if self._thread is None:
            self._thread = threading.Thread(target=self._runner, daemon=True)
            self._thread.start()

    def add_timer(self, timer_period_ns, callback):
        """
        Call callback when timer triggers.

        :param timer_period_ns: time until timer triggers in nanoseconds
        :type timer_period_ns: integer
        :param callback: called when the graph updates
        :type callback: callable
        :rtype: rclpy.timer.WallTimer
        """
        with self._lock:
            tmr = _WallTimer(callback, None, timer_period_ns)
            self._timers[tmr] = callback
            self._wait_set.add_timer(tmr.timer_handle, tmr.timer_pointer)

            self._try_start_thread()
            return tmr

    def remove_timer(self, timer):
        """
        Remove a timer from the wait set.

        :param timer: A timer returned from add_timer()
        :type timer: rclpy.timer.WallTimer instance
        """
        with self._lock:
            if timer in self._timers:
                del self._timers[timer]
                self._wait_set.remove_timer(timer.timer_pointer)

    def add_callback(self, node_handle, callback):
        """
        Call callback when node graph gets updates.

        :param node_handle: rclpy node handle
        :type node_handle: PyCapsule
        :param callback: called when the graph updates
        :type callback: callable
        """
        with self._lock:
            gc_handle, gc_pointer = _rclpy.rclpy_get_graph_guard_condition(node_handle)
            if gc_pointer not in self._callbacks:
                # new node, rebuild wait set
                self._callbacks[gc_pointer] = []
                self._wait_set.add_guard_condition(gc_handle, gc_pointer)
                _rclpy.rclpy_trigger_guard_condition(self._gc_handle)

            # Add a callback
            self._callbacks[gc_pointer].append(callback)

            self._try_start_thread()
            # start the thread if necessary
            if self._thread is None:
                self._thread = threading.Thread(target=self._runner)
                self._thread.daemon = True
                self._thread.start()

    def remove_callback(self, node_handle, callback):
        """
        Stop listening for graph updates for the given node and callback.

        :param node_handle: rclpy node handle
        :type node_handle: PyCapsule
        :param callback: called when the graph updates
        :type callback: callable
        """
        with self._lock:
            gc_handle, gc_pointer = _rclpy.rclpy_get_graph_guard_condition(node_handle)
            if gc_pointer in self._callbacks:
                # Remove the callback
                self._callbacks[gc_pointer].remove(callback)

                if not self._callbacks[gc_pointer]:
                    # last subscriber for this node, remove the node and rebuild the wait set
                    del self._callbacks[gc_pointer]
                    self._wait_set.remove_guard_condition(gc_pointer)
                    _rclpy.rclpy_trigger_guard_condition(self._gc_handle)

    def _runner(self):
        while True:
            # Wait 1 second
            self._wait_set.wait(S_TO_NS)

            with self._lock:
                # Shutdown if necessary
                if not _ok():
                    self.destroy()
                    break

                # notify graph event subscribers
                if not self._thread:
                    # Asked to shut down thread
                    return
                ready_callbacks = []
                # Guard conditions
                for gc_pointer, callback_list in self._callbacks.items():
                    if self._wait_set.is_ready(gc_pointer):
                        for callback in callback_list:
                            ready_callbacks.append(callback)
                # Timers
                for tmr, callback in self._timers.items():
                    if self._wait_set.is_ready(tmr.timer_pointer):
                        ready_callbacks.append(callback)
                        _rclpy.rclpy_call_timer(tmr.timer_handle)
                # Call callbacks
                for callback in ready_callbacks:
                    try:
                        callback()
                    except Exception:
                        rclpy.logging.logwarn(traceback.format_exc())


class GraphEventSubscription:
    """Manage subscription to node graph updates."""

    def __init__(self, node_handle, callback, timeout_ns=-1, timeout_callback=None):
        self._listener = GraphListenerSingleton()
        self._node_handle = node_handle
        self._callback = callback
        self._listener.add_callback(self._node_handle, self._callback)
        self._timeout_callback = timeout_callback
        self._timer = None
        if timeout_ns >= 0:
            self._timer = self._listener.add_timer(timeout_ns, self.on_timeout)

    def on_timeout(self):
        self._timeout_callback()
        self._unsubscribe()

    def _unsubscribe(self):
        if self._callback:
            self._listener.remove_callback(self._node_handle, self._callback)
            self._callback = None
        if self._timer:
            self._listener.remove_timer(self._timer)
            self._timeout_callback = None
            self._timer = None

    def __del__(self):
        self._unsubscribe()

    def __enter__(self):
        return self

    def __exit__(self, t, v, tb):
        self._unsubscribe()
