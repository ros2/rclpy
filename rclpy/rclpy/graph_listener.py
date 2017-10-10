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
from rclpy.wait_set import WaitSet as _WaitSet


class GraphListenerSingleton:
    """
    Manage a thread to listen for graph events.

    This class manages a single thread to listen for graph event updates.
    Every node has a graph event guard condition associated with it.
    It is triggered when a publisher, service, etc either becomes or stops being available.
    These guard conditions could be waited on by the executor, but that would mean deadlock if
    all executor threads are waiting for graph events (e.g. by calling wait_for_service() in a
    subscriber callback).
    This listener enables executor threads to be blocked for graph events without deadlock.
    """

    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, "__singleton"):
            setattr(cls, "__singleton", super().__new__(cls, *args, **kwargs))
        return getattr(cls, "__singleton")

    def __init__(self):
        # Maps node_handle to a list of subscriber threading.Condition
        self._subscriptions = {}
        self._gc, self._gc_handle = _rclpy.rclpy_create_guard_condition()
        self._thread = None
        self._lock = threading.Lock()

    def __del__(self):
        with self._lock:
            assert not self._subscriptions
            th = self._thread
        if th:
            th.join()
        _rclpy.rclpy_destroy_entity('guard_condition', self._gc)

    def add_node(self, node_handle):
        """
        Listen for graph updates on the given node.

        :param node_handle: rclpy node handle
        :type node_handle: PyCapsule
        :rtype: threading.Condition
        """
        with self._lock:
            if self._thread is None:
                self._thread = threading.Thread(target=self._runner)
                self._thread.daemon = True
                self._thread.start()

            condition = threading.Condition()

            if node_handle not in self._subscriptions:
                self._subscriptions[node_handle] = [condition]
                # signal thread to rebuild wait list
                _rclpy.rclpy_trigger_guard_condition(self._gc)
            else:
                self._subscriptions[node_handle].append(condition)

            return condition

    def remove_condition(self, node_handle, condition):
        """
        Stop listening for graph updates for the given node and condition.

        :param node_handle: rclpy node handle
        :type node_handle: PyCapsule
        :param condition: the condition object used by the subscriber
        :type condition: threading.Condition
        """
        with self._lock:
            assert node_handle in self._subscriptions

            condition_list = self._subscriptions[node_handle]
            condition_list.remove(condition)

            if not condition_list:
                # last subscriber for this node, remove the node
                del self._subscriptions[node_handle]

                # tell the thread to rebuild the wait set
                _rclpy.rclpy_trigger_guard_condition(self._gc)

    def _runner(self):
        while True:
            # Map guard condition handles to node_handle
            gc_handle_to_node_handle = {self._gc_handle: None}
            guard_conditions = [self._gc]
            with self._lock:
                if not self._subscriptions:
                    # nothing to wait on, kill the thread
                    self._thread = None
                    return
                for node_handle in self._subscriptions.keys():
                    gc, gc_handle = _rclpy.rclpy_get_graph_guard_condition(node_handle)
                    gc_handle_to_node_handle[gc_handle] = node_handle
                    guard_conditions.append(gc)

            # Build a wait set
            guards_ready = []
            with _WaitSet() as wait_set:
                _rclpy.rclpy_wait_set_init(
                    wait_set,
                    0,
                    len(guard_conditions),
                    0,
                    0,
                    0)
                for gc in guard_conditions:
                    _rclpy.rclpy_wait_set_add_entity('guard_condition', wait_set, gc)

                # Wait forever
                _rclpy.rclpy_wait(wait_set, -1)
                guards_ready = _rclpy.rclpy_get_ready_entities('guard_condition', wait_set)

            # notify graph event subscribers
            with self._lock:
                for gc_handle in guards_ready:
                    node_handle = gc_handle_to_node_handle[gc_handle]
                    if node_handle in self._subscriptions:
                        for condition in self._subscriptions[node_handle]:
                            condition.acquire()
                            try:
                                condition.notify_all()
                            finally:
                                condition.release()


class GraphEventSubscription:
    """Manage a subscription to graph event updates."""

    def __init__(self, node_handle):
        self._listener = GraphListenerSingleton()
        self._node_handle = node_handle
        self._condition = None

    def __enter__(self):
        self._condition = self._listener.add_node(self._node_handle)
        return self

    def __exit__(self, t, v, tb):
        self._listener.remove_condition(self._node_handle, self._condition)

    def wait_for(self, predicate, timeout_sec=None):
        """
        Wait for either the predicate to return true, or timeout.

        The predicate is called at the beginning, and every time the graph is updated.

        :param predicate: a callable that returns a boolean
        :type predicate: callable
        :param timeout_sec: Seconds to wait. Block forever if None. Don't wait if <= 0
        :type timeout_sec: float or None
        :rtype: the returned value of the last call to the predicate
        """
        result = predicate()
        if result or (timeout_sec is not None and timeout_sec <= 0):
            return result

        self._condition.acquire()
        try:
            return self._condition.wait_for(predicate, timeout=timeout_sec)
        finally:
            self._condition.release()
