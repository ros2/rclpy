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

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executor_handle import ExecutorHandle
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class GuardCondition:

    def __init__(self, callback, callback_group, guard_handle=None, guard_pointer=None):
        if guard_handle is None:
            guard_handle, guard_pointer = _rclpy.rclpy_create_guard_condition()
        self.guard_handle = guard_handle
        self.guard_pointer = guard_pointer
        self.callback = callback
        self.callback_group = callback_group
        # Holds info the executor uses to do work for this entity
        self._executor_handle = ExecutorHandle(
            self._take, self._execute, cancel_ready_callback=self._cancel_ready)

    def _take(self):
        pass

    def _execute(self, _):
        return self.callback()

    def _cancel_ready(self):
        # guard conditions are auto-taken when they come up in the wait-list, so retrigger
        self.trigger()

    def trigger(self):
        _rclpy.rclpy_trigger_guard_condition(self.guard_handle)


class NodeGraphGuardCondition(GuardCondition):

    def __init__(self, node_handle):
        gc_handle, gc_pointer = _rclpy.rclpy_get_graph_guard_condition()
        super().__init__(
            self._multiplex_callbacks, MutuallyExclusiveCallbackGroup(), guard_handle=gc_handle,
            guard_pointer=gc_pointer)
        self._callbacks = []

    def _multiplex_callbacks(self):
        for callback in self._callbacks:
            callback()

    def add_callback(self, callback):
        """Add a callback to be executed when a node graph event occurrs."""
        self._callbacks.append(callback)

    def remove_callback(self, callback):
        """Stop executing a given callback when a node graph event occurrs."""
        self._callbacks.remove(callback)

    def has_callbacks(self):
        """Return True if their are callbacks to be executed on graph events."""
        return bool(self._callbacks)
