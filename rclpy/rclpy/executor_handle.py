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

import queue

import rclpy.future
from rclpy.utilities import timeout_sec_to_nsec


class ExecutorHandle:
    """Interface between an entity and an executor."""

    def __init__(self, take_callback, execute_callback, cancel_ready_callback=None):
        self._take_from_wait_list = take_callback
        self.execute_callback = execute_callback
        self._cancel_ready_callback = cancel_ready_callback
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._ready = False

    def notify_ready(self):
        """Receive notification from executor that this entity was ready in the wait list."""
        self._ready = True

    def cancel_ready(self):
        """Receive notification from executor that this entity could not be taken."""
        self._ready = False
        # Hook for guard conditions to retrigger themselves
        if self._cancel_ready_callback:
            self._cancel_ready_callback

    def ready(self):
        return self._ready

    def take_from_wait_list(self):
        """Get data from rcl."""
        args = self._take_from_wait_list()
        self._ready = False
        return args
