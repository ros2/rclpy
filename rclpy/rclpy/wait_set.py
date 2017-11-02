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

import itertools

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class WaitSet:
    """Provide conveneint methods and destroy the wait set when garbage collected."""

    def __init__(self, subscriptions, guards, timers, clients, services):
        # List of entity pointers (the python integer, not the PyCapsule) that are ready
        self._ready_pointers = []
        self._subscriptions = [s.subscription_handle for s in subscriptions]
        self._guards = [g.guard_handle for g in guards]
        self._timers = [t.timer_handle for t in timers]
        self._clients = [c.client_handle for c in clients]
        self._services = [s.service_handle for s in services]

        self._wait_set = None

    def __enter__(self):
        self._wait_set = _rclpy.rclpy_get_zero_initialized_wait_set()
        _rclpy.rclpy_wait_set_init(
            self._wait_set,
            len(self._subscriptions),
            len(self._guards),
            len(self._timers),
            len(self._clients),
            len(self._services))
        return self

    def __exit__(self, t, v, tb):
        _rclpy.rclpy_destroy_wait_set(self._wait_set)

    def wait(self, timeout_nsec):
        # Populate wait set
        _rclpy.rclpy_wait_set_clear_entities(self._wait_set)
        entities = itertools.chain(
            self._subscriptions, self._guards, self._timers, self._clients, self._services)
        _rclpy.rclpy_wait_set_add_entities(self._wait_set, entities)

        # Wait
        _rclpy.rclpy_wait(self._wait_set, timeout_nsec)

        # Get results
        self._ready_pointers = _rclpy.rclpy_get_ready_entities(self._wait_set)

    def is_ready(self, entity_pointer):
        return entity_pointer in self._ready_pointers
