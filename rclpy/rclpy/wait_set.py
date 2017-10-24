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


class WaitSet:
    """Provide conveneint methods and destroy the wait set when garbage collected."""

    def __init__(self):
        # List of entity pointers (the python integer, not the PyCapsule) that are ready
        self._ready_pointers = []

        # maps pointers (integers) to handles (PyCapsule)
        self._subscriptions = {}
        self._guard_conditions = {}
        self._timers = {}
        self._clients = {}
        self._services = {}

        # Set when the wait set needs to be built or rebuilt
        self._needs_building = True
        self._wait_set = None
        # rcl_wait is not thread safe, so prevent multiple wait calls at once
        self._wait_lock = threading.Lock()

    def destroy(self):
        if self._wait_set is not None:
            _rclpy.rclpy_destroy_wait_set(self._wait_set)
            self._wait_set = None

    def __del__(self):
        self.destroy()

    def __enter__(self):
        return self

    def __exit__(self, t, v, tb):
        self.destroy()

    def add_subscription(self, subscription_handle, subscription_pointer):
        self._subscriptions[subscription_pointer] = subscription_handle
        self._needs_building = True

    def add_subscriptions(self, subscriptions):
        for sub in subscriptions:
            self.add_subscription(sub.subscription_handle, sub.subscription_pointer)

    def remove_subscription(self, subscription_pointer):
        del self._subscriptions[subscription_pointer]
        self._needs_building = True

    def add_guard_condition(self, gc_handle, gc_pointer):
        self._guard_conditions[gc_pointer] = gc_handle
        self._needs_building = True

    def add_guard_conditions(self, guards):
        for gc in guards:
            self.add_guard_condition(gc.guard_handle, gc.guard_pointer)

    def remove_guard_condition(self, gc_pointer):
        del self._guard_conditions[gc_pointer]
        self._needs_building = True

    def add_timer(self, timer_handle, timer_pointer):
        self._timers[timer_pointer] = timer_handle
        self._needs_building = True

    def add_timers(self, timers):
        for tmr in timers:
            self.add_timer(tmr.timer_handle, tmr.timer_pointer)

    def remove_timer(self, timer_pointer):
        del self._timers[timer_pointer]
        self._needs_building = True

    def add_client(self, client_handle, client_pointer):
        self._clients[client_pointer] = client_handle
        self._needs_building = True

    def add_clients(self, clients):
        for cli in clients:
            self.add_client(cli.client_handle, cli.client_pointer)

    def remove_client(self, client_pointer):
        del self._clients[client_pointer]
        self._needs_building = True

    def add_service(self, service_handle, service_pointer):
        self._services[service_pointer] = service_handle
        self._needs_building = True

    def add_services(self, services):
        for srv in services:
            self.add_service(srv.service_handle, srv.service_pointer)

    def remove_service(self, service_pointer):
        del self._services[service_pointer]
        self._needs_building = True

    def wait(self, timeout_nsec):
        with self._wait_lock:
            if self._needs_building:
                if self._wait_set is not None:
                    _rclpy.rclpy_destroy_wait_set(self._wait_set)

                self._wait_set = _rclpy.rclpy_get_zero_initialized_wait_set()
                _rclpy.rclpy_wait_set_init(
                    self._wait_set,
                    len(self._subscriptions),
                    len(self._guard_conditions),
                    len(self._timers),
                    len(self._clients),
                    len(self._services))

                _rclpy.rclpy_wait_set_clear_entities('subscription', self._wait_set)
                _rclpy.rclpy_wait_set_clear_entities('guard_condition', self._wait_set)
                _rclpy.rclpy_wait_set_clear_entities('timer', self._wait_set)
                _rclpy.rclpy_wait_set_clear_entities('client', self._wait_set)
                _rclpy.rclpy_wait_set_clear_entities('service', self._wait_set)

                for handle in self._subscriptions.values():
                    _rclpy.rclpy_wait_set_add_entity('subscription', self._wait_set, handle)
                for handle in self._guard_conditions.values():
                    _rclpy.rclpy_wait_set_add_entity('guard_condition', self._wait_set, handle)
                for handle in self._timers.values():
                    _rclpy.rclpy_wait_set_add_entity('timer', self._wait_set, handle)
                for handle in self._clients.values():
                    _rclpy.rclpy_wait_set_add_entity('client', self._wait_set, handle)
                for handle in self._services.values():
                    _rclpy.rclpy_wait_set_add_entity('service', self._wait_set, handle)

            _rclpy.rclpy_wait(self._wait_set, timeout_nsec)

            ws = self._wait_set
            self._ready_pointers = _rclpy.rclpy_get_ready_entities('subscription', ws)
            self._ready_pointers.extend(_rclpy.rclpy_get_ready_entities('guard_condition', ws))
            self._ready_pointers.extend(_rclpy.rclpy_get_ready_entities('timer', ws))
            self._ready_pointers.extend(_rclpy.rclpy_get_ready_entities('client', ws))
            self._ready_pointers.extend(_rclpy.rclpy_get_ready_entities('service', ws))

    def is_ready(self, entity_pointer):
        return entity_pointer in self._ready_pointers
