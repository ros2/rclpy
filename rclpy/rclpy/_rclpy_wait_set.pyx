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

from cpython.pycapsule cimport PyCapsule_GetPointer
from cpython.pycapsule cimport PyCapsule_IsValid
from rcl cimport *


def raise_runtime_error():
    raw_error = rcl_get_error_string_safe()
    try:
        error = 'Failed to wait on waitset: "%s"' % raw_error.decode('UTF-8', 'strict')
    finally:
        rcl_reset_error()


cdef class WaitSet:
    # List of PyCapsule containing rcl_subscription_t
    cdef public list _pysubs
    # List of PyCapsule containing rcl_guard_condition_t
    cdef public list _pygcs
    # List of PyCapsule containing rcl_timer_t
    cdef public list _pytmrs
    # List of PyCapsule containing rcl_client_t
    cdef public list _pyclis
    # List of PyCapsule containing rcl_service_t
    cdef public list _pysrvs

    def __init__(self):
        self._pysubs = []
        self._pygcs = []
        self._pytmrs = []
        self._pyclis = []
        self._pysrvs = []

    def add_subscription(self, entity):
        self._pysubs.append(entity.subscription_handle)

    def add_guard_condition(self, entity):
        self._pygcs.append(entity.guard_handle)

    def add_timer(self, entity):
        self._pytmrs.append(entity.timer_handle)

    def add_client(self, entity):
        self._pyclis.append(entity.client_handle)

    def add_service(self, entity):
        self._pysrvs.append(entity.service_handle)

    def wait(self, timeout=-1):
        cdef rcl_ret_t ret
        cdef bytes raw_error
        cdef size_t ci
        cdef void * entity_pointer
        # coerce timeout from python type while GIL is held
        cdef int ctimeout = timeout

        cdef rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set()
        try:
            # Initialize the wait set
            ret = rcl_wait_set_init(
                &wait_set, len(self._pysubs), len(self._pygcs), len(self._pytmrs),
                len(self._pyclis), len(self._pysrvs), rcl_get_default_allocator())
            if ret != RCL_RET_OK:
                raise_runtime_error()

            # Populate wait set
            for i, capsule in enumerate(self._pysubs):
                wait_set.subscriptions[i] = PyCapsule_GetPointer(capsule, 'rcl_subscription_t')
            for i, capsule in enumerate(self._pygcs):
                wait_set.guard_conditions[i] = PyCapsule_GetPointer(
                    capsule, 'rcl_guard_condition_t')
            for i, capsule in enumerate(self._pytimers):
                wait_set.timers[i] = PyCapsule_GetPointer(capsule, 'rcl_timer_t')
            for i, capsule in enumerate(self._pyclis):
                wait_set.clients[i] = PyCapsule_GetPointer(capsule, 'rcl_client_t')
            for i, capsule in enumerate(self._pysrvs):
                wait_set.services[i] = PyCapsule_GetPointer(capsule, 'rcl_service_t')

            # Actually wait
            with nogil:
                ret = rcl_wait(&wait_set, ctimeout);
            if ret != RCL_RET_OK or ret != RCL_RET_TIMEOUT:
                raise_runtime_error()

            # Make ready entities available to python code
            ready_subscriptions = []
            ready_guard_conditions = []
            ready_timers = []
            ready_clients = []
            ready_services = []
            if ret != RCL_RET_TIMEOUT:
                for entity in self._pysubs:
                    ci = 0
                    while ci < wait_set.size_of_subscriptions:
                        entity_pointer = PyCapsule_GetPointer(capsule, 'rcl_subscription_t')
                        if wait_set.subscriptions[i] == entity_pointer:
                            self.ready_subscriptions.append(entity)
                        ci += 1
                for entity in self._pygcs:
                    ci = 0
                    while ci < wait_set.size_of_guard_conditions:
                        entity_pointer = PyCapsule_GetPointer(capsule, 'rcl_guard_condition_t')
                        if wait_set.guard_conditions[i] == entity_pointer:
                            self.ready_guard_conditions.append(entity)
                        ci += 1
                for entity in self._pytmrs:
                    ci = 0
                    while ci < wait_set.size_of_timers:
                        entity_pointer = PyCapsule_GetPointer(capsule, 'rcl_timer_t')
                        if wait_set.timers[i] == entity_pointer:
                            self.ready_timers.append(entity)
                        ci += 1
                for entity in self._pyclis:
                    ci = 0
                    while ci < wait_set.size_of_clients:
                        entity_pointer = PyCapsule_GetPointer(capsule, 'rcl_client_t')
                        if wait_set.clients[i] == entity_pointer:
                            self.ready_clients.append(entity)
                        ci += 1
                for entity in self._pysrvs:
                    ci = 0
                    while ci < wait_set.size_of_services:
                        entity_pointer = PyCapsule_GetPointer(capsule, 'rcl_service_t')
                        if wait_set.services[i] == entity_pointer:
                            self.ready_services.append(entity)
                        ci += 1
            self._pysubs = ready_subscriptions
            self._pygcs = ready_guard_conditions
            self._pytmrs = ready_timers
            self._pyclis = ready_clients
            self._pysrvs = ready_services
        finally:
            ret = rcl_wait_set_fini(&(wait_set))
            if ret != RCL_RET_OK:
                raise_runtime_error()

    def ready_subscriptions(self):
        return self._pysubs

    def ready_guard_conditions(self):
        return self._pygcs

    def ready_timers(self):
        return self._pytmrs

    def ready_clients(self):
        return self._pyclis

    def ready_services(self):
        return self._pysrvs
