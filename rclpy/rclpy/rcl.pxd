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

cdef extern from "<rcl/rcl.h>":
    # Let cython know what comes from this header with roughly correct types
    ctypedef int rcl_ret_t
    int RCL_RET_OK
    int RCL_RET_TIMEOUT

    struct rcl_wait_set_t:
        int size_of_subscriptions
        int size_of_guard_conditions
        int size_of_timers
        int size_of_clients
        int size_of_services
        void ** subscriptions
        void ** guard_conditions
        void ** timers
        void ** clients
        void ** services

    struct rcl_allocator_t:
        pass

    rcl_allocator_t rcl_get_default_allocator() nogil;

    rcl_ret_t rcl_wait(rcl_wait_set_t *, int timeout) nogil
    rcl_ret_t rcl_wait_set_init(rcl_wait_set_t *, int, int, int, int, int, rcl_allocator_t) nogil;
    rcl_ret_t rcl_wait_set_fini(rcl_wait_set_t *) nogil
    rcl_wait_set_t rcl_get_zero_initialized_wait_set() nogil


cdef extern from "<rcl/error_handling.h>":
    const char * rcl_get_error_string_safe() nogil
    void rcl_reset_error() nogil
