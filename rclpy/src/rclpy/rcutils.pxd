# Copyright 2021 Open Source Robotics Foundation, Inc.
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

cdef extern from "<rcutils/types/rcutils_ret.h>":
    ctypedef int rcutils_ret_t
    int RCUTILS_RET_OK
    int RCUTILS_RET_WARN
    int RCUTILS_RET_ERROR
    int RCUTILS_RET_BAD_ALLOC
    int RCUTILS_RET_INVALID_ARGUMENT
    int RCUTILS_RET_NOT_ENOUGH_SPACE
    int RCUTILS_RET_NOT_INITIALIZED
    int RCUTILS_RET_NOT_FOUND
    int RCUTILS_RET_STRING_MAP_ALREADY_INIT
    int RCUTILS_RET_STRING_MAP_INVALID
    int RCUTILS_RET_STRING_KEY_NOT_FOUND
    int RCUTILS_RET_LOGGING_SEVERITY_MAP_INVALID
    int RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID
    int RCUTILS_RET_HASH_MAP_NO_MORE_ENTRIES


cdef extern from "<rcutils/allocator.h>":
    ctypedef struct rcutils_allocator_t:
        void * (*allocate)(size_t size, void * state)
        void (* deallocate)(void * pointer, void * state)
        void * (*reallocate)(void * pointer, size_t size, void * state)
        void * (*zero_allocate)(size_t number_of_elements, size_t size_of_element, void * state)
        void * state

    rcutils_allocator_t rcutils_get_default_allocator()


cdef extern from "<rcutils/error_handling.h>":
    ctypedef struct rcutils_error_string_t:
        char * str

    void rcutils_reset_error()


cdef extern from "<rcutils/logging.h>":
    """
    /* This is literal C code inserted into the Python module */
    static void auto_init_rcutils_logging() { RCUTILS_LOGGING_AUTOINIT; }
    """
    void auto_init_rcutils_logging()

    cdef enum RCUTILS_LOG_SEVERITY:
        RCUTILS_LOG_SEVERITY_UNSET
        RCUTILS_LOG_SEVERITY_DEBUG
        RCUTILS_LOG_SEVERITY_INFO
        RCUTILS_LOG_SEVERITY_WARN
        RCUTILS_LOG_SEVERITY_ERROR
        RCUTILS_LOG_SEVERITY_FATAL

    ctypedef struct rcutils_log_location_t:
        char * function_name
        char * file_name
        unsigned long long line_number

    rcutils_ret_t rcutils_logging_initialize()
    rcutils_ret_t rcutils_logging_shutdown()
    rcutils_ret_t rcutils_logging_set_logger_level(const char * name, int level)
    int rcutils_logging_get_logger_effective_level(const char * name)
    int rcutils_logging_logger_is_enabled_for(const char * name, int severity)
    void rcutils_log( const rcutils_log_location_t * location, int severity, const char * name, const char * format, ...)
    rcutils_ret_t rcutils_logging_severity_level_from_string( char * severity_string, rcutils_allocator_t allocator, int * severity)
