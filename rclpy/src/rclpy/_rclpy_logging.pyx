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

# cython: language_level=3

cimport rcutils
cimport rcl_logging_interface


def rclpy_logging_initialize():
    ret = rcutils.rcutils_logging_initialize()
    if ret != rcutils.RCUTILS_RET_OK:
        rcutils.rcutils_reset_error()
        raise RuntimeError(f'Failed to initialize logging system: {ret}')


def rclpy_logging_shutdown():
    ret = rcutils.rcutils_logging_shutdown()
    if ret != rcutils.RCUTILS_RET_OK:
        rcutils.rcutils_reset_error()
        raise RuntimeError('Failed to shutdown logging system: {ret}')


def rclpy_logging_set_logger_level(pyname, int level):
    ret = rcutils.rcutils_logging_set_logger_level(pyname.encode('utf-8'), level)
    if ret != rcutils.RCUTILS_RET_OK:
        rcutils.rcutils_reset_error()
        raise RuntimeError(f'Failed to set level "{level}" for logger "{pyname}": {ret}')


def rclpy_logging_get_logger_effective_level(pyname):
    logger_level = rcutils.rcutils_logging_get_logger_effective_level(pyname.encode('utf-8'));
    if logger_level < 0:
        rcutils.rcutils_reset_error()
        raise RuntimeError(f'Failed to get effective level for logger "{pyname}": {logger_level}')
    return logger_level


def rclpy_logging_logger_is_enabled_for(pyname, severity):
    cdef int cseverity = severity
    return rcutils.rcutils_logging_logger_is_enabled_for(
        pyname.encode('utf-8'), cseverity)


def rclpy_logging_rcutils_log(int severity, name, message, function_name, file_name, line_number):
    cdef int cseverity = severity
    rcutils.auto_init_rcutils_logging()

    # Must keep reference to encoded bytes before assigning them
    py_encoded_function_name = function_name.encode('utf-8')
    py_encoded_file_name = file_name.encode('utf-8')
    py_encoded_message = message.encode('utf-8')

    cdef rcutils.rcutils_log_location_t logging_location
    logging_location.function_name = py_encoded_function_name
    logging_location.file_name = py_encoded_file_name
    logging_location.line_number = line_number

    # Must specify type for vaargs argument
    cdef char * cmessage = py_encoded_message
    rcutils.rcutils_log(&logging_location, severity, name.encode('utf-8'), '%s', cmessage)


def rclpy_logging_severity_level_from_string(log_level):
    py_encoded_log_level = log_level.encode('utf-8')
    cdef char * clog_level = py_encoded_log_level
    cdef int cseverity;
    allocator = rcutils.rcutils_get_default_allocator()
    ret = rcutils.rcutils_logging_severity_level_from_string(clog_level, allocator, &cseverity);
    if ret != rcutils.RCUTILS_RET_OK:
        rcutils.rcutils_reset_error()
        raise RuntimeError(f'Failed to get log serverity "{log_level}": {ret}')
    return cseverity


RCUTILS_LOG_SEVERITY_UNSET = rcutils.RCUTILS_LOG_SEVERITY_UNSET
RCUTILS_LOG_SEVERITY_DEBUG = rcutils.RCUTILS_LOG_SEVERITY_DEBUG
RCUTILS_LOG_SEVERITY_INFO = rcutils.RCUTILS_LOG_SEVERITY_INFO
RCUTILS_LOG_SEVERITY_WARN = rcutils.RCUTILS_LOG_SEVERITY_WARN
RCUTILS_LOG_SEVERITY_ERROR = rcutils.RCUTILS_LOG_SEVERITY_ERROR
RCUTILS_LOG_SEVERITY_FATAL = rcutils.RCUTILS_LOG_SEVERITY_FATAL


def rclpy_logging_get_logging_directory():
    allocator = rcutils.rcutils_get_default_allocator()
    cdef char * log_dir = NULL
    ret = rcl_logging_interface.rcl_logging_get_logging_directory(allocator, &log_dir)
    if ret != rcl_logging_interface.RCL_LOGGING_RET_OK:
        rcutils.rcutils_reset_error()
        raise RuntimeError(f'Failed to get current logging directory: {ret}')

    try:
        return log_dir.decode('utf-8')
    finally:
        allocator.deallocate(log_dir, allocator.state);
