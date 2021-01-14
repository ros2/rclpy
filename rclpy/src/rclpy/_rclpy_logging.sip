// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

%Module(name=_rclpy_logging, language="C", use_argument_names=true)

%DefaultEncoding "UTF-8"

%ModuleCode
#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>
#include <rcutils/types/rcutils_ret.h>
#include <rcutils/logging.h>

#include <rcl_logging_interface/rcl_logging_interface.h>
%End


void rclpy_logging_initialize();
%MethodCode
  (void) sipSelf;
  rcutils_ret_t ret = rcutils_logging_initialize();
  if (ret != RCUTILS_RET_OK) {
    sipIsErr = 1;
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to initialize logging system, return code: %d", ret);
    rcutils_reset_error();
  }
%End


void rclpy_logging_shutdown();
%MethodCode
  (void) sipSelf;
  rcutils_ret_t ret = rcutils_logging_shutdown();
  if (ret != RCUTILS_RET_OK) {
    sipIsErr = 1;
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to shutdown logging system, return code: %d", ret);
    rcutils_reset_error();
  }
%End


void rclpy_logging_set_logger_level(const char * name, int level);
%MethodCode
  (void) sipSelf;
  rcutils_ret_t ret = rcutils_logging_set_logger_level(name, level);
  if (ret != RCUTILS_RET_OK) {
    sipIsErr = 1;
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to set level \"%d\" for logger \"%s\", return code: %d",
      level, name, ret);
    rcutils_reset_error();
  }
%End


unsigned long long rclpy_logging_get_logger_effective_level(const char * name);
%MethodCode
  (void) sipSelf;
  int logger_level = rcutils_logging_get_logger_effective_level(name);

  if (logger_level < 0) {
    sipIsErr = 1;
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to get effective level for logger \"%s\", return code: %d",
      name, logger_level);
    rcutils_reset_error();
  } else {
    sipRes = logger_level;
  }
%End


int rclpy_logging_logger_is_enabled_for(const char * name, int severity);
%MethodCode
  (void) sipSelf;
  sipRes = rcutils_logging_logger_is_enabled_for(name, severity);
%End


void rclpy_logging_rcutils_log(
  int severity,
  const char * name,
  const char * message,
  const char * function_name,
  const char * file_name,
  unsigned long long line_number);
%MethodCode
  (void) sipSelf;
  RCUTILS_LOGGING_AUTOINIT;
  rcutils_log_location_t logging_location = {function_name, file_name, line_number};
  rcutils_log(&logging_location, severity, name, "%s", message);
%End


int rclpy_logging_severity_level_from_string(const char *log_level);
%MethodCode
  (void) sipSelf;
  int severity;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcutils_ret_t ret = rcutils_logging_severity_level_from_string(log_level, allocator, &severity);
  if (ret != RCUTILS_RET_OK) {
    sipIsErr = 1;
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to get log severity from name \"%s\", return code: %d",
      log_level, ret);
    rcutils_reset_error();
  } else {
    sipRes = severity;
  }
%End


int rclpy_get_unset_logging_severity();
%MethodCode
  (void) sipSelf;
  sipRes = RCUTILS_LOG_SEVERITY_UNSET;
%End

int rclpy_get_debug_logging_severity();
%MethodCode
  (void) sipSelf;
  sipRes = RCUTILS_LOG_SEVERITY_DEBUG;
%End

int rclpy_get_info_logging_severity();
%MethodCode
  (void) sipSelf;
  sipRes = RCUTILS_LOG_SEVERITY_INFO;
%End

int rclpy_get_warn_logging_severity();
%MethodCode
  (void) sipSelf;
  sipRes = RCUTILS_LOG_SEVERITY_WARN;
%End

int rclpy_get_error_logging_severity();
%MethodCode
  (void) sipSelf;
  sipRes = RCUTILS_LOG_SEVERITY_ERROR;
%End

int rclpy_get_fatal_logging_severity();
%MethodCode
  (void) sipSelf;
  sipRes = RCUTILS_LOG_SEVERITY_FATAL;
%End


// TODO(sloretz) this method leads to a warning of type -Wcast-function-type - SIP bug?
PyObject * rclpy_logging_get_logging_directory() /NoArgParser/;
%MethodCode
  (void) sipSelf;
  (void) sipArgs;
  (void) sipKwds;
  char * log_dir = NULL;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_logging_ret_t ret = rcl_logging_get_logging_directory(allocator, &log_dir);
  if (RCL_LOGGING_RET_OK != ret) {
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to get current logging directory, error: \"%s\", return code: \"%d\"",
      rcutils_get_error_string().str, ret);
    rcutils_reset_error();
    // Not using sipIsErr or sipRes because /NoArgParser/
    return NULL;
  }
  // Not using sipRes because /NoArgParser/
  PyObject * py_log_dir = PyUnicode_DecodeFSDefault(log_dir);
  allocator.deallocate(log_dir, allocator.state);
  return py_log_dir;
%End
