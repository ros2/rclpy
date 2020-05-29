// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "./execute_with_logging_mutex.h"

// Must be before Python.h; makes #-formats use ssize_t instead of int
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "rcutils/logging.h"
#include "rcutils/visibility_control_macros.h"

#include "./logging_mutex.hpp"

extern "C"
{

PyObject *
rclpy_detail_execute_with_logging_mutex(
  rclpy_detail_execute_with_logging_mutex_sig function,
  PyObject * self,
  PyObject * args)
{
  try {
    std::shared_ptr<std::recursive_mutex> mutex = rclpy::detail::get_global_logging_mutex();
    std::lock_guard<std::recursive_mutex> guard(*mutex);
    return function(self, args);
  } catch (std::exception & ex) {
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to acquire logging mutex: %s", ex.what());
    return NULL;
  } catch (...) {
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to acquire logging mutex");
    return NULL;
  }
}

void
rclpy_detail_execute_with_logging_mutex2(
  rclpy_detail_execute_with_logging_mutex_sig2 function,
  void * arg)
{
  try {
    std::shared_ptr<std::recursive_mutex> mutex = rclpy::detail::get_global_logging_mutex();
    std::lock_guard<std::recursive_mutex> guard(*mutex);
    function(arg);
  } catch (std::exception & ex) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level,
      "Failed to acquire logging mutex: %s", ex.what());
  } catch (...) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level,
      "Failed to acquire logging mutex");
  }
}

}  // extern "C"
