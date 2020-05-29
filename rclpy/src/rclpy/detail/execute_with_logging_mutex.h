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

#ifndef RCLPY__DETAIL__EXECUTE_WITH_LOGGING_MUTEX_H_
#define RCLPY__DETAIL__EXECUTE_WITH_LOGGING_MUTEX_H_

// Must be before Python.h; makes #-formats use ssize_t instead of int
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "rcutils/logging.h"
#include "rcutils/visibility_control_macros.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef PyObject * (*rclpy_detail_execute_with_logging_mutex_sig)(PyObject *, PyObject *);

/// Execute the given function pointer after acquiring the logging mutex.
RCUTILS_LOCAL
PyObject *
rclpy_detail_execute_with_logging_mutex(
  rclpy_detail_execute_with_logging_mutex_sig function,
  PyObject * self,
  PyObject * args);

typedef void (*rclpy_detail_execute_with_logging_mutex_sig2)(void *);

/// Execute the given function pointer after acquiring the logging mutex.
/**
 * This version has a slightly different signature which is used when destroying a node.
 */
RCUTILS_LOCAL
void
rclpy_detail_execute_with_logging_mutex2(
  rclpy_detail_execute_with_logging_mutex_sig2 function,
  void * arg);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // RCLPY__DETAIL__EXECUTE_WITH_LOGGING_MUTEX_H_