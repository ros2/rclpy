// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "src/rclpy/sigint_gc.h"

#include <Python.h>

#include <rcl/error_handling.h>

rcl_guard_condition_t * g_rclpy_sigint_gc_handle;

/// Catch signals
void rclpy_catch_function(int signo)
{
  (void) signo;
  rcl_ret_t ret = rcl_trigger_guard_condition(g_rclpy_sigint_gc_handle);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to trigger guard_condition: %s", rcl_get_error_string_safe());
    rcl_reset_error();
  }
}
