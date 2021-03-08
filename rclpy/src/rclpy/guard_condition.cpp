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

// Include pybind11 before rclpy_common/handle.h includes Python.h
#include <pybind11/pybind11.h>

#include <rcl/error_handling.h>
#include <rcl/guard_condition.h>
#include <rcl/rcl.h>
#include <rcl/types.h>

#include <memory>
#include <stdexcept>

#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "guard_condition.hpp"

namespace rclpy
{
/// Handle destructor for guard condition
static void
_rclpy_destroy_guard_condition(void * p)
{
  auto gc = static_cast<rcl_guard_condition_t *>(p);
  if (!gc) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_guard_condition got NULL pointer");
    return;
  }

  rcl_ret_t ret = rcl_guard_condition_fini(gc);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini guard condition: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(gc);
}

py::capsule
guard_condition_create(py::capsule pycontext)
{
  auto context = static_cast<rcl_context_t *>(
    rclpy_handle_get_pointer_from_capsule(pycontext.ptr(), "rcl_context_t"));
  if (!context) {
    throw py::error_already_set();
  }

  // Use smart pointer to make sure memory is free'd on error
  auto deleter = [](rcl_guard_condition_t * ptr) {_rclpy_destroy_guard_condition(ptr);};
  auto gc = std::unique_ptr<rcl_guard_condition_t, decltype(deleter)>(
    static_cast<rcl_guard_condition_t *>(PyMem_Malloc(sizeof(rcl_guard_condition_t))),
    deleter);
  if (!gc) {
    throw std::bad_alloc();
  }

  *gc = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t gc_options = rcl_guard_condition_get_default_options();

  rcl_ret_t ret = rcl_guard_condition_init(gc.get(), context, gc_options);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to create guard_condition");
  }

  PyObject * pygc_c =
    rclpy_create_handle_capsule(gc.get(), "rcl_guard_condition_t", _rclpy_destroy_guard_condition);
  if (!pygc_c) {
    throw py::error_already_set();
  }
  auto pygc = py::reinterpret_steal<py::capsule>(pygc_c);
  // pygc now owns the rcl_guard_contition_t
  gc.release();

  auto gc_handle = static_cast<rclpy_handle_t *>(pygc);
  auto context_handle = static_cast<rclpy_handle_t *>(pycontext);
  _rclpy_handle_add_dependency(gc_handle, context_handle);
  if (PyErr_Occurred()) {
    _rclpy_handle_dec_ref(gc_handle);
    throw py::error_already_set();
  }

  return pygc;
}

void
guard_condition_trigger(py::capsule pygc)
{
  auto gc = static_cast<rcl_guard_condition_t *>(
    rclpy_handle_get_pointer_from_capsule(pygc.ptr(), "rcl_guard_condition_t"));
  if (!gc) {
    throw py::error_already_set();
  }

  rcl_ret_t ret = rcl_trigger_guard_condition(gc);

  if (ret != RCL_RET_OK) {
    throw RCLError("failed to trigger guard condition");
  }
}
}  // namespace rclpy
