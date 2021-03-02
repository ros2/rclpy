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

#include <rcl/context.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/types.h>

#include <stdexcept>

#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "context.hpp"

namespace rclpy
{
size_t
context_get_domain_id(py::capsule pycontext)
{
  auto context = static_cast<rcl_context_t *>(
    rclpy_handle_get_pointer_from_capsule(pycontext.ptr(), "rcl_context_t"));
  if (!context) {
    throw py::error_already_set();
  }

  size_t domain_id;
  rcl_ret_t ret = rcl_context_get_domain_id(context, &domain_id);
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to get domain id");
  }

  return domain_id;
}

void
_rclpy_context_handle_destructor(void * p)
{
  auto context = static_cast<rcl_context_t *>(p);
  if (!context) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_context_handle_destructor failed to get pointer");
    return;
  }
  if (NULL != context->impl) {
    rcl_ret_t ret;
    if (rcl_context_is_valid(context)) {
      // shutdown first, if still valid
      ret = rcl_shutdown(context);
      if (RCL_RET_OK != ret) {
        fprintf(
          stderr,
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "failed to shutdown rcl_context_t (%d) during PyCapsule destructor: %s\n",
          ret,
          rcl_get_error_string().str);
        rcl_reset_error();
      }
    }
    ret = rcl_context_fini(context);
    if (RCL_RET_OK != ret) {
      fprintf(
        stderr,
        "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
        "failed to fini rcl_context_t (%d) during PyCapsule destructor: %s\n",
        ret,
        rcl_get_error_string().str);
      rcl_reset_error();
    }
  }
  PyMem_FREE(context);
}

py::capsule
create_context()
{
  auto context = static_cast<rcl_context_t *>(PyMem_Malloc(sizeof(rcl_context_t)));
  if (!context) {
    throw std::bad_alloc();
  }
  *context = rcl_get_zero_initialized_context();
  PyObject * capsule = rclpy_create_handle_capsule(
    context, "rcl_context_t", _rclpy_context_handle_destructor);
  if (!capsule) {
    throw py::error_already_set();
  }
  return py::reinterpret_steal<py::capsule>(capsule);
}

/// Status of the the client library
/**
 * \return True if rcl is running properly, False otherwise
 */
bool
context_is_valid(py::capsule pycontext)
{
  auto context = static_cast<rcl_context_t *>(rclpy_handle_get_pointer_from_capsule(
      pycontext.ptr(), "rcl_context_t"));
  if (!context) {
    throw py::error_already_set();
  }

  return rcl_context_is_valid(context);
}
}  // namespace rclpy
