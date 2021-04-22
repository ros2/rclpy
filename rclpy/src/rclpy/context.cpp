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

#include <memory>

#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "context.hpp"

namespace rclpy
{
Context::Context()
{
  rcl_context_ = std::shared_ptr<rcl_context_t>(
    new rcl_context_t,
    [](rcl_context_t * context)
    {
      if (NULL != context->impl) {
        rcl_ret_t ret;
        if (rcl_context_is_valid(context)) {
          // shutdown first, if still valid
          ret = rcl_shutdown(context);
          if (RCL_RET_OK != ret) {
            // Warning should use line number of the current stack frame
            int stack_level = 1;
            PyErr_WarnFormat(
              PyExc_RuntimeWarning, stack_level,
              "[rclpy| %s : %s ]: failed to shutdown rcl_context_t: %s",
              RCUTILS_STRINGIFY(__FILE__), RCUTILS_STRINGIFY(__LINE__), rcl_get_error_string().str);
            rcl_reset_error();
          }
        }
        ret = rcl_context_fini(context);
        if (RCL_RET_OK != ret) {
          // Warning should use line number of the current stack frame
          int stack_level = 1;
          PyErr_WarnFormat(
            PyExc_RuntimeWarning, stack_level,
            "[rclpy| %s : %s ]: failed to fini rcl_context_t: %s",
            RCUTILS_STRINGIFY(__FILE__), RCUTILS_STRINGIFY(__LINE__), rcl_get_error_string().str);
          rcl_reset_error();
        }
      }
      delete context;
    });
  *rcl_context_ = rcl_get_zero_initialized_context();
}

void
Context::destroy()
{
  rcl_context_.reset();
}

size_t
Context::get_domain_id()
{
  size_t domain_id;
  rcl_ret_t ret = rcl_context_get_domain_id(rcl_context_.get(), &domain_id);
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to get domain id");
  }

  return domain_id;
}

bool
Context::ok()
{
  return rcl_context_is_valid(rcl_context_.get());
}

void
Context::shutdown()
{
  rcl_ret_t ret = rcl_shutdown(rcl_context_.get());
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to shutdown");
  }
}

void define_context(py::object module)
{
  py::class_<Context, Destroyable, std::shared_ptr<Context>>(module, "Context")
  .def(py::init<>())
  .def_property_readonly(
    "pointer", [](const Context & context) {
      return reinterpret_cast<size_t>(context.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "get_domain_id", &Context::get_domain_id,
    "Retrieves domain id from init_options of context.")
  .def(
    "ok", &Context::ok,
    "Status of the the client library")
  .def(
    "shutdown", &Context::shutdown,
    "Shutdown context");
}

}  // namespace rclpy
