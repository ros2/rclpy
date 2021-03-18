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
#include <pybind11/stl.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/types.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "init.hpp"

namespace rclpy
{
struct InitOptions
{
  explicit InitOptions(rcl_allocator_t allocator)
  {
    rcl_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = rcl_init_options_init(&rcl_options, allocator);
    if (RCL_RET_OK != ret) {
      throw RCLError("Failed to initialize init options");
    }
  }

  ~InitOptions()
  {
    rcl_ret_t ret = rcl_init_options_fini(&rcl_options);
    if (RCL_RET_OK != ret) {
      fprintf(
        stderr,
        "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
        "failed to fini rcl_init_options_t (%d) in destructor: %s\n",
        ret,
        rcl_get_error_string().str);
      rcl_reset_error();
    }
  }

  rcl_init_options_t rcl_options;
};

void
init(py::list pyargs, py::capsule pycontext, size_t domain_id)
{
  auto context = static_cast<rcl_context_t *>(
    rclpy_handle_get_pointer_from_capsule(pycontext.ptr(), "rcl_context_t"));
  if (!context) {
    throw py::error_already_set();
  }

  // turn the arguments into an array of C-style strings
  std::vector<const char *> arg_c_values(pyargs.size());
  for (size_t i = 0; i < pyargs.size(); ++i) {
    // CPython owns const char * memory - no need to free it
    arg_c_values[i] = PyUnicode_AsUTF8(pyargs[i].ptr());
    if (!arg_c_values[i]) {
      throw py::error_already_set();
    }
  }

  InitOptions init_options(rcl_get_default_allocator());

  // Set domain id
  rcl_ret_t ret = rcl_init_options_set_domain_id(&init_options.rcl_options, domain_id);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to set domain id to init options");
  }

  ret = rcl_init(arg_c_values.size(), &(arg_c_values[0]), &init_options.rcl_options, context);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to initialize rcl");
  }

  throw_if_unparsed_ros_args(pyargs, context->global_arguments);
}

void
shutdown(py::capsule pycontext)
{
  auto context = static_cast<rcl_context_t *>(
    rclpy_handle_get_pointer_from_capsule(pycontext.ptr(), "rcl_context_t"));
  if (!context) {
    throw py::error_already_set();
  }

  rcl_ret_t ret = rcl_shutdown(context);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to shutdown");
  }
}
}  // namespace rclpy
