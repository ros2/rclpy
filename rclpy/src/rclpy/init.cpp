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

#include <limits>
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
      RCUTILS_SAFE_FWRITE_TO_STDERR(
        "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
        "failed to fini rcl_init_options_t in destructor:");
      RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
      RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
      rcl_reset_error();
    }
  }

  rcl_init_options_t rcl_options;
};

void
init(py::list pyargs, Context & context, size_t domain_id)
{
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

  if (arg_c_values.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::range_error("Too many cli arguments");
  }
  int argc = static_cast<int>(arg_c_values.size());
  const char ** argv = argc > 0 ? &(arg_c_values[0]) : nullptr;
  ret = rcl_init(argc, argv, &init_options.rcl_options, context.rcl_ptr());
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to initialize rcl");
  }

  throw_if_unparsed_ros_args(pyargs, context.rcl_ptr()->global_arguments);
}

void
throw_if_unparsed_ros_args(py::list pyargs, const rcl_arguments_t & rcl_args)
{
  int unparsed_ros_args_count = rcl_arguments_get_count_unparsed_ros(&rcl_args);

  if (unparsed_ros_args_count < 0) {
    throw std::runtime_error("failed to count unparsed arguments");
  }
  if (0 == unparsed_ros_args_count) {
    return;
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();

  int * unparsed_indices_c = nullptr;
  rcl_ret_t ret = rcl_arguments_get_unparsed_ros(&rcl_args, allocator, &unparsed_indices_c);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to get unparsed arguments");
  }

  auto deallocator = [&](int ptr[]) {allocator.deallocate(ptr, allocator.state);};
  auto unparsed_indices = std::unique_ptr<int[], decltype(deallocator)>(
    unparsed_indices_c, deallocator);

  py::list unparsed_args;
  for (int i = 0; i < unparsed_ros_args_count; ++i) {
    int index = unparsed_indices_c[i];
    if (index < 0 || static_cast<size_t>(index) >= pyargs.size()) {
      throw std::runtime_error("got invalid unparsed ROS arg index");
    }
    unparsed_args.append(pyargs[index]);
  }

  throw UnknownROSArgsError(static_cast<std::string>(py::repr(unparsed_args)));
}
}  // namespace rclpy
