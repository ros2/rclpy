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

#include <pybind11/pybind11.h>

#include <rcl/allocator.h>
#include <rcl/arguments.h>
#include <rcl/error_handling.h>
#include <rcutils/error_handling.h>
#include <rmw/error_handling.h>

#include <memory>
#include <stdexcept>
#include <string>

#include "rclpy_common/exceptions.hpp"

namespace py = pybind11;

namespace rclpy
{
std::string append_rcutils_error(std::string prepend)
{
  prepend += ": ";
  prepend += rcutils_get_error_string().str;
  rcutils_reset_error();
  return prepend;
}

std::string append_rcl_error(std::string prepend)
{
  prepend += ": ";
  prepend += rcl_get_error_string().str;
  rcl_reset_error();
  return prepend;
}

std::string append_rmw_error(std::string prepend)
{
  prepend += ": ";
  prepend += rmw_get_error_string().str;
  rmw_reset_error();
  return prepend;
}

RCUtilsError::RCUtilsError(const std::string & error_text)
: std::runtime_error(append_rcl_error(error_text))
{
}

RCLError::RCLError(const std::string & error_text)
: std::runtime_error(append_rcl_error(error_text))
{
}

RMWError::RMWError(const std::string & error_text)
: std::runtime_error(append_rmw_error(error_text))
{
}

void
throw_if_unparsed_ros_args(py::list pyargs, const rcl_arguments_t & rcl_args)
{
  int unparsed_ros_args_count = rcl_arguments_get_count_unparsed_ros(&rcl_args);

  if (unparsed_ros_args_count < 0) {
    throw std::runtime_error("failed to count unparsed arguments");
  } else if (0 == unparsed_ros_args_count) {
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
