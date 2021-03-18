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

#ifndef RCLPY_COMMON__EXCEPTIONS_HPP_
#define RCLPY_COMMON__EXCEPTIONS_HPP_

#include <pybind11/pybind11.h>

#include <rcl/arguments.h>

#include <stdexcept>
#include <string>

#include "rclpy_common/visibility_control.h"

namespace py = pybind11;

namespace rclpy
{
RCLPY_COMMON_PUBLIC
std::string append_rcutils_error(std::string prepend);

RCLPY_COMMON_PUBLIC
std::string append_rcl_error(std::string prepend);

RCLPY_COMMON_PUBLIC
std::string append_rmw_error(std::string prepend);

RCLPY_COMMON_PUBLIC
void
throw_if_unparsed_ros_args(py::list pyargs, const rcl_arguments_t & rcl_args);

class RCUtilsError : public std::runtime_error
{
public:
  RCLPY_COMMON_PUBLIC
  explicit RCUtilsError(const std::string & error_text);

  RCLPY_COMMON_PUBLIC
  ~RCUtilsError() = default;
};

class RCLError : public std::runtime_error
{
public:
  RCLPY_COMMON_PUBLIC
  explicit RCLError(const std::string & error_text);

  RCLPY_COMMON_PUBLIC
  ~RCLError() = default;
};

class RMWError : public std::runtime_error
{
public:
  RCLPY_COMMON_PUBLIC
  explicit RMWError(const std::string & error_text);

  RCLPY_COMMON_PUBLIC
  ~RMWError() = default;
};

class RCLInvalidROSArgsError : public RCLError
{
};

class UnknownROSArgsError : public std::runtime_error
{
private:
  using std::runtime_error::runtime_error;

  friend void throw_if_unparsed_ros_args(py::list, const rcl_arguments_t &);
};

class NodeNameNonExistentError : public RCLError
{
  using RCLError::RCLError;
};

class UnsupportedEventTypeError : public RCLError
{
  using RCLError::RCLError;
};
}  // namespace rclpy

#endif  // RCLPY_COMMON__EXCEPTIONS_HPP_
