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

#ifndef RCLPY__EXCEPTIONS_HPP_
#define RCLPY__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace rclpy
{
std::string append_rcutils_error(std::string prepend);

std::string append_rcl_error(std::string prepend);

std::string append_rmw_error(std::string prepend);

class RCUtilsError : public std::runtime_error
{
public:
  explicit RCUtilsError(const std::string & error_text);

  ~RCUtilsError() = default;
};

class RCLError : public std::runtime_error
{
public:
  explicit RCLError(const std::string & error_text);

  ~RCLError() = default;
};

class RMWError : public std::runtime_error
{
public:
  explicit RMWError(const std::string & error_text);

  ~RMWError() = default;
};

class RCLInvalidROSArgsError : public RCLError
{
  using RCLError::RCLError;
};

class UnknownROSArgsError : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

class NodeNameNonExistentError : public RCLError
{
  using RCLError::RCLError;
};

class UnsupportedEventTypeError : public RCLError
{
  using RCLError::RCLError;
};

class NotImplementedError : public RCLError
{
  using RCLError::RCLError;
};

class InvalidHandle : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

}  // namespace rclpy

#endif  // RCLPY__EXCEPTIONS_HPP_
