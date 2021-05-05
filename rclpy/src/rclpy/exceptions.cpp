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

#include "exceptions.hpp"

#include <rcl/error_handling.h>
#include <rcutils/error_handling.h>
#include <rmw/error_handling.h>

#include <stdexcept>
#include <string>

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
}  // namespace rclpy
