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

#include "rclpy_common/exceptions.hpp"

#include <rcl/error_handling.h>

#include <stdexcept>
#include <string>

namespace rclpy
{
std::string append_rcl_error(std::string prepend)
{
  prepend += ": ";
  prepend += rcl_get_error_string().str;
  rcl_reset_error();
  return prepend;
}

RCLError::RCLError(const std::string & error_text)
: std::runtime_error(append_rcl_error(error_text))
{
}
}  // namespace rclpy
