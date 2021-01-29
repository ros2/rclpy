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
namespace rclpy
{
class RCLError : public std::runtime_error
{
};

class RCLInvalidROSArgsError : public RCLError
{
};

class UnknownROSArgsError : public RCLError
{
};

class NodeNameNonExistentError : public RCLError
{
};

class UnsupportedEventTypeError : public RCLError
{
};
}  // namespace rclpy

#endif  // RCLPY__EXCEPTIONS_HPP_
