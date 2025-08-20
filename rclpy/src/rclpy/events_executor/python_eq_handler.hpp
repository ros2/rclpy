// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef RCLPY__EVENTS_EXECUTOR__PYTHON_EQ_HANDLER_HPP_
#define RCLPY__EVENTS_EXECUTOR__PYTHON_EQ_HANDLER_HPP_

#include <pybind11/pybind11.h>

namespace rclpy
{
namespace events_executor
{
/// This is a workaround to the deprecation of `operator==` in Pybind11 >=2.2
/// See https://pybind11.readthedocs.io/en/stable/upgrade.html#deprecation-of-some-py-object-apis
/// It's intended to be replaced with the
struct PythonEqHandler
{
  inline auto operator()(const pybind11::handle & x, const pybind11::handle & y) const
  {
    return x.is(y);
  }
};
}  // namespace events_executor
}  // namespace rclpy

#endif  // RCLPY__EVENTS_EXECUTOR__PYTHON_EQ_HANDLER_HPP_
