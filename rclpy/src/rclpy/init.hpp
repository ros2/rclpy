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

#ifndef RCLPY__INIT_HPP_
#define RCLPY__INIT_HPP_

#include <pybind11/pybind11.h>

#include <string>
#include <vector>

namespace py = pybind11;

namespace rclpy
{
/// Initialize rcl with default options, ignoring parameters
/**
 * Raises RCLError if rcl could not be initialized
 * Raises UnknownROSArgsError if any CLI arguments could not be parsed
 * Raises RuntimeError if an internal error happens
 */
void
init(py::list pyargs, py::capsule pycontext, size_t domain_id);

/// Request shutdown of the client library
/**
 * Raises RCLError if the library could not be shutdown
 */
void
shutdown(py::capsule pycontext);
}  // namespace rclpy

#endif  // RCLPY__INIT_HPP_
