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

#ifndef RCLPY__GUARD_CONDITION_HPP_
#define RCLPY__GUARD_CONDITION_HPP_

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rclpy
{
/// Create a general purpose guard condition
/**
 * A successful call will return a Capsule with the pointer of the created
 * rcl_guard_condition_t * structure
 *
 * Raises RuntimeError if initializing the guard condition fails
 *
 * \return a guard condition capsule
 */
py::capsule
guard_condition_create(py::capsule pycontext);

/// Trigger a general purpose guard condition
/**
 * Raises ValueError if pygc is not a guard condition capsule
 * Raises RCLError if the guard condition could not be triggered
 *
 * \param[in] pygc Capsule pointing to guard condtition
 */
void
guard_condition_trigger(py::capsule pygc);
}  // namespace rclpy

#endif  // RCLPY__GUARD_CONDITION_HPP_
