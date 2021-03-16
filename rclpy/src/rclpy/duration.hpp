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

#ifndef RCLPY__DURATION_HPP_
#define RCLPY__DURATION_HPP_

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rclpy
{
/// Create a duration
/**
 * Raises TypeError if argument of invalid type
 *
 * \param[in] nanoseconds The nanoseconds value of the duration in a 64-bit signed integer
 * \return Capsule of the pointer to the created rcl_duration_t * structure
 */
py::capsule
create_duration(int64_t nanoseconds);

/// Returns the nanoseconds value of the duration
/**
 * Raises ValueError if pyduration is not a duration capsule
 *
 * \param[in] pyduration Capsule pointing to the duration
 * \return integer nanoseconds
 */
int64_t
duration_get_nanoseconds(py::capsule pyduration);
}  // namespace rclpy

#endif  // RCLPY__DURATION_HPP_
