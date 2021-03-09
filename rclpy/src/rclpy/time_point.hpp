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

#ifndef RCLPY__TIME_POINT_HPP_
#define RCLPY__TIME_POINT_HPP_

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rclpy
{
/// Create a time point
/**
 * Raises TypeError if argument of invalid type
 * Raises OverflowError if nanoseconds argument cannot be converted to uint64_t
 *
 * \param[in] nanoseconds the nanoseconds to store in the time point
 * \param[in] clock_type enum of type ClockType
 * \return Capsule of the pointer to the created rcl_time_point_t
 */
py::capsule
create_time_point(int64_t nanoseconds, rcl_clock_type_t clock_type);

/// Returns the nanoseconds value of the time point
/**
 * Raises ValueError if pytime_point is not a time point capsule
 *
 * \param[in] pytime_point Capsule pointing to the time point
 * \return nanoseconds on the time time point
 */
int64_t
time_point_get_nanoseconds(py::capsule pytime_point);
}  // namespace rclpy

#endif  // RCLPY__TIME_POINT_HPP_
