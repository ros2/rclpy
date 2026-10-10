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

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace rclpy
{
/// Define a nanobind wrapper for an rcl_duration_t
/**
 * \param[in] module a nanobind module to add the definition to
 */
void define_duration(nb::object module);
}  // namespace rclpy

#endif  // RCLPY__DURATION_HPP_
