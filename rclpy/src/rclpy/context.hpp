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

#ifndef RCLPY__CONTEXT_HPP_
#define RCLPY__CONTEXT_HPP_

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rclpy
{
/// Retrieves domain id from init_options of context
/**
 * \param[in] pycontext Capsule containing rcl_context_t
 * \return domain id
 */
size_t
context_get_domain_id(py::capsule pycontext);

/// Create a capsule with an rcl_context_t instance.
/**
 * The returned context is zero-initialized for use with rclpy_init().
 *
 * Raises MemoryError if allocating memory fails.
 * Raises RuntimeError if creating the context fails.
 *
 * \return capsule with the rcl_context_t instance
 */
py::capsule
create_context();

/// Status of the the client library
/**
 * \return True if rcl is running properly, False otherwise
 */
bool
context_is_valid(py::capsule context);
}  // namespace rclpy

#endif  // RCLPY__CONTEXT_HPP_
