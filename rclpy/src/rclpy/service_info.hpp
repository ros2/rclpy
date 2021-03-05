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

#ifndef RCLPY__SERVICE_INFO_HPP_
#define RCLPY__SERVICE_INFO_HPP_

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rclpy
{
/// Retrieves the sequence number from a rmw_service_info_t capsule
/**
 * Raises ValueError if pyservice_info is not a service info capsule
 *
 * \param[in] pyservice_info Capsule pointing to the rmw_service_info_t
 * \return the sequence number as a long
 */
int64_t
service_info_get_sequence_number(py::capsule pyservice_info);

/// Retrieves the source timestamp number from a rmw_service_info_t capsule
/**
 * Raises ValueError if pyservice_info is not a service info capsule
 *
 * \param[in] pyservice_info Capsule pointing to the rmw_service_info_t
 * \return the source timestamps as a long
 */
int64_t
service_info_get_source_timestamp(py::capsule pyservice_info);

/// Retrieves the received timestsamp number from a rmw_service_info_t capsule
/**
 * Raises ValueError if pyservice_info is not a service info capsule
 *
 * \param[in] pyservice_info Capsule pointing to the rmw_service_info_t
 * \return the receive timestamp as a long
 */
int64_t
service_info_get_received_timestamp(py::capsule pyservice_info);
}  // namespace rclpy

#endif  // RCLPY__SERVICE_INFO_HPP_
