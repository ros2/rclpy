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

#ifndef RCLPY__SERIALIZATION_HPP_
#define RCLPY__SERIALIZATION_HPP_

#include <pybind11/pybind11.h>

#include <rcl/types.h>
#include <rcutils/allocator.h>

namespace py = pybind11;

namespace rclpy
{
/// Helper struct to manage an rcl_serialized_message_t lifetime.
struct SerializedMessage
{
  explicit SerializedMessage(rcutils_allocator_t allocator);

  ~SerializedMessage();

  rcl_serialized_message_t rcl_msg;
};

/// Serialize a ROS message
/**
 * Raises RCUtilsError on failure to initialize a serialized message
 * Raises RMWError on serialization failure
 *
 * \param[in] pymsg an instance of a ROS message
 * \param[in] pymsg_type the type of the ROS message
 * \return serialized bytes
 */
py::bytes
serialize(py::object pymsg, py::object pymsg_type);

/// Deserialize a ROS message
/**
 * Raises RMWError on deserialization failure
 *
 * \param[in] pybuffer a serialized ROS message
 * \param[in] pymsg_type the type of the ROS message to deserialize
 * \return an instance of a ROS message
 */
py::object
deserialize(py::bytes pybuffer, py::object pymsg_type);
}  // namespace rclpy

#endif  // RCLPY__SERIALIZATION_HPP_
