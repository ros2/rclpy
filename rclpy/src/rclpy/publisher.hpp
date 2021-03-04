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

#ifndef RCLPY__PUBLISHER_HPP_
#define RCLPY__PUBLISHER_HPP_

#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

namespace rclpy
{
/// Count subscribers from a publisher.
/**
 * Raise ValueError if capsule is not a publisher
 * Raises RCLError if the subscriber count cannot be determined
 *
 * \param[in] pypublisher Capsule pointing to the publisher
 * \return count of subscribers
 */
size_t
publisher_get_subscription_count(py::capsule pypublisher);

/// Retrieve the topic name from a rclpy_publisher_t
/**
 * Raise ValueError if capsule is not a publisher
 * Raises RCLError if the name cannot be determined
 *
 * \param[in] pypublisher Capsule pointing to the publisher
 * \return name of the publisher's topic
 */
std::string
publisher_get_topic_name(py::capsule pypublisher);
}  // namespace rclpy

#endif  // RCLPY__PUBLISHER_HPP_
