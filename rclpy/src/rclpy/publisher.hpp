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
/// Create a publisher
/**
 * This function will create a publisher and attach it to the provided topic name
 * This publisher will use the typesupport defined in the message module
 * provided as pymsg_type to send messages over the wire.
 *
 * Raises ValueError if the topic name is invalid
 * Raises ValueError if the capsules are not the correct types
 * Raises RCLError if the publisher cannot be created
 *
 * \param[in] pynode Capsule pointing to the node to add the publisher to
 * \param[in] pymsg_type Message type associated with the publisher
 * \param[in] topic The name of the topic to attach the publisher to
 * \param[in] pyqos_profile QoSProfile object with the profile of this publisher
 * \return Capsule of the pointer to the created rcl_publisher_t * structure, or
 */
py::capsule
publisher_create(
  py::capsule pynode, py::object pymsg_type, std::string topic,
  py::capsule pyqos_profile);

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

/// Publish a message
/**
 * Raises ValueError if pypublisher is not a publisher capsule
 * Raises RCLError if the message cannot be published
 *
 * \param[in] pypublisher Capsule pointing to the publisher
 * \param[in] pymsg Message to send
 */
void
publisher_publish_message(py::capsule pypublisher, py::object pymsg);

/// Publish a serialized message
/**
 * Raises ValueError if pypublisher is not a publisher capsule
 * Raises RCLError if the message cannot be published
 *
 * \param[in] pypublisher Capsule pointing to the publisher
 * \param[in] msg serialized message to send
 */
void
publisher_publish_raw(py::capsule pypublisher, std::string msg);
}  // namespace rclpy

#endif  // RCLPY__PUBLISHER_HPP_
