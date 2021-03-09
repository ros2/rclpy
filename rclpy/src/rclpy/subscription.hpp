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

#ifndef RCLPY__SUBSCRIPTION_HPP_
#define RCLPY__SUBSCRIPTION_HPP_

#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

namespace rclpy
{
/// Create a subscription
/**
 * This function will create a subscription for the given topic name.
 * This subscription will use the typesupport defined in the message module
 * provided as pymsg_type to send messages over the wire.
 *
 * On a successful call a Capsule pointing to the pointer of the created
 * rcl_subscription_t * structure is returned
 *
 * Raises ValueError if the capsules are not the correct types
 * Raises RCLError if the subscription could not be created
 *
 * \param[in] pynode Capsule pointing to the node to add the subscriber to
 * \param[in] pymsg_type Message module associated with the subscriber
 * \param[in] topic The topic name
 * \param[in] pyqos_profile QoSProfile Python object for this subscription
 * \return capsule containing the rclpy_subscription_t
 */
py::capsule
subscription_create(
  py::capsule pynode, py::object pymsg_type, std::string topic,
  py::capsule pyqos_profile);

/// Return the resolved topic name of a subscription.
/**
 * The returned string is the resolved topic name after remappings have be applied.
 *
 * Raises ValueError if pysubscription is not a subscription capsule
 * Raises RCLError if the topic name could not be determined
 *
 * \param[in] pysubscription Capsule pointing to the subscription to get the topic name of
 * \return a string with the topic name
 */
std::string
subscription_get_topic_name(py::capsule pysubscription);
}  // namespace rclpy

#endif  // RCLPY__SUBSCRIPTION_HPP_
