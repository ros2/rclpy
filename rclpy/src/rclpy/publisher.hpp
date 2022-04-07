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

#include <rcl/publisher.h>
#include <rcl/time.h>

#include <memory>
#include <string>

#include "destroyable.hpp"
#include "node.hpp"

namespace py = pybind11;

namespace rclpy
{
/**
 * This class will create a publisher and attach it to the provided topic name
 * This publisher will use the typesupport defined in the message module
 * provided as pymsg_type to send messages.
 */
class Publisher : public Destroyable, public std::enable_shared_from_this<Publisher>
{
public:
  /// Create a publisher
  /**
   * Raises ValueError if the topic name is invalid
   * Raises ValueError if the capsules are not the correct types
   * Raises RCLError if the publisher cannot be created
   *
   * \param[in] node Node to add the publisher to.
   * \param[in] pymsg_type Message type associated with the publisher.
   * \param[in] topic The name of the topic to attach the publisher to.
   * \param[in] pyqos_profile rmw_qos_profile_t object for this publisher.
   */
  Publisher(
    Node & node, py::object pymsg_type, std::string topic,
    py::object pyqos_profile);

  /// Get the name of the logger associated with the node of the publisher.
  /**
   * Raises RCLError if logger name not set
   *
   * \return the logger name
   */
  const char *
  get_logger_name();

  /// Count subscribers from a publisher.
  /**
   * Raises RCLError if the subscriber count cannot be determined
   *
   * \return number of subscribers
   */
  size_t
  get_subscription_count();

  /// Retrieve the topic name from a rclpy_publisher_t
  /**
   * Raises RCLError if the name cannot be determined
   *
   * \return name of the publisher's topic
   */
  std::string
  get_topic_name();

  /// Publish a message
  /**
   * Raises RCLError if the message cannot be published
   *
   * \param[in] pymsg Message to send.
   */
  void
  publish(py::object pymsg);

  /// Publish a serialized message
  /**
   * Raises RCLError if the message cannot be published
   *
   * \param[in] msg The serialized message to send.
   */
  void
  publish_raw(std::string msg);

  /// Get rcl_publisher_t pointer
  rcl_publisher_t *
  rcl_ptr() const
  {
    return rcl_publisher_.get();
  }

  /// Force an early destruction of this object
  void
  destroy() override;

  /// Wait until all published message data is acknowledged or until the specified timeout elapses
  /**
   * If the timeout is negative then this function will block indefinitely until all published
   * message data is acknowledged.
   * If the timeout is 0 then it will check if all published message has been acknowledged without
   * waiting.
   * If the timeout is greater than 0 then it will return after that period of time has elapsed or
   * all published message data is acknowledged.
   *
   * Raises RCLError if an error occurs, such as the middleware not supporting this feature.
   *
   * \param[in] pytimeout The duration to wait for all published message data to be acknowledged.
   * \return `true` if all published message data is acknowledged before the timeout, otherwise
   *   `false`.
   */
  bool
  wait_for_all_acked(rcl_duration_t pytimeout);

private:
  Node node_;
  std::shared_ptr<rcl_publisher_t> rcl_publisher_;
};
/// Define a pybind11 wrapper for an rclpy::Service
void define_publisher(py::object module);
}  // namespace rclpy

#endif  // RCLPY__PUBLISHER_HPP_
