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

#ifndef RCLPY__EVENT_HANDLE_HPP_
#define RCLPY__EVENT_HANDLE_HPP_

#include <pybind11/pybind11.h>

#include <rcl/event.h>

#include <memory>
#include <variant>

#include "destroyable.hpp"
#include "publisher.hpp"
#include "subscription.hpp"

namespace py = pybind11;

namespace rclpy
{
/*
 * This class will create an event handle for the given subscription.
 */
class EventHandle : public Destroyable, public std::enable_shared_from_this<EventHandle>
{
public:
  /// Create a subscription event
  /**
   * Raises UnsupportedEventTypeError if the event type is not supported
   * Raises TypeError if arguments are not of the correct types i.e. a subscription capsule
   * Raises MemoryError if the event can't be allocated
   * Raises RCLError if event initialization failed in rcl
   *
   * \param[in] subscription Subscription wrapping the underlying ``rcl_subscription_t`` object.
   * \param[in] event_type Type of event to create
   */
  EventHandle(rclpy::Subscription & subscriber, rcl_subscription_event_type_t event_type);

  /// Create a publisher event
  /**
   * This function will create an event handle for the given publisher.
   *
   * Raises UnsupportedEventTypeError if the event type is not supported
   * Raises TypeError if arguments are not of the correct types i.e. a publisher capsule
   * Raises MemoryError if the event can't be allocated
   * Raises RCLError if event initialization failed in rcl
   *
   * \param[in] publisher Publisher wrapping the underlying ``rcl_publisher_t`` object.
   * \param[in] event_type Type of event to create
   */
  EventHandle(rclpy::Publisher & publisher, rcl_publisher_event_type_t event_type);

  ~EventHandle() = default;

  /// Get pending data from a ready QoS event.
  /**
   * After having determined that a middleware event is ready, get the callback payload.
   *
   * Raises MemoryError if event data can't be allocated
   * Raises TypeError if arguments are not of the correct types
   * Raises RCLError if taking event data failed in rcl
   *
   * \return Event data as an instance of a suitable rclpy.event_handle type, or None
   *   if no event was taken.
   */
  py::object
  take_event();

  /// Get rcl_event_t pointer
  rcl_event_t *
  rcl_ptr() const
  {
    return rcl_event_.get();
  }

  /// Force an early destruction of this object
  void
  destroy() override;

private:
  std::variant<rcl_subscription_event_type_t, rcl_publisher_event_type_t> event_type_;
  std::variant<Publisher, Subscription> grandparent_;
  std::shared_ptr<rcl_event_t> rcl_event_;
};

/// Define a pybind11 wrapper for an rclpy::EventHandle
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_event_handle(py::module module);
}  // namespace rclpy

#endif  // RCLPY__EVENT_HANDLE_HPP_
