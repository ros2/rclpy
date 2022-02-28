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

#ifndef RCLPY__WAIT_SET_HPP_
#define RCLPY__WAIT_SET_HPP_

#include <pybind11/pybind11.h>

#include <rcl/wait.h>

#include <memory>
#include <string>

#include "client.hpp"
#include "context.hpp"
#include "destroyable.hpp"
#include "guard_condition.hpp"
#include "qos_event.hpp"
#include "service.hpp"
#include "subscription.hpp"
#include "timer.hpp"

namespace py = pybind11;

namespace rclpy
{
class WaitSet : public Destroyable, public std::enable_shared_from_this<WaitSet>
{
public:
  /// Initialize a wait set
  WaitSet();

  /// Initialize a wait set
  /**
   * Raises RCLError if the wait set could not be initialized
   *
   * \param[in] number_of_subscriptions a positive number or zero
   * \param[in] number_of_guard_conditions int
   * \param[in] number_of_timers int
   * \param[in] number_of_clients int
   * \param[in] number_of_services int
   * \param[in] number_of_events int
   * \param[in] context Capsule pointing to an rcl_context_t
   */
  WaitSet(
    size_t number_of_subscriptions,
    size_t number_of_guard_conditions,
    size_t number_of_timers,
    size_t number_of_clients,
    size_t number_of_services,
    size_t number_of_events,
    Context & context);

  /// Clear all the pointers in the wait set
  /**
   * Raises RCLError if any rcl error occurs
   */
  void
  clear_entities();

  /// Add a service to the wait set structure
  /**
   * Raises RCLError if any lower level error occurs
   *
   * \param[in] service A service to add to the wait set
   * \return Index in waitset entity was added at
   */
  size_t
  add_service(const Service & service);

  /// Add a subcription to the wait set structure
  /**
   * Raises RCLError if any lower level error occurs
   *
   * \param[in] subscription A subscription to add to the wait set
   * \return Index in waitset entity was added at
   */
  size_t
  add_subscription(const Subscription & subscription);

  /// Add a client to the wait set structure
  /**
   * Raises RCLError if any lower level error occurs
   *
   * \param[in] client A client to add to the wait set
   * \return Index in waitset entity was added at
   */
  size_t
  add_client(const Client & client);

  /// Add a guard condition to the wait set structure
  /**
   * Raises RCLError if any lower level error occurs
   *
   * \param[in] gc A guard condition to add to the wait set
   * \return Index in waitset entity was added at
   */
  size_t
  add_guard_condition(const GuardCondition & gc);

  /// Add a timer to the wait set structure
  /**
   * Raises RCLError if any lower level error occurs
   *
   * \param[in] timer A timer to add to the wait set
   * \return Index in waitset entity was added at
   */
  size_t
  add_timer(const Timer & timer);

  /// Add an event to the wait set structure
  /**
   * Raises RCLError if any lower level error occurs
   *
   * \param[in] event A QoSEvent to add to the wait set
   * \return Index in waitset entity was added at
   */
  size_t
  add_event(const QoSEvent & event);

  /// Check if an entity in the wait set is ready by its index
  /**
   * This must be called after waiting on the wait set.
   * Raises RuntimeError if the entity type is unknown
   * Raises IndexError if the given index is beyond the number of entities in the set
   *
   * \param[in] entity_type String defining the entity ["subscription, client, service"]
   * \param[in] index Location in the wait set of the entity to check
   * \return True if the entity at the index in the wait set is not NULL
   */
  bool
  is_ready(const std::string & entity_type, size_t index);

  /// Get list of entities ready by entity type
  /**
   * Raises RuntimeError if the entity type is not known
   *
   * \param[in] entity_type String defining the entity ["subscription, client, service"]
   * \return List of wait set entities pointers ready for take
   */
  py::list
  get_ready_entities(const std::string & entity_type);

  /// Wait until timeout is reached or event happened
  /**
   * Raises RCLError if there was an error while waiting
   *
   * This function will wait for an event to happen or for the timeout to expire.
   * A negative timeout means wait forever, a timeout of 0 means no wait
   * \param[in] timeout Optional time to wait before waking up (in nanoseconds)
   */
  void
  wait(int64_t timeout);

  /// Get rcl_wait_set_t pointer
  rcl_wait_set_t * rcl_ptr() const
  {
    return rcl_wait_set_.get();
  }

  /// Force an early destruction of this object
  void destroy() override;

private:
  Context context_;
  std::shared_ptr<rcl_wait_set_t> rcl_wait_set_;
};

/// Define a pybind11 wrapper for an rclpy::Service
void define_waitset(py::object module);
}  // namespace rclpy

#endif  // RCLPY__WAIT_SET_HPP_
