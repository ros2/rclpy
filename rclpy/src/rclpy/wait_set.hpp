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

#include <memory>
#include <string>

#include "client.hpp"
#include "service.hpp"

namespace py = pybind11;

namespace rclpy
{
/// Return a Capsule pointing to a zero initialized rcl_wait_set_t structure
py::capsule
get_zero_initialized_wait_set();

/// Initialize a wait set
/**
 * Raises RCLError if the wait set could not be initialized
 *
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \param[in] node_name string name of the node to be created
 * \param[in] number_of_subscriptions int
 * \param[in] number_of_guard_conditions int
 * \param[in] number_of_timers int
 * \param[in] number_of_clients int
 * \param[in] number_of_services int
 * \param[in] pycontext Capsule pointing to an rcl_context_t
 */
void
wait_set_init(
  py::capsule pywait_set,
  size_t number_of_subscriptions,
  size_t number_of_guard_conditions,
  size_t number_of_timers,
  size_t number_of_clients,
  size_t number_of_services,
  size_t number_of_events,
  py::capsule pycontext);

/// Clear all the pointers in the wait set
/**
 * Raises RCLError if any rcl error occurs
 *
 * \param[in] pywait_set Capsule pointing to the wait set structure
 */
void
wait_set_clear_entities(py::capsule pywait_set);

/// Add an entity to the wait set structure
/**
 * Raises RuntimeError if the entity type is unknown
 * Raises RCLError if any lower level error occurs
 *
 * \param[in] entity_type string defining the entity ["subscription, client, service"]
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \param[in] pyentity Capsule pointing to the entity to add
 * \return Index in waitset entity was added at
 */
size_t
wait_set_add_entity(const std::string & entity_type, py::capsule pywait_set, py::capsule pyentity);

/// Add a service to the wait set structure
/**
 * Raises RCLError if any lower level error occurs
 *
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \param[in] service a service to add to the wait set
 * \return Index in waitset entity was added at
 */
size_t
wait_set_add_service(const py::capsule pywait_set, const Service & service);

/// Add a client to the wait set structure
/**
 * Raises RCLError if any lower level error occurs
 *
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \param[in] client a client to add to the wait set
 * \return Index in waitset entity was added at
 */
size_t
wait_set_add_client(const py::capsule pywait_set, const Client & client);

/// Check if an entity in the wait set is ready by its index
/**
 * This must be called after waiting on the wait set.
 * Raises RuntimeError if the entity type is unknown
 * Raises IndexError if the given index is beyond the number of entities in the set
 *
 * \param[in] entity_type string defining the entity ["subscription, client, service"]
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \param[in] index location in the wait set of the entity to check
 * \return True if the entity at the index in the wait set is not NULL
 */
bool
wait_set_is_ready(const std::string & entity_type, py::capsule pywait_set, size_t index);

/// Get list of non-null entities in wait set
/**
 * Raises ValueError if pywait_set is not a wait set capsule
 * Raises RuntimeError if the entity type is not known
 *
 * \param[in] entity_type string defining the entity ["subscription, client, service"]
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \return List of wait set entities pointers ready for take
 */
py::list
get_ready_entities(const std::string & entity_type, py::capsule pywait_set);

/// Wait until timeout is reached or event happened
/**
 * Raises ValueError if pywait_set is not a wait set capsule
 * Raises RCLError if there was an error while waiting
 *
 * This function will wait for an event to happen or for the timeout to expire.
 * A negative timeout means wait forever, a timeout of 0 means no wait
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \param[in] timeout optional time to wait before waking up (in nanoseconds)
 */
void
wait(py::capsule pywait_set, int64_t timeout);
}  // namespace rclpy

#endif  // RCLPY__WAIT_SET_HPP_
