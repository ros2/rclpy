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

#ifndef RCLPY__ACTION_CLIENT_HPP_
#define RCLPY__ACTION_CLIENT_HPP_

#include <pybind11/pybind11.h>

#include <rcl_action/action_client.h>
#include <rmw/types.h>

#include <memory>

#include "destroyable.hpp"
#include "node.hpp"
#include "wait_set.hpp"

namespace py = pybind11;

namespace rclpy
{
/**
 * This class will create an action client for the given action name.
 * This client will use the typesupport defined in the action module
 * provided as pyaction_type to send messages.
 */
class ActionClient : public Destroyable, public std::enable_shared_from_this<ActionClient>
{
public:
  /// Create an action client.
  /*
   * Raises ValueError if action name is invalid.
   * Raises RuntimeError if the action client could not be created.
   *
   * \param[in] node Node to add the action client to.
   * \param[in] pyaction_type Action module associated with the action client.
   * \param[in] action_name The action name.
   * \param[in] goal_service_qos rmw_qos_profile_t object for the goal service.
   * \param[in] result_service_qos rmw_qos_profile_t object for the result service.
   * \param[in] cancel_service_qos rmw_qos_profile_t object for the cancel service.
   * \param[in] feedback_topic_qos rmw_qos_profile_t object for the feedback subscriber.
   * \param[in] status_topic_qos rmw_qos_profile_t object for the status subscriber.
   */
  ActionClient(
    Node & node,
    py::object pyaction_type,
    const char * action_name,
    const rmw_qos_profile_t & goal_service_qos,
    const rmw_qos_profile_t & result_service_qos,
    const rmw_qos_profile_t & cancel_service_qos,
    const rmw_qos_profile_t & feedback_topic_qos,
    const rmw_qos_profile_t & status_topic_qos);

  /// Take an action goal response.
  /**
   * Raises AttributeError if there is an issue parsing the pygoal_response_type.
   * Raises RuntimeError if the underlying rcl library returns an error when taking the response.
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] pymsg_type An instance of the response message type to take.
   * \return 2-tuple (sequence number, received response), or
   * \return 2-tuple (None, None) if there is no response, or
   * \return (None, None) if there is a failure in the rcl API call.
   */
  py::tuple
  take_goal_response(py::object pymsg_type);

  /// Send an action result request.
  /**
   * Raises AttributeError if there is an issue parsing the pyresult_request.
   * Raises RuntimeError if the underlying rcl library returns an error when sending the request.
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] pyrequest The request message to send.
   * \return sequence_number the index of the sent request
   */
  int64_t
  send_result_request(py::object pyrequest);

  /// Take an action cancel response.
  /**
   * Raises AttributeError if there is an issue parsing the pycancel_response_type.
   * Raises RuntimeError if the underlying rcl library returns an error when taking the response.
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] pymsg_type An instance of the response message type to take.
   * \return 2-tuple (sequence number, received response), or
   * \return 2-tuple (None, None) if there is no response, or
   * \return (None, None) if there is a failure in the rcl API call.
   */
  py::tuple
  take_cancel_response(py::object pymsg_type);

  /// Send an action cancel request.
  /**
   * Raises AttributeError if there is an issue parsing the pycancel_request.
   * Raises RuntimeError if the underlying rcl library returns an error when sending the request.
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] pyrequest The request message to send.
   * \return The index of the sent request.
   */
  int64_t
  send_cancel_request(py::object pyrequest);

  /// Take a feedback message from a given action client.
  /**
   * Raises AttributeError if there is an issue parsing the pyfeedback_type.
   * Raises RuntimeError on failure while taking a feedback message. Note, this does not include
   * the case where there are no messages available.
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] pymsg_type Instance of the feedback message type to take.
   * \return Python message with all fields populated with received message, or
   * \return None if there is nothing to take, or
   * \return None if there is a failure in the rcl API call.
   */
  py::object
  take_feedback(py::object pymsg_type);

  /// Take a status message from a given action client.
  /**
   * Raises AttributeError if there is an issue parsing the pystatus_type.
   * Raises RuntimeError on failure while taking a status message. Note, this does not include
   * the case where there are no messages available.
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] pymsg_type Instance of the status message type to take.
   * \return Python message with all fields populated with received message, or
   * \return None if there is nothing to take, or
   * \return None if there is a failure in the rcl API call.
   */
  py::object
  take_status(py::object pymsg_type);

  /// Send an action goal request.
  /**
   * Raises AttributeError if there is an issue parsing the pygoal_request.
   * Raises RuntimeError on failure.
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] pyrequest The request message to send.
   * \return The index of the sent request
   */
  int64_t
  send_goal_request(py::object pyrequest);

  /// Take an action result response.
  /**
   * Raises AttributeError if there is an issue parsing the pyresult_response_type.
   * Raises RuntimeError if the underlying rcl library returns an error when taking the response.
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] pymsg_type An instance of the response message type to take.
   * \return 2-tuple (sequence number, received response), or
   * \return 2-tuple (None, None) if there is no response, or
   * \return (None, None) if there is a failure in the rcl API call.
   */
  py::tuple
  take_result_response(py::object pymsg_type);

  /// Get the number of wait set entities that make up an action entity.
  /**
   * \return Tuple containing the number of wait set entities:
   *   (num_subscriptions,
   *    num_guard_conditions,
   *    num_timers,
   *    num_clients,
   *    num_services)
   */
  py::tuple
  get_num_entities();

  /// Check if an action server is available for the given action client.
  /**
   * Raises RuntimeError on failure.
   *
   * \return True if an action server is available, False otherwise.
   */
  bool
  is_action_server_available();

  /// Check if an action entity has any ready wait set entities.
  /**
   * This must be called after waiting on the wait set.
   * Raises RuntimeError on failure.
   *
   * \param[in] wait_set Capsule pointing to the wait set structure.
   * \return A tuple of booleans representing the sub-entities ready:
   *       (is_feedback_ready,
   *        is_status_ready,
   *        is_goal_response_ready,
   *        is_cancel_response_ready,
   *        is_result_response_ready)
   */
  py::tuple
  is_ready(WaitSet & wait_set);

  /// Add an action entitiy to a wait set.
  /**
   * Raises RuntimeError on failure.
   * \param[in] wait_set Capsule pointer to an rcl_wait_set_t.
   */
  void
  add_to_waitset(WaitSet & wait_set);

  /// Get rcl_action_client_t pointer
  rcl_action_client_t *
  rcl_ptr() const
  {
    return rcl_action_client_.get();
  }

  /// Force an early destruction of this object
  void
  destroy() override;

private:
  Node node_;
  std::shared_ptr<rcl_action_client_t> rcl_action_client_;
};
/// Define a pybind11 wrapper for an rcl_time_point_t
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void define_action_client(py::object module);
}  // namespace rclpy

#endif  // RCLPY__ACTION_CLIENT_HPP_
