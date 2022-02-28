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

#ifndef RCLPY__ACTION_SERVER_HPP_
#define RCLPY__ACTION_SERVER_HPP_

#include <pybind11/pybind11.h>

#include <rcl_action/action_server.h>
#include <rmw/types.h>

#include <memory>

#include "clock.hpp"
#include "destroyable.hpp"
#include "node.hpp"
#include "wait_set.hpp"

namespace py = pybind11;

namespace rclpy
{
/*
  * This class will create an action server for the given action name.
  * This client will use the typesupport defined in the action module
  * provided as pyaction_type to send messages.
  */
class ActionServer : public Destroyable, public std::enable_shared_from_this<ActionServer>
{
public:
  /// Create an action server.
  /**
   * Raises AttributeError if action type is invalid
   * Raises ValueError if action name is invalid
   * Raises RuntimeError if the action server could not be created.
   *
   * \param[in] node Node to add the action server to.
   * \param[in] rclpy_clock Clock use to create the action server.
   * \param[in] pyaction_type Action module associated with the action server.
   * \param[in] action_name The action name.
   * \param[in] goal_service_qos rmw_qos_profile_t object for the goal service.
   * \param[in] result_service_qos rmw_qos_profile_t object for the result service.
   * \param[in] cancel_service_qos rmw_qos_profile_t object for the cancel service.
   * \param[in] feedback_topic_qos rmw_qos_profile_t object for the feedback subscriber.
   * \param[in] status_topic_qos rmw_qos_profile_t object for the status subscriber.
   * \param[in] result_timeout The number of seconds to wait for the result.
   */
  ActionServer(
    Node & node,
    const rclpy::Clock & rclpy_clock,
    py::object pyaction_type,
    const char * action_name,
    const rmw_qos_profile_t & goal_service_qos,
    const rmw_qos_profile_t & result_service_qos,
    const rmw_qos_profile_t & cancel_service_qos,
    const rmw_qos_profile_t & feedback_topic_qos,
    const rmw_qos_profile_t & status_topic_qos,
    double result_timeout);

  /// Take an action goal request.
  /**
   * Raises RCLError if an error occurs in rcl
   * Raises RuntimeError on failure.
   *
   * \param[in] pymsg_type An instance of the type of request message to take.
   * \return 2-tuple (header, received request message) where the header is an
   *   "rclpy.rmw_request_id_t" type, or
   * \return 2-tuple (None, None) if there as no message to take
   */
  py::tuple
  take_goal_request(py::object pymsg_type);

  /// Send an action goal response.
  /**
   * Raises RCLError if an error occurs in rcl
   * Raises RuntimeError on failure.
   *
   * \param[in] header Pointer to the message header.
   * \param[in] pyresponse The response message to send.
   */
  void
  send_goal_response(
    rmw_request_id_t * header, py::object pyresponse);

  /// Send an action result response.
  /**
   * Raises RCLError if an error occurs in rcl
   * Raises RuntimeError on failure.
   *
   * \param[in] header Pointer to the message header.
   * \param[in] pyresponse The response message to send.
   */
  void
  send_result_response(
    rmw_request_id_t * header, py::object pyresponse);

  /// Take an action cancel request.
  /**
   * Raises RCLError if an error occurs in rcl
   * Raises RuntimeError on failure.
   *
   * \param[in] pymsg_type An instance of the type of request message to take.
   * \return 2-tuple (header, received request message) where the header is an
   *   "rmw_request_id_t" type, or
   * \return 2-tuple (None, None) if there as no message to take
   */
  py::tuple
  take_cancel_request(py::object pymsg_type);

  /// Take an action result request.
  /**
   * Raises RCLError if an error occurs in rcl
   * Raises RuntimeError on failure.
   *
   * \param[in] pymsg_type An instance of the type of request message to take.
   * \return 2-tuple (header, received request message) where the header is an
   *   "rclpy.rmw_request_id_t" type, or
   * \return 2-tuple (None, None) if there as no message to take
   */
  py::tuple
  take_result_request(py::object pymsg_type);

  /// Send an action cancel response.
  /**
   * Raises RCLError if an error occurs in rcl
   * Raises RuntimeError on failure.
   *
   * \param[in] header Pointer to the message header.
   * \param[in] pyresponse The response message to send.
   */
  void
  send_cancel_response(
    rmw_request_id_t * header, py::object pyresponse);

  /// Publish a feedback message from a given action server.
  /**
   * Raises RCLError if an error occurs in rcl
   * Raises RuntimeError on failure while publishing a feedback message.
   *
   * \param[in] pymsg The feedback message to publish.
   */
  void
  publish_feedback(py::object pymsg);

  /// Publish a status message from a given action server.
  /**
   * Raises RCLError if an error occurs in rcl
   * Raises RuntimeError on failure while publishing a status message.
   */
  void
  publish_status();

  /// Notifies action server that a goal handle reached a terminal state.
  /**
   * Raises RCLError if an error occurs in rcl
   */
  void
  notify_goal_done();

  /// Check if a goal is already being tracked by an action server.
  /**
   * Raises AttributeError if there is an issue parsing the pygoal_info.
   *
   * \param[in] pygoal_info The identifiers of goals that expired, or set to `NULL` if unused.
   * \return True if the goal exists, false otherwise.
   */
  bool
  goal_exists(py::object pygoal_info);

  /// Process a cancel request using an action server.
  /**
   * This is a non-blocking call.
   *
   * Raises RuntimeError on failure while publishing a status message.
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] pycancel_request The request message to send.
   * \param[in] pycancel_response_type The cancel response type.
   * \return The cancel response message.
   */
  py::object
  process_cancel_request(
    py::object pycancel_request, py::object pycancel_response_type);

  /// Expires goals associated with an action server.
  /**
   * \param[in] max_num_goals The maximum number of goals to expire.
   * \return A tuple of GoalInfos corresponding to the canceled goals.
   */
  py::tuple
  expire_goals(int64_t max_num_goals);

  /// Get the number of wait set entities that make up an action entity.
  /**
   * Raises RCLError if an error occurs in rcl
   *
   * \return Tuple containing the number of wait set entities:
   *   (num_subscriptions,
   *    num_guard_conditions,
   *    num_timers,
   *    num_clients,
   *    num_services)
   */
  py::tuple
  get_num_entities();

  /// Check if an action entity has any ready wait set entities.
  /**
   * This must be called after waiting on the wait set.
   * Raises RuntimeError on failure.
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] wait_set Capsule pointing to the wait set structure.
   * \return A tuple of booleans representing ready sub-entities.
   *       (is_goal_request_ready,
   *        is_cancel_request_ready,
   *        is_result_request_ready,
   *        is_goal_expired)
   */
  py::tuple
  is_ready(WaitSet & wait_set);

  /// Add an action entitiy to a wait set.
  /**
   * Raises RuntimeError on failure.
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] wait_set Capsule pointer to an rcl_wait_set_t.
   */
  void
  add_to_waitset(WaitSet & wait_set);

  /// Get rcl_action_server_t pointer
  rcl_action_server_t *
  rcl_ptr() const
  {
    return rcl_action_server_.get();
  }

  /// Force an early destruction of this object
  void
  destroy() override;

private:
  Node node_;
  std::shared_ptr<rcl_action_server_t> rcl_action_server_;
};
/// Define a pybind11 wrapper for an rcl_time_point_t
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void define_action_server(py::object module);
}  // namespace rclpy

#endif  // RCLPY__ACTION_SERVER_HPP_
