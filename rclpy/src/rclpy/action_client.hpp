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

#include <rcl_action/rcl_action.h>

#include <memory>

#include "destroyable.hpp"
#include "handle.hpp"

namespace py = pybind11;

namespace rclpy
{
class ActionClient : public Destroyable, public std::enable_shared_from_this<ActionClient>
{
public:
  /// Create an action client.
  /**
   * This function will create an action client for the given action name.
   * This client will use the typesupport defined in the action module
   * provided as pyaction_type to send messages over the wire.
   *
   * Raises ValueError if action name is invalid
   * Raises RuntimeError if the action client could not be created.
   *
   * \param[in] pynode Capsule pointing to the node to add the action client to.
   * \param[in] pyaction_type Action module associated with the action client.
   * \param[in] pyaction_name Python object containing the action name.
   * \param[in] goal_service_qos rmw_qos_profile_t object for the goal service.
   * \param[in] result_service_qos rmw_qos_profile_t object for the result service.
   * \param[in] cancel_service_qos rmw_qos_profile_t object for the cancel service.
   * \param[in] feedback_qos rmw_qos_profile_t object for the feedback subscriber.
   * \param[in] status_qos rmw_qos_profile_t object for the status subscriber.
   */
  ActionClient(
    py::capsule pynode,
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
   *
   * \param[in] pygoal_response_type An instance of the response message type to take.
   * \return 2-tuple (sequence number, received response), or
   * \return 2-tuple (None, None) if there is no response, or
   * \return NULL if there is a failure.
   */
  py::tuple
  take_goal_response(py::object pymsg_type);

  /// Send an action result request.
  /**
   * Raises AttributeError if there is an issue parsing the pyresult_request.
   * Raises RuntimeError if the underlying rcl library returns an error when sending the request.
   *
   * \param[in] pyresult_request The request message to send.
   * \return sequence_number PyLong object representing the index of the sent request, or
   * \return NULL if there is a failure.
   */
  int64_t
  send_result_request(py::object pyrequest);

  /// Take an action cancel response.
  /**
   * Raises AttributeError if there is an issue parsing the pycancel_response_type.
   * Raises RuntimeError if the underlying rcl library returns an error when taking the response.
   *
   * \param[in] pycancel_response_type An instance of the response message type to take.
   * \return 2-tuple (sequence number, received response), or
   * \return 2-tuple (None, None) if there is no response, or
   * \return NULL if there is a failure.
   */
  py::tuple
  take_cancel_response(py::object pymsg_type);

  /// Send an action cancel request.
  /**
   * Raises AttributeError if there is an issue parsing the pycancel_request.
   * Raises RuntimeError if the underlying rcl library returns an error when sending the request.
   *
   * \param[in] pycancel_request The request message to send.
   * \return sequence_number PyLong object representing the index of the sent request, or
   * \return NULL if there is a failure.
   */
  int64_t
  send_cancel_request(py::object pyrequest);

  /// Take a feedback message from a given action client.
  /**
   * Raises AttributeError if there is an issue parsing the pyfeedback_type.
   * Raises RuntimeError on failure while taking a feedback message. Note, this does not include
   * the case where there are no messages available.
   *
   * \param[in] pyfeedback_type Instance of the feedback message type to take.
   * \return Python message with all fields populated with received message, or
   * \return None if there is nothing to take, or
   * \return NULL if there is a failure.
   */
  py::object
  take_feedback(py::object pymsg_type);

  /// Take a status message from a given action client.
  /**
   * Raises AttributeError if there is an issue parsing the pystatus_type.
   * Raises RuntimeError on failure while taking a status message. Note, this does not include
   * the case where there are no messages available.
   *
   * \param[in] pystatus_type Instance of the status message type to take.
   * \return Python message with all fields populated with received message, or
   * \return None if there is nothing to take, or
   * \return NULL if there is a failure.
   */
  py::object
  take_status(py::object pymsg_type);

  /// Send an action goal request.
  /**
   * Raises AttributeError if there is an issue parsing the pygoal_request.
   * Raises RuntimeError on failure.
   *
   * \param[in] pygoal_request The request message to send.
   * \return sequence_number PyLong object representing the index of the sent request, or
   * \return NULL if there is a failure.
   */
  int64_t
  send_goal_request(py::object pyrequest);

  /// Take an action result response.
  /**
   * Raises AttributeError if there is an issue parsing the pyresult_response_type.
   * Raises RuntimeError if the underlying rcl library returns an error when taking the response.
   *
   * \param[in] pyresult_response_type An instance of the response message type to take.
   * \return 2-tuple (sequence number, received response), or
   * \return 2-tuple (None, None) if there is no response, or
   * \return NULL if there is a failure.
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
  is_available();

  /// Check if an action entity has any ready wait set entities.
  /**
   * This must be called after waiting on the wait set.
   * Raises RuntimeError on failure.
   *
   * \param[in] pywait_set Capsule pointing to the wait set structure.
   * \return A tuple of Bool representing the ready sub-entities.
   *     For a rcl_action_client_t:
   *       (is_feedback_ready,
   *        is_status_ready,
   *        is_goal_response_ready,
   *        is_cancel_response_ready,
   *        is_result_response_ready)
   */
  py::tuple
  wait_set_is_ready(py::capsule pywait_set);

  /// Add an action entitiy to a wait set.
  /**
   * Raises RuntimeError on failure.
   * \param[in] pywait_set Capsule pointer to an rcl_wait_set_t.
   */
  void
  wait_set_add(py::capsule pywait_set);

  /// Get rcl_client_t pointer
  rcl_action_client_t *
  rcl_ptr() const
  {
    return rcl_action_client_.get();
  }

  /// Get rcl_client_t pointer
  std::shared_ptr<rcl_action_client_t>
  get_rcl_shared_ptr()
  {
    return rcl_action_client_;
  }

  /// Force an early destruction of this object
  void
  destroy() override;

private:
  std::shared_ptr<Handle> node_handle_;
  std::shared_ptr<rcl_action_client_t> rcl_action_client_;
};
/// Define a pybind11 wrapper for an rcl_time_point_t
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void define_action_client(py::object module);
}  // namespace rclpy

#endif  // RCLPY__ACTION_CLIENT_HPP_
