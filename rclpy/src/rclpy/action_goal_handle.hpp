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

#ifndef RCLPY__ACTION_GOAL_HANDLE_HPP_
#define RCLPY__ACTION_GOAL_HANDLE_HPP_

#include <pybind11/pybind11.h>

#include <rcl_action/goal_handle.h>
#include <rcl_action/types.h>

#include <memory>

#include "action_server.hpp"
#include "destroyable.hpp"

namespace py = pybind11;

namespace rclpy
{

class ActionGoalHandle : public Destroyable, public std::enable_shared_from_this<ActionGoalHandle>
{
public:
  /// Create an action goal handle
  /**
   * This function will create an action goal handle for the given info message from the action server.
   * This action goal handle will use the typesupport defined in the service module
   * provided as pysrv_type to send messages.
   *
   * Raises RCLError if the action goal handle could not be created
   *
   * \param[in] pyaction_server handle to the action server that is accepting the goal
   * \param[in] pygoal_info_msg a message containing info about the goal being accepted
   */
  ActionGoalHandle(rclpy::ActionServer & action_server, py::object pygoal_info_msg);

  ~ActionGoalHandle() = default;

  rcl_action_goal_state_t
  get_status();

  void
  update_goal_state(rcl_action_goal_event_t event);

  /// Check if the goal is still active
  bool
  is_active()
  {
    return rcl_action_goal_handle_is_active(rcl_ptr());
  }

  /// Get rcl_action goal handle_t pointer
  rcl_action_goal_handle_t *
  rcl_ptr() const
  {
    return rcl_action_goal_handle_.get();
  }

  /// Force an early destruction of this object
  void
  destroy() override;

private:
  ActionServer action_server_;
  std::shared_ptr<rcl_action_goal_handle_t> rcl_action_goal_handle_;
};

/// Define a pybind11 wrapper for an rclpy::ActionGoalHandle
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_action_goal_handle(py::module module);
}  // namespace rclpy

#endif  // RCLPY__ACTION_GOAL_HANDLE_HPP_
