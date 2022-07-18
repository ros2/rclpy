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

#include <pybind11/pybind11.h>

#include <rcl/error_handling.h>
#include <rcl_action/action_server.h>
#include <rcl_action/goal_handle.h>
#include <rcl_action/types.h>

#include <memory>

#include "action_goal_handle.hpp"
#include "action_server.hpp"
#include "exceptions.hpp"
#include "utils.hpp"

namespace py = pybind11;

namespace rclpy
{
ActionGoalHandle::ActionGoalHandle(
  rclpy::ActionServer & action_server, py::object pygoal_info_msg)
: action_server_(action_server)
{
  auto goal_info_msg = convert_from_py(pygoal_info_msg);
  rcl_action_goal_info_t * goal_info_msg_ptr =
    static_cast<rcl_action_goal_info_t *>(goal_info_msg.get());

  if (!goal_info_msg) {
    throw py::error_already_set();
  }

  auto rcl_handle = rcl_action_accept_new_goal(
    action_server.rcl_ptr(), goal_info_msg_ptr);
  if (!rcl_handle) {
    throw rclpy::RCLError("Failed to accept new goal");
  }

  rcl_action_goal_handle_ = std::shared_ptr<rcl_action_goal_handle_t>(
    new rcl_action_goal_handle_t,
    [](rcl_action_goal_handle_t * goal_handle)
    {
      rcl_ret_t ret = rcl_action_goal_handle_fini(goal_handle);
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Error destroying action goal handle: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete goal_handle;
    });
  // Copy out goal handle since action server storage disappears when it is fini'd
  *rcl_action_goal_handle_ = *rcl_handle;
}

void
ActionGoalHandle::destroy()
{
  rcl_action_goal_handle_.reset();
  action_server_.destroy();
}

rcl_action_goal_state_t
ActionGoalHandle::get_status()
{
  rcl_action_goal_state_t status;
  rcl_ret_t ret = rcl_action_goal_handle_get_status(rcl_action_goal_handle_.get(), &status);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get goal status");
  }

  return status;
}

void
ActionGoalHandle::update_goal_state(rcl_action_goal_event_t event)
{
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_action_goal_handle_.get(), event);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to update goal state");
  }
}

void
define_action_goal_handle(py::module module)
{
  py::class_<ActionGoalHandle, Destroyable, std::shared_ptr<ActionGoalHandle>>(
    module, "ActionGoalHandle")
  .def(py::init<rclpy::ActionServer &, py::object>())
  .def_property_readonly(
    "pointer", [](const ActionGoalHandle & handle) {
      return reinterpret_cast<size_t>(handle.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "get_status", &ActionGoalHandle::get_status,
    "Get the status of a goal.")
  .def(
    "update_goal_state", &ActionGoalHandle::update_goal_state,
    "Update a goal state.")
  .def(
    "is_active", &ActionGoalHandle::is_active,
    "Check if a goal is active.");
}
}  // namespace rclpy
