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

#include <memory>
#include <string>
#include <iostream>

#include "rclpy_common/common.h"
#include "rclpy_common/exceptions.hpp"

#include "utils.hpp"
#include "action_goal_handle.hpp"

namespace py = pybind11;

namespace rclpy
{
template<typename T>
T
get_pointer(py::capsule & capsule, const char * name)
{
  if (strcmp(name, capsule.name())) {
    std::string error_text{"Expected capusle with name '"};
    error_text += name;
    error_text += "' but got '";
    error_text += capsule.name();
    error_text += "'";
    throw py::value_error(error_text);
  }
  // TODO(sloretz) use get_pointer() in pybind11 2.6+
  return static_cast<T>(capsule);
}

ActionGoalHandle::ActionGoalHandle(
  py::capsule pyaction_server, py::object pygoal_info_msg)
{
  auto action_server = get_pointer<rcl_action_server_t *>(
    pyaction_server, "rcl_action_server_t");

  destroy_ros_message_signature * destroy_ros_message = NULL;
  auto goal_info_msg = static_cast<rcl_action_goal_info_t *>(
    rclpy_convert_from_py(pygoal_info_msg.ptr(), &destroy_ros_message));

  if (!goal_info_msg) {
    throw py::error_already_set();
  }

  auto goal_info_msg_ptr = std::unique_ptr<rcl_action_goal_info_t, decltype(destroy_ros_message)>(
    goal_info_msg, destroy_ros_message);

  auto handle = rcl_action_accept_new_goal(action_server, goal_info_msg);
  if (!handle) {
    throw rclpy::RCLError("Failed to accept new goal");
  }

  rcl_action_goal_handle_ = std::shared_ptr<rcl_action_goal_handle_t>(
    handle,
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
    });
}

void
ActionGoalHandle::destroy()
{
  rcl_action_goal_handle_.reset();
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
  py::class_<ActionGoalHandle, Destroyable>(module, "ActionGoalHandle")
  .def(py::init<py::capsule, py::object>())
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
