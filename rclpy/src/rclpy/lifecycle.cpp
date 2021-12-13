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

#include <memory>

#include <pybind11/pybind11.h>

#include <lifecycle_msgs/msg/transition_event.h>
#include <lifecycle_msgs/srv/change_state.h>
#include <lifecycle_msgs/srv/get_state.h>
#include <lifecycle_msgs/srv/get_available_states.h>
#include <lifecycle_msgs/srv/get_available_transitions.h>

#include <rcl_lifecycle/rcl_lifecycle.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>

#include "destroyable.hpp"
#include "exceptions.hpp"
#include "lifecycle.hpp"
#include "node.hpp"
#include "service.hpp"

namespace py = pybind11;

namespace
{
class LifecycleStateMachine : public rclpy::Destroyable
{
public:
  LifecycleStateMachine(
    rclpy::Node node, bool enable_com_interface)
  : node_(std::move(node))
  {
    state_machine_ = std::shared_ptr<rcl_lifecycle_state_machine_t>(
      new rcl_lifecycle_state_machine_t(rcl_lifecycle_get_zero_initialized_state_machine()),
      [node] (rcl_lifecycle_state_machine_t * state_machine) {
        rcl_ret_t ret = rcl_lifecycle_state_machine_fini(state_machine, node.rcl_ptr());
        if (RCL_RET_OK != ret) {
          // Warning should use line number of the current stack frame
          int stack_level = 1;
          PyErr_WarnFormat(
            PyExc_RuntimeWarning, stack_level, "Failed to fini lifecycle state machine: %s",
            rcl_get_error_string().str);
          rcl_reset_error();
        }
      });
    auto state_machine_options = rcl_lifecycle_get_default_state_machine_options();
    state_machine_options.enable_com_interface = enable_com_interface;
    rcl_ret_t ret = rcl_lifecycle_state_machine_init(
      state_machine_.get(),
      node_.rcl_ptr(),
      ROSIDL_GET_MSG_TYPE_SUPPORT(lifecycle_msgs, msg, TransitionEvent),
      ROSIDL_GET_SRV_TYPE_SUPPORT(lifecycle_msgs, srv, ChangeState),
      ROSIDL_GET_SRV_TYPE_SUPPORT(lifecycle_msgs, srv, GetState),
      ROSIDL_GET_SRV_TYPE_SUPPORT(lifecycle_msgs, srv, GetAvailableStates),
      ROSIDL_GET_SRV_TYPE_SUPPORT(lifecycle_msgs, srv, GetAvailableTransitions),
      ROSIDL_GET_SRV_TYPE_SUPPORT(lifecycle_msgs, srv, GetAvailableTransitions),
      &state_machine_options);
    if (RCL_RET_OK != ret) {
      throw rclpy::RCLError("Failed to create lifecycle state machine");
    }
    if (state_machine_->options.enable_com_interface) {
      srv_change_state_ = std::make_shared<rclpy::Service>(
        node_,
        std::shared_ptr<rcl_service_t>(
          state_machine_, &state_machine_->com_interface.srv_change_state));
      srv_get_state_ = std::make_shared<rclpy::Service>(
        node_,
        std::shared_ptr<rcl_service_t>(
          state_machine_, &state_machine_->com_interface.srv_get_state));
      srv_get_available_states_ = std::make_shared<rclpy::Service>(
        node_,
        std::shared_ptr<rcl_service_t>(
          state_machine_, &state_machine_->com_interface.srv_get_available_states));
      srv_get_available_transitions_ = std::make_shared<rclpy::Service>(
        node_,
        std::shared_ptr<rcl_service_t>(
          state_machine_, &state_machine_->com_interface.srv_get_available_transitions));
      srv_get_transition_graph_ = std::make_shared<rclpy::Service>(
        node_,
        std::shared_ptr<rcl_service_t>(
          state_machine_, &state_machine_->com_interface.srv_get_transition_graph));
    }
  }

  ~LifecycleStateMachine()
  {
    this->destroy();
  }

  void
  destroy() override
  {
    srv_change_state_.reset();
    srv_get_state_.reset();
    srv_get_available_states_.reset();
    srv_get_available_transitions_.reset();
    srv_get_transition_graph_.reset();
    state_machine_.reset();
    node_.destroy();
  }

  bool
  is_initialized()
  {
    return RCL_RET_OK == rcl_lifecycle_state_machine_is_initialized(state_machine_.get());
  }

  void
  trigger_transition_by_id(uint8_t transition_id, bool publish_update)
  {
    if (
      rcl_lifecycle_trigger_transition_by_id(
        state_machine_.get(), transition_id, publish_update) != RCL_RET_OK)
    {
      throw rclpy::RCLError("Failed to trigger lifecycle state machine transition");;
    }
  }

  void
  trigger_transition_by_label(std::string label, bool publish_update)
  {
    if (
      rcl_lifecycle_trigger_transition_by_label(
        state_machine_.get(), label.c_str(), publish_update) != RCL_RET_OK)
    {
      throw rclpy::RCLError("Failed to trigger lifecycle state machine transition");;
    }
  }

  void
  print()
  {
    rcl_print_state_machine(state_machine_.get());
  }

  std::shared_ptr<rclpy::Service>
  get_srv_change_state()
  {
    return srv_change_state_;
  }

  std::shared_ptr<rclpy::Service>
  get_srv_get_state()
  {
    return srv_get_state_;
  }

  std::shared_ptr<rclpy::Service>
  get_srv_get_available_states()
  {
    return srv_get_available_states_;
  }

  std::shared_ptr<rclpy::Service>
  get_srv_get_available_transitions()
  {
    return srv_get_available_transitions_;
  }

  std::shared_ptr<rclpy::Service>
  get_srv_get_transition_graph()
  {
    return srv_get_transition_graph_;
  }

private:
  rclpy::Node node_;
  std::shared_ptr<rclpy::Service> srv_change_state_;
  std::shared_ptr<rclpy::Service> srv_get_state_;
  std::shared_ptr<rclpy::Service> srv_get_available_states_;
  std::shared_ptr<rclpy::Service> srv_get_available_transitions_;
  std::shared_ptr<rclpy::Service> srv_get_transition_graph_;
  std::shared_ptr<rcl_lifecycle_state_machine_t> state_machine_;
};

enum class TransitionCallbackReturnType
{
  Success = lifecycle_msgs__msg__Transition__TRANSITION_CALLBACK_SUCCESS,
  Failure = lifecycle_msgs__msg__Transition__TRANSITION_CALLBACK_FAILURE,
  Error = lifecycle_msgs__msg__Transition__TRANSITION_CALLBACK_ERROR,
};

}

namespace rclpy
{
void
define_lifecycle_api(py::module m)
{
  py::class_<LifecycleStateMachine, Destroyable>(
    m, "LifecycleStateMachine")
    .def(py::init<Node, bool>())
    .def(
      "is_initialized", &LifecycleStateMachine::is_initialized,
      "Check if state machine is initialized.")
    .def(
      "trigger_transition_by_id", &LifecycleStateMachine::trigger_transition_by_id,
      "Trigger a transition by transition id.")
    .def(
      "trigger_transition_by_label", &LifecycleStateMachine::trigger_transition_by_id,
      "Trigger a transition by label.")
    .def(
      "get_srv_change_state", &LifecycleStateMachine::get_srv_change_state,
      "Get the change state service.")
    .def(
      "get_srv_get_state", &LifecycleStateMachine::get_srv_get_state,
      "Get the get state service.")
    .def(
      "get_srv_get_available_states", &LifecycleStateMachine::get_srv_get_available_states,
      "Get the get available states service.")
    .def(
      "get_srv_get_available_transitions", &LifecycleStateMachine::get_srv_get_available_transitions,
      "Get the get available transitions service.")
    .def(
      "get_srv_get_transition_graph", &LifecycleStateMachine::get_srv_get_transition_graph,
      "Get the get transition graph service.");
  py::enum_<TransitionCallbackReturnType>(m, "TransitionCallbackReturnType")
    .value("SUCCESS", TransitionCallbackReturnType::Success)
    .value("FAILURE", TransitionCallbackReturnType::Failure)
    .value("ERROR", TransitionCallbackReturnType::Error);
}
}  // namespace rclpy
