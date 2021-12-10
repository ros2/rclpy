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

namespace py = pybind11;

namespace
{
class LifecycleStateMachine : public rclpy::Destroyable, public std::enable_shared_from_this<LifecycleStateMachine>
{
public:
  LifecycleStateMachine(
    rclpy::Node node, bool enable_com_interface)
  : node_(std::move(node))
  {
    state_machine_ = rcl_lifecycle_get_zero_initialized_state_machine();
    auto state_machine_options = rcl_lifecycle_get_default_state_machine_options();
    state_machine_options.enable_com_interface = enable_com_interface;
    rcl_ret_t ret = rcl_lifecycle_state_machine_init(
      &state_machine_,
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
  }
  
  void init()
  {
    if (state_machine_.options.enable_com_interface) {
      
    }
    // if (enable_com_interface) {
    //   {
    //     srv_change_state_ = rclpy::Service(
    //       node,
    //       &state_machine_.com_interface.srv_change_state);
    //     node_services_interface_->add_service(
    //       std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_change_state_),
    //       nullptr);
    //   }

    //   { // get_state
    //     auto cb = std::bind(
    //       &LifecycleNodeInterfaceImpl::on_get_state, this,
    //       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    //     rclcpp::AnyServiceCallback<GetStateSrv> any_cb;
    //     any_cb.set(std::move(cb));

    //     srv_get_state_ = std::make_shared<rclcpp::Service<GetStateSrv>>(
    //       node_base_interface_->get_shared_rcl_node_handle(),
    //       &state_machine_.com_interface.srv_get_state,
    //       any_cb);
    //     node_services_interface_->add_service(
    //       std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_state_),
    //       nullptr);
    //   }

    //   { // get_available_states
    //     auto cb = std::bind(
    //       &LifecycleNodeInterfaceImpl::on_get_available_states, this,
    //       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    //     rclcpp::AnyServiceCallback<GetAvailableStatesSrv> any_cb;
    //     any_cb.set(std::move(cb));

    //     srv_get_available_states_ = std::make_shared<rclcpp::Service<GetAvailableStatesSrv>>(
    //       node_base_interface_->get_shared_rcl_node_handle(),
    //       &state_machine_.com_interface.srv_get_available_states,
    //       any_cb);
    //     node_services_interface_->add_service(
    //       std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_available_states_),
    //       nullptr);
    //   }

    //   { // get_available_transitions
    //     auto cb = std::bind(
    //       &LifecycleNodeInterfaceImpl::on_get_available_transitions, this,
    //       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    //     rclcpp::AnyServiceCallback<GetAvailableTransitionsSrv> any_cb;
    //     any_cb.set(std::move(cb));

    //     srv_get_available_transitions_ =
    //       std::make_shared<rclcpp::Service<GetAvailableTransitionsSrv>>(
    //       node_base_interface_->get_shared_rcl_node_handle(),
    //       &state_machine_.com_interface.srv_get_available_transitions,
    //       any_cb);
    //     node_services_interface_->add_service(
    //       std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_available_transitions_),
    //       nullptr);
    //   }

    //   { // get_transition_graph
    //     auto cb = std::bind(
    //       &LifecycleNodeInterfaceImpl::on_get_transition_graph, this,
    //       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    //     rclcpp::AnyServiceCallback<GetAvailableTransitionsSrv> any_cb;
    //     any_cb.set(std::move(cb));

    //     srv_get_transition_graph_ =
    //       std::make_shared<rclcpp::Service<GetAvailableTransitionsSrv>>(
    //       node_base_interface_->get_shared_rcl_node_handle(),
    //       &state_machine_.com_interface.srv_get_transition_graph,
    //       any_cb);
    //     node_services_interface_->add_service(
    //       std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_transition_graph_),
    //       nullptr);
    //   }
    // }
  }

  ~LifecycleStateMachine()
  {

  }

  bool
  is_initialized()
  {
    return RCL_RET_OK == rcl_lifecycle_state_machine_is_initialized(&state_machine_);
  }

  void
  trigger_transition_by_id(uint8_t transition_id, bool publish_update)
  {
    if (
      rcl_lifecycle_trigger_transition_by_id(
        &state_machine_, transition_id, publish_update) != RCL_RET_OK)
    {
      throw rclpy::RCLError("Failed to trigger lifecycle state machine transition");;
    }
  }

  void
  trigger_transition_by_label(std::string label, bool publish_update)
  {
    if (
      rcl_lifecycle_trigger_transition_by_label(
        &state_machine_, label.c_str(), publish_update) != RCL_RET_OK)
    {
      throw rclpy::RCLError("Failed to trigger lifecycle state machine transition");;
    }
  }

  void
  print()
  {
    rcl_print_state_machine(&state_machine_);
  }

private:
  rclpy::Node node_;
  rcl_lifecycle_state_machine_t state_machine_;
};
}

namespace rclpy
{
void
define_lifecycle_api(py::module m)
{
}
}  // namespace rclpy
