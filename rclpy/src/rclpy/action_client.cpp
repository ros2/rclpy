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
#include <rcl/types.h>
#include <rcl_action/action_client.h>
#include <rcl_action/wait.h>
#include <rosidl_runtime_c/action_type_support_struct.h>
#include <rmw/types.h>

#include <memory>
#include <string>

#include "action_client.hpp"
#include "exceptions.hpp"
#include "node.hpp"
#include "utils.hpp"

namespace rclpy
{

void
ActionClient::destroy()
{
  rcl_action_client_.reset();
  node_.destroy();
}

ActionClient::ActionClient(
  Node & node,
  py::object pyaction_type,
  const char * action_name,
  const rmw_qos_profile_t & goal_service_qos,
  const rmw_qos_profile_t & result_service_qos,
  const rmw_qos_profile_t & cancel_service_qos,
  const rmw_qos_profile_t & feedback_topic_qos,
  const rmw_qos_profile_t & status_topic_qos)
: node_(node)
{
  rosidl_action_type_support_t * ts =
    static_cast<rosidl_action_type_support_t *>(common_get_type_support(pyaction_type));
  if (!ts) {
    throw py::error_already_set();
  }

  rcl_action_client_options_t action_client_ops = rcl_action_client_get_default_options();

  action_client_ops.goal_service_qos = goal_service_qos;
  action_client_ops.result_service_qos = result_service_qos;
  action_client_ops.cancel_service_qos = cancel_service_qos;
  action_client_ops.feedback_topic_qos = feedback_topic_qos;
  action_client_ops.status_topic_qos = status_topic_qos;

  rcl_action_client_ = std::shared_ptr<rcl_action_client_t>(
    new rcl_action_client_t,
    [node](rcl_action_client_t * action_client)
    {
      // Intentionally capture node by value so shared_ptr can be transferred to copies
      rcl_ret_t ret = rcl_action_client_fini(action_client, node.rcl_ptr());
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini publisher: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete action_client;
    });

  *rcl_action_client_ = rcl_action_get_zero_initialized_client();

  rcl_ret_t ret = rcl_action_client_init(
    rcl_action_client_.get(),
    node_.rcl_ptr(),
    ts,
    action_name,
    &action_client_ops);
  if (RCL_RET_ACTION_NAME_INVALID == ret) {
    std::string error_text{"Failed to create action client due to invalid topic name '"};
    error_text += action_name;
    error_text += "' : ";
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw py::value_error(error_text);
  }
  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to create action client: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw py::value_error(error_text);
  }
}

#define TAKE_SERVICE_RESPONSE(Type) \
  /* taken_msg is always destroyed in this function */ \
  auto taken_msg = create_from_py(pymsg_type); \
  rmw_request_id_t header; \
  rcl_ret_t ret = rcl_action_take_ ## Type ## _response( \
    rcl_action_client_.get(), &header, taken_msg.get()); \
  int64_t sequence = header.sequence_number; \
  /* Create the tuple to return */ \
  if (RCL_RET_ACTION_CLIENT_TAKE_FAILED == ret || RCL_RET_ACTION_SERVER_TAKE_FAILED == ret) { \
    return py::make_tuple(py::none(), py::none()); \
  } else if (RCL_RET_OK != ret) { \
    throw rclpy::RCLError("Failed to take " #Type); \
  } \
  return py::make_tuple(sequence, convert_to_py(taken_msg.get(), pymsg_type)); \

py::tuple
ActionClient::take_goal_response(py::object pymsg_type)
{
  TAKE_SERVICE_RESPONSE(goal)
}

#define SEND_SERVICE_REQUEST(Type) \
  auto ros_request = convert_from_py(pyrequest); \
  int64_t sequence_number; \
  rcl_ret_t ret = rcl_action_send_ ## Type ## _request( \
    rcl_action_client_.get(), ros_request.get(), &sequence_number); \
  if (RCL_RET_OK != ret) { \
    throw rclpy::RCLError("Failed to send " #Type " request"); \
  } \
  return sequence_number;

int64_t
ActionClient::send_result_request(py::object pyrequest)
{
  SEND_SERVICE_REQUEST(result);
}

py::tuple
ActionClient::take_cancel_response(py::object pymsg_type)
{
  TAKE_SERVICE_RESPONSE(cancel)
}

#define TAKE_MESSAGE(Type) \
  auto taken_msg = create_from_py(pymsg_type); \
  rcl_ret_t ret = rcl_action_take_ ## Type(rcl_action_client_.get(), taken_msg.get()); \
  if (RCL_RET_OK != ret) { \
    if (RCL_RET_ACTION_CLIENT_TAKE_FAILED == ret) { \
      /* if take failed, just do nothing */ \
      return py::none(); \
    } \
    throw rclpy::RCLError("Failed to take " #Type " with an action client"); \
  } \
  return convert_to_py(taken_msg.get(), pymsg_type);

py::object
ActionClient::take_feedback(py::object pymsg_type)
{
  TAKE_MESSAGE(feedback)
}

py::object
ActionClient::take_status(py::object pymsg_type)
{
  TAKE_MESSAGE(status)
}

int64_t
ActionClient::send_cancel_request(py::object pyrequest)
{
  SEND_SERVICE_REQUEST(cancel)
}

int64_t
ActionClient::send_goal_request(py::object pyrequest)
{
  SEND_SERVICE_REQUEST(goal)
}

py::tuple
ActionClient::take_result_response(py::object pymsg_type)
{
  TAKE_SERVICE_RESPONSE(result);
}

py::tuple
ActionClient::get_num_entities()
{
  size_t num_subscriptions = 0u;
  size_t num_guard_conditions = 0u;
  size_t num_timers = 0u;
  size_t num_clients = 0u;
  size_t num_services = 0u;

  rcl_ret_t ret;
  ret = rcl_action_client_wait_set_get_num_entities(
    rcl_action_client_.get(),
    &num_subscriptions,
    &num_guard_conditions,
    &num_timers,
    &num_clients,
    &num_services);
  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to get number of entities for 'rcl_action_client_t'"};
    throw rclpy::RCLError(error_text);
  }

  return py::make_tuple(
    num_subscriptions, num_guard_conditions, num_timers,
    num_clients, num_services);
}

bool
ActionClient::is_action_server_available()
{
  bool is_available = false;
  rcl_ret_t ret = rcl_action_server_is_available(
    node_.rcl_ptr(), rcl_action_client_.get(), &is_available);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to check if action server is available");
  }
  return is_available;
}

void
ActionClient::add_to_waitset(WaitSet & wait_set)
{
  rcl_ret_t ret = rcl_action_wait_set_add_action_client(
    wait_set.rcl_ptr(), rcl_action_client_.get(), NULL, NULL);
  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to add 'rcl_action_client_t' to wait set"};
    throw rclpy::RCLError(error_text);
  }
}

py::tuple
ActionClient::is_ready(WaitSet & wait_set)
{
  bool is_feedback_ready = false;
  bool is_status_ready = false;
  bool is_goal_response_ready = false;
  bool is_cancel_response_ready = false;
  bool is_result_response_ready = false;
  rcl_ret_t ret = rcl_action_client_wait_set_get_entities_ready(
    wait_set.rcl_ptr(),
    rcl_action_client_.get(),
    &is_feedback_ready,
    &is_status_ready,
    &is_goal_response_ready,
    &is_cancel_response_ready,
    &is_result_response_ready);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get number of ready entities for action client");
  }

  py::tuple result_tuple(5);
  result_tuple[0] = py::bool_(is_feedback_ready);
  result_tuple[1] = py::bool_(is_status_ready);
  result_tuple[2] = py::bool_(is_goal_response_ready);
  result_tuple[3] = py::bool_(is_cancel_response_ready);
  result_tuple[4] = py::bool_(is_result_response_ready);
  return result_tuple;
}

void
define_action_client(py::object module)
{
  py::class_<ActionClient, Destroyable, std::shared_ptr<ActionClient>>(module, "ActionClient")
  .def(
    py::init<Node &, py::object, const char *, const rmw_qos_profile_t &,
    const rmw_qos_profile_t &, const rmw_qos_profile_t &,
    const rmw_qos_profile_t &, const rmw_qos_profile_t &>())
  .def_property_readonly(
    "pointer", [](const ActionClient & action_client) {
      return reinterpret_cast<size_t>(action_client.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "take_goal_response", &ActionClient::take_goal_response,
    "Take an action goal response.")
  .def(
    "send_result_request", &ActionClient::send_result_request,
    "Send an action result request.")
  .def(
    "take_cancel_response", &ActionClient::take_cancel_response,
    "Take an action cancel response.")
  .def(
    "take_feedback", &ActionClient::take_feedback,
    "Take a feedback message from a given action client.")
  .def(
    "send_cancel_request", &ActionClient::send_cancel_request,
    "Send an action cancel request.")
  .def(
    "send_goal_request", &ActionClient::send_goal_request,
    "Send an action goal request.")
  .def(
    "take_result_response", &ActionClient::take_result_response,
    "Take an action result response.")
  .def(
    "get_num_entities", &ActionClient::get_num_entities,
    "Get the number of wait set entities that make up an action entity.")
  .def(
    "is_action_server_available", &ActionClient::is_action_server_available,
    "Check if an action server is available for the given action client.")
  .def(
    "add_to_waitset", &ActionClient::add_to_waitset,
    "Add an action entity to a wait set.")
  .def(
    "is_ready", &ActionClient::is_ready,
    "Check if an action entity has any ready wait set entities.")
  .def(
    "take_status", &ActionClient::take_status,
    "Take an action status response.");
}
}  // namespace rclpy
