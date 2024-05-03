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

#include <rcl_action/action_server.h>
#include <rcl_action/wait.h>
#include <rcl/error_handling.h>
#include <rcl/time.h>
#include <rcl/types.h>
#include <rosidl_runtime_c/action_type_support_struct.h>
#include <rmw/types.h>

#include <memory>
#include <stdexcept>
#include <string>

#include <rcpputils/scope_exit.hpp>

#include "action_server.hpp"
#include "clock.hpp"
#include "exceptions.hpp"
#include "node.hpp"

namespace rclpy
{

void
ActionServer::destroy()
{
  rcl_action_server_.reset();
  node_.destroy();
}

ActionServer::ActionServer(
  Node & node,
  const rclpy::Clock & rclpy_clock,
  py::object pyaction_type,
  const char * action_name,
  const rmw_qos_profile_t & goal_service_qos,
  const rmw_qos_profile_t & result_service_qos,
  const rmw_qos_profile_t & cancel_service_qos,
  const rmw_qos_profile_t & feedback_topic_qos,
  const rmw_qos_profile_t & status_topic_qos,
  double result_timeout)
: node_(node)
{
  rcl_clock_t * clock = rclpy_clock.rcl_ptr();

  rosidl_action_type_support_t * ts = static_cast<rosidl_action_type_support_t *>(
    common_get_type_support(pyaction_type));
  if (!ts) {
    throw py::error_already_set();
  }

  rcl_action_server_options_t action_server_ops = rcl_action_server_get_default_options();

  action_server_ops.goal_service_qos = goal_service_qos;
  action_server_ops.result_service_qos = result_service_qos;
  action_server_ops.cancel_service_qos = cancel_service_qos;
  action_server_ops.feedback_topic_qos = feedback_topic_qos;
  action_server_ops.status_topic_qos = status_topic_qos;
  action_server_ops.result_timeout.nanoseconds = (rcl_duration_value_t)RCL_S_TO_NS(result_timeout);

  rcl_action_server_ = std::shared_ptr<rcl_action_server_t>(
    new rcl_action_server_t,
    [node](rcl_action_server_t * action_server)
    {
      // Intentionally capture node by value so shared_ptr can be transferred to copies
      rcl_ret_t ret = rcl_action_server_fini(action_server, node.rcl_ptr());
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini publisher: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete action_server;
    });

  *rcl_action_server_ = rcl_action_get_zero_initialized_server();

  rcl_ret_t ret = rcl_action_server_init(
    rcl_action_server_.get(),
    node_.rcl_ptr(),
    clock,
    ts,
    action_name,
    &action_server_ops);
  if (RCL_RET_ACTION_NAME_INVALID == ret) {
    std::string error_text{"Failed to create action server due to invalid topic name '"};
    error_text += action_name;
    error_text += "' : ";
    throw py::value_error(append_rcl_error(error_text));
  } else if (RCL_RET_OK != ret) {
    throw py::value_error(append_rcl_error("Failed to create action server"));
  }
}

#define TAKE_SERVICE_REQUEST(Type) \
  /* taken_msg is always destroyed in this function */ \
  auto taken_msg = create_from_py(pymsg_type); \
  rmw_request_id_t header; \
  rcl_ret_t ret = \
    rcl_action_take_ ## Type ## _request(rcl_action_server_.get(), &header, taken_msg.get()); \
  /* Create the tuple to return */ \
  if (ret == RCL_RET_ACTION_CLIENT_TAKE_FAILED || ret == RCL_RET_ACTION_SERVER_TAKE_FAILED) { \
    return py::make_tuple(py::none(), py::none()); \
  } else if (RCL_RET_OK != ret) { \
    throw rclpy::RCLError("Failed to take " #Type); \
  } \
  return py::make_tuple(header, convert_to_py(taken_msg.get(), pymsg_type)); \

py::tuple
ActionServer::take_goal_request(py::object pymsg_type)
{
  TAKE_SERVICE_REQUEST(goal)
}

py::tuple
ActionServer::take_result_request(py::object pymsg_type)
{
  TAKE_SERVICE_REQUEST(result)
}

#define SEND_SERVICE_RESPONSE(Type) \
  auto ros_response = convert_from_py(pyresponse); \
  rcl_ret_t ret = rcl_action_send_ ## Type ## _response( \
    rcl_action_server_.get(), header, ros_response.get()); \
  if (RCL_RET_OK != ret) { \
    if (RCL_RET_TIMEOUT == ret) { \
      int stack_level = 1; \
      PyErr_WarnFormat( \
        PyExc_RuntimeWarning, stack_level, "failed to send response (timeout): %s", \
        rcl_get_error_string().str); \
      rcl_reset_error(); \
    } else { \
      throw rclpy::RCLError("Failed to send " #Type " response"); \
    } \
  }

void
ActionServer::send_goal_response(
  rmw_request_id_t * header, py::object pyresponse)
{
  SEND_SERVICE_RESPONSE(goal)
}

void
ActionServer::send_result_response(
  rmw_request_id_t * header, py::object pyresponse)
{
  SEND_SERVICE_RESPONSE(result)
}

py::tuple
ActionServer::take_cancel_request(py::object pymsg_type)
{
  TAKE_SERVICE_REQUEST(cancel)
}

void
ActionServer::send_cancel_response(
  rmw_request_id_t * header, py::object pyresponse)
{
  SEND_SERVICE_RESPONSE(cancel)
}

void
ActionServer::publish_feedback(py::object pymsg)
{
  auto ros_message = convert_from_py(pymsg);
  rcl_ret_t ret = rcl_action_publish_feedback(rcl_action_server_.get(), ros_message.get());
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to publish feedback with an action server");
  }
}

void
ActionServer::publish_status()
{
  rcl_action_goal_status_array_t status_message =
    rcl_action_get_zero_initialized_goal_status_array();
  rcl_ret_t ret = rcl_action_get_goal_status_array(rcl_action_server_.get(), &status_message);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed get goal status array");
  }

  RCPPUTILS_SCOPE_EXIT(
    {
      ret = rcl_action_goal_status_array_fini(&status_message);

      if (RCL_RET_OK != ret) {
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to finalize goal status array: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
    });

  ret = rcl_action_publish_status(rcl_action_server_.get(), &status_message);

  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed publish goal status array");
  }
}

void
ActionServer::notify_goal_done()
{
  rcl_ret_t ret = rcl_action_notify_goal_done(rcl_action_server_.get());
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to notfiy action server of goal done");
  }
}

bool
ActionServer::goal_exists(py::object pygoal_info)
{
  auto goal_info = convert_from_py(pygoal_info);
  rcl_action_goal_info_t * goal_info_type = static_cast<rcl_action_goal_info_t *>(goal_info.get());
  return rcl_action_server_goal_exists(rcl_action_server_.get(), goal_info_type);
}

py::tuple
ActionServer::get_num_entities()
{
  size_t num_subscriptions = 0u;
  size_t num_guard_conditions = 0u;
  size_t num_timers = 0u;
  size_t num_clients = 0u;
  size_t num_services = 0u;

  rcl_ret_t ret = rcl_action_server_wait_set_get_num_entities(
    rcl_action_server_.get(),
    &num_subscriptions,
    &num_guard_conditions,
    &num_timers,
    &num_clients,
    &num_services);

  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get number of entities for 'rcl_action_server_t'");
  }

  py::tuple result_tuple(5);
  result_tuple[0] = py::int_(num_subscriptions);
  result_tuple[1] = py::int_(num_guard_conditions);
  result_tuple[2] = py::int_(num_timers);
  result_tuple[3] = py::int_(num_clients);
  result_tuple[4] = py::int_(num_services);
  return result_tuple;
}

py::tuple
ActionServer::is_ready(WaitSet & wait_set)
{
  bool is_goal_request_ready = false;
  bool is_cancel_request_ready = false;
  bool is_result_request_ready = false;
  bool is_goal_expired = false;
  rcl_ret_t ret = rcl_action_server_wait_set_get_entities_ready(
    wait_set.rcl_ptr(),
    rcl_action_server_.get(),
    &is_goal_request_ready,
    &is_cancel_request_ready,
    &is_result_request_ready,
    &is_goal_expired);

  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get number of ready entities for action server");
  }

  py::tuple result_tuple(4);
  result_tuple[0] = py::bool_(is_goal_request_ready);
  result_tuple[1] = py::bool_(is_cancel_request_ready);
  result_tuple[2] = py::bool_(is_result_request_ready);
  result_tuple[3] = py::bool_(is_goal_expired);
  return result_tuple;
}

void
ActionServer::add_to_waitset(WaitSet & wait_set)
{
  rcl_ret_t ret = rcl_action_wait_set_add_action_server(
    wait_set.rcl_ptr(), rcl_action_server_.get(), NULL);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to add 'rcl_action_server_t' to wait set");
  }
}

py::object
ActionServer::process_cancel_request(
  py::object pycancel_request, py::object pycancel_response_type)
{
  auto cancel_request = convert_from_py(pycancel_request);
  rcl_action_cancel_request_t * cancel_request_tmp = static_cast<rcl_action_cancel_request_t *>(
    cancel_request.get());

  rcl_action_cancel_response_t cancel_response = rcl_action_get_zero_initialized_cancel_response();
  rcl_ret_t ret = rcl_action_process_cancel_request(
    rcl_action_server_.get(), cancel_request_tmp, &cancel_response);

  if (RCL_RET_OK != ret) {
    std::string error_text = append_rcl_error("Failed to process cancel request");
    ret = rcl_action_cancel_response_fini(&cancel_response);
    if (RCL_RET_OK != ret) {
      error_text = append_rcl_error(".  Also failed to cleanup response");
    }
    throw std::runtime_error(error_text);
  }

  py::object return_value = convert_to_py(&cancel_response.msg, pycancel_response_type);
  RCPPUTILS_SCOPE_EXIT(
    {
      ret = rcl_action_cancel_response_fini(&cancel_response);

      if (RCL_RET_OK != ret) {
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to finalize cancel response: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
    });
  return return_value;
}

py::tuple
ActionServer::expire_goals(int64_t max_num_goals)
{
  auto expired_goals =
    std::unique_ptr<rcl_action_goal_info_t[]>(new rcl_action_goal_info_t[max_num_goals]);
  size_t num_expired;
  rcl_ret_t ret = rcl_action_expire_goals(
    rcl_action_server_.get(), expired_goals.get(), max_num_goals, &num_expired);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to expire goals");
  }

  // Get Python GoalInfo type
  py::module pyaction_msgs_module = py::module::import("action_msgs.msg");
  py::object pygoal_info_class = pyaction_msgs_module.attr("GoalInfo");
  py::object pygoal_info_type = pygoal_info_class();

  // Create a tuple of GoalInfo instances to return
  py::tuple result_tuple(num_expired);

  for (size_t i = 0; i < num_expired; ++i) {
    result_tuple[i] =
      convert_to_py(&(expired_goals.get()[i]), pygoal_info_type);
  }

  return result_tuple;
}

void
define_action_server(py::object module)
{
  py::class_<ActionServer, Destroyable, std::shared_ptr<ActionServer>>(module, "ActionServer")
  .def(
    py::init<Node &, const rclpy::Clock &, py::object, const char *,
    const rmw_qos_profile_t &, const rmw_qos_profile_t &, const rmw_qos_profile_t &,
    const rmw_qos_profile_t &, const rmw_qos_profile_t &, double>())
  .def_property_readonly(
    "pointer", [](const ActionServer & action_server) {
      return reinterpret_cast<size_t>(action_server.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "take_goal_request", &ActionServer::take_goal_request,
    "Take an action goal request.")
  .def(
    "send_goal_response", &ActionServer::send_goal_response,
    "Send an action goal response.")
  .def(
    "send_result_response", &ActionServer::send_result_response,
    "Send an action result response.")
  .def(
    "take_cancel_request", &ActionServer::take_cancel_request,
    "Take an action cancel request.")
  .def(
    "take_result_request", &ActionServer::take_result_request,
    "Take an action result request.")
  .def(
    "send_cancel_response", &ActionServer::send_cancel_response,
    "Send an action cancel response.")
  .def(
    "publish_feedback", &ActionServer::publish_feedback,
    " Publish a feedback message from a given action server.")
  .def(
    "publish_status", &ActionServer::publish_status,
    "Publish a status message from a given action server.")
  .def(
    "notify_goal_done", &ActionServer::notify_goal_done,
    "Notify goal is done.")
  .def(
    "goal_exists", &ActionServer::goal_exists,
    "Check is a goal exists in the server.")
  .def(
    "process_cancel_request", &ActionServer::process_cancel_request,
    "Process a cancel request")
  .def(
    "expire_goals", &ActionServer::expire_goals,
    "Expired goals.")
  .def(
    "get_num_entities", &ActionServer::get_num_entities,
    "Get the number of wait set entities that make up an action entity.")
  .def(
    "is_ready", &ActionServer::is_ready,
    "Check if an action entity has any ready wait set entities.")
  .def(
    "add_to_waitset", &ActionServer::add_to_waitset,
    "Add an action entitiy to a wait set.");
}

}  // namespace rclpy
