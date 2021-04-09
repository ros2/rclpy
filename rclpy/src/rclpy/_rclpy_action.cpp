// Copyright 2019 Open Source Robotics Foundation, Inc.
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
#include <rcl_action/rcl_action.h>

#include <cstring>
#include <memory>
#include <string>

#include "rclpy_common/exceptions.hpp"

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"

#include "action_goal_handle.hpp"
#include "action_server.hpp"
#include "clock.hpp"

namespace py = pybind11;


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


/// Destroy an rcl_action entity.
/**
 * Raises RuntimeError on failure.
 *
 * \param[in] pyentity Capsule pointing to the entity to destroy.
 * \param[in] pynode Capsule pointing to the node the action client was added to.
 */
void
rclpy_action_destroy_entity(py::capsule pyentity, py::capsule pynode)
{
  rcl_node_t * node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_ret_t ret;
  if (0 == std::strcmp("rcl_action_client_t", pyentity.name())) {
    auto action_client = get_pointer<rcl_action_client_t *>(pyentity, "rcl_action_client_t");
    ret = rcl_action_client_fini(action_client, node);
    PyMem_Free(action_client);
  } else {
    std::string entity_name = pyentity.name();
    throw std::runtime_error(entity_name + " is not a known entity");
  }

  if (ret != RCL_RET_OK) {
    std::string error_text = "Failed to fini '";
    error_text += pyentity.name();
    error_text += "'";
    throw rclpy::RCLError(error_text);
  }

  if (PyCapsule_SetName(pyentity.ptr(), "_destroyed_by_rclpy_action_destroy_entity_")) {
    throw py::error_already_set();
  }
}

/// Fetch a predefined qos_profile from rcl_action and convert it to a Python QoSProfile object.
/**
 * Raises RuntimeError if the QoS profile is unknown.
 *
 * This function takes a string defining a rmw_qos_profile_t and returns the
 * corresponding Python QoSProfile object.
 * \param[in] rmw_profile String with the name of the profile to load.
 * \return QoSProfile object.
 */
py::dict
rclpy_action_get_rmw_qos_profile(const char * rmw_profile)
{
  PyObject * pyqos_profile = NULL;
  if (0 == strcmp(rmw_profile, "rcl_action_qos_profile_status_default")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rcl_action_qos_profile_status_default);
  } else {
    std::string error_text = "Requested unknown rmw_qos_profile: ";
    error_text += rmw_profile;
    throw std::runtime_error(error_text);
  }
  return py::reinterpret_steal<py::dict>(pyqos_profile);
}

/// Add an action entitiy to a wait set.
/**
 * Raises RuntimeError on failure.
 * \param[in] pyentity Capsule pointer to an action entity
 *   (rcl_action_client_t or rcl_action_server_t).
 * \param[in] pywait_set Capsule pointer to an rcl_wait_set_t.
 */
void
rclpy_action_wait_set_add(py::capsule pyentity, py::capsule pywait_set)
{
  auto wait_set = get_pointer<rcl_wait_set_t *>(pywait_set, "rcl_wait_set_t");

  rcl_ret_t ret;
  if (0 == strcmp(pyentity.name(), "rcl_action_client_t")) {
    auto action_client = get_pointer<rcl_action_client_t *>(pyentity, "rcl_action_client_t");
    ret = rcl_action_wait_set_add_action_client(wait_set, action_client, NULL, NULL);
  } else if (0 == strcmp(pyentity.name(), "rcl_action_server_t")) {
    auto action_server = get_pointer<rcl_action_server_t *>(pyentity, "rcl_action_server_t");
    ret = rcl_action_wait_set_add_action_server(wait_set, action_server, NULL);
  } else {
    std::string error_text{"Unknown entity: "};
    error_text += pyentity.name();
    throw std::runtime_error(error_text);
  }

  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to add '"};
    error_text += pyentity.name();
    error_text += "' to wait set";
    throw rclpy::RCLError(error_text);
  }
}

/// Get the number of wait set entities that make up an action entity.
/**
 * \param[in] pyentity Capsule pointer to an action entity
 *   (rcl_action_client_t or rcl_action_server_t).
 * \return Tuple containing the number of wait set entities:
 *   (num_subscriptions,
 *    num_guard_conditions,
 *    num_timers,
 *    num_clients,
 *    num_services)
 */
py::tuple
rclpy_action_wait_set_get_num_entities(py::capsule pyentity)
{
  size_t num_subscriptions = 0u;
  size_t num_guard_conditions = 0u;
  size_t num_timers = 0u;
  size_t num_clients = 0u;
  size_t num_services = 0u;

  rcl_ret_t ret;
  if (0 == strcmp(pyentity.name(), "rcl_action_client_t")) {
    auto action_client = get_pointer<rcl_action_client_t *>(pyentity, "rcl_action_client_t");

    ret = rcl_action_client_wait_set_get_num_entities(
      action_client,
      &num_subscriptions,
      &num_guard_conditions,
      &num_timers,
      &num_clients,
      &num_services);
  } else {
    std::string error_text{"Unknown entity: "};
    error_text += pyentity.name();
    throw std::runtime_error(error_text);
  }

  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to get number of entities for '"};
    error_text += pyentity.name();
    error_text += "'";
    throw rclpy::RCLError(error_text);
  }

  py::tuple result_tuple(5);
  result_tuple[0] = py::int_(num_subscriptions);
  result_tuple[1] = py::int_(num_guard_conditions);
  result_tuple[2] = py::int_(num_timers);
  result_tuple[3] = py::int_(num_clients);
  result_tuple[4] = py::int_(num_services);
  return result_tuple;
}

/// Check if an action entity has any ready wait set entities.
/**
 * This must be called after waiting on the wait set.
 * Raises RuntimeError on failure.
 *
 * \param[in] entity Capsule pointing to the action entity
 *   (rcl_action_client_t or rcl_action_server_t).
 * \param[in] pywait_set Capsule pointing to the wait set structure.
 * \return A tuple of Bool representing the ready sub-entities.
 *     For a rcl_action_client_t:
 *       (is_feedback_ready,
 *        is_status_ready,
 *        is_goal_response_ready,
 *        is_cancel_response_ready,
 *        is_result_response_ready)
 *
 *     For a rcl_action_server_t:
 *       (is_goal_request_ready,
 *        is_cancel_request_ready,
 *        is_result_request_ready,
 *        is_goal_expired)
 */
py::tuple
rclpy_action_wait_set_is_ready(py::capsule pyentity, py::capsule pywait_set)
{
  auto wait_set = get_pointer<rcl_wait_set_t *>(pywait_set, "rcl_wait_set_t");

  if (0 == strcmp(pyentity.name(), "rcl_action_client_t")) {
    auto action_client = get_pointer<rcl_action_client_t *>(pyentity, "rcl_action_client_t");
    bool is_feedback_ready = false;
    bool is_status_ready = false;
    bool is_goal_response_ready = false;
    bool is_cancel_response_ready = false;
    bool is_result_response_ready = false;
    rcl_ret_t ret = rcl_action_client_wait_set_get_entities_ready(
      wait_set,
      action_client,
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

  std::string error_text{"Unknown entity: "};
  error_text += pyentity.name();
  throw std::runtime_error(error_text);
}

/// Create an action client.
/**
 * This function will create an action client for the given action name.
 * This client will use the typesupport defined in the action module
 * provided as pyaction_type to send messages over the wire.
 *
 * On a successful call a capsule referencing the created rcl_action_client_t structure
 * is returned.
 *
 * Raises ValueError if action name is invalid
 * Raises RuntimeError if the action client could not be created.
 *
 * \remark Call rclpy_action_destroy_entity() to destroy an action client.
 * \param[in] pynode Capsule pointing to the node to add the action client to.
 * \param[in] pyaction_type Action module associated with the action client.
 * \param[in] pyaction_name Python object containing the action name.
 * \param[in] goal_service_qos rmw_qos_profile_t object for the goal service.
 * \param[in] result_service_qos rmw_qos_profile_t object for the result service.
 * \param[in] cancel_service_qos rmw_qos_profile_t object for the cancel service.
 * \param[in] feedback_qos rmw_qos_profile_t object for the feedback subscriber.
 * \param[in] status_qos rmw_qos_profile_t object for the status subscriber.
 * \return Capsule named 'rcl_action_client_t', or
 * \return NULL on failure.
 */
py::capsule
rclpy_action_create_client(
  py::capsule pynode,
  py::object pyaction_type,
  const char * action_name,
  const rmw_qos_profile_t & goal_service_qos,
  const rmw_qos_profile_t & result_service_qos,
  const rmw_qos_profile_t & cancel_service_qos,
  const rmw_qos_profile_t & feedback_topic_qos,
  const rmw_qos_profile_t & status_topic_qos)
{
  rcl_node_t * node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rosidl_action_type_support_t * ts =
    static_cast<rosidl_action_type_support_t *>(rclpy_common_get_type_support(
      pyaction_type.ptr()));
  if (!ts) {
    throw py::error_already_set();
  }

  rcl_action_client_options_t action_client_ops = rcl_action_client_get_default_options();

  action_client_ops.goal_service_qos = goal_service_qos;
  action_client_ops.result_service_qos = result_service_qos;
  action_client_ops.cancel_service_qos = cancel_service_qos;
  action_client_ops.feedback_topic_qos = feedback_topic_qos;
  action_client_ops.status_topic_qos = status_topic_qos;

  auto deleter = [](rcl_action_client_t * ptr) {PyMem_Free(ptr);};
  auto action_client = std::unique_ptr<rcl_action_client_t, decltype(deleter)>(
    static_cast<rcl_action_client_t *>(PyMem_Malloc(sizeof(rcl_action_client_t))),
    deleter);
  if (!action_client) {
    throw std::bad_alloc();
  }

  *action_client = rcl_action_get_zero_initialized_client();
  rcl_ret_t ret = rcl_action_client_init(
    action_client.get(),
    node,
    ts,
    action_name,
    &action_client_ops);
  if (ret == RCL_RET_ACTION_NAME_INVALID) {
    std::string error_text{"Failed to create action client due to invalid topic name '"};
    error_text += action_name;
    error_text += "' : ";
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw py::value_error(error_text);
  } else if (ret != RCL_RET_OK) {
    std::string error_text{"Failed to create action client: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw py::value_error(error_text);
  }

  return py::capsule(action_client.release(), "rcl_action_client_t");
}

/// Check if an action server is available for the given action client.
/**
 * Raises RuntimeError on failure.
 *
 * \param[in] pynode Capsule pointing to the node to associated with the action client.
 * \param[in] pyaction_client The action client to use when checking for an available server.
 * \return True if an action server is available, False otherwise.
 */
bool
rclpy_action_server_is_available(py::capsule pynode, py::capsule pyaction_client)
{
  rcl_node_t * node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  auto action_client = get_pointer<rcl_action_client_t *>(pyaction_client, "rcl_action_client_t");

  bool is_available = false;
  rcl_ret_t ret = rcl_action_server_is_available(node, action_client, &is_available);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to check if action server is available");
  }
  return is_available;
}

#define SEND_SERVICE_REQUEST(Type) \
  auto action_client = get_pointer<rcl_action_client_t *>(pyaction_client, "rcl_action_client_t"); \
  destroy_ros_message_signature * destroy_ros_message = NULL; \
  void * raw_ros_request = rclpy_convert_from_py(pyrequest.ptr(), &destroy_ros_message); \
  if (!raw_ros_request) { \
    throw py::error_already_set(); \
  } \
  int64_t sequence_number; \
  rcl_ret_t ret = rcl_action_send_ ## Type ## _request( \
    action_client, raw_ros_request, &sequence_number); \
  destroy_ros_message(raw_ros_request); \
  if (ret != RCL_RET_OK) { \
    throw rclpy::RCLError("Failed to send " #Type " request"); \
  } \
  return sequence_number;
#define TAKE_SERVICE_RESPONSE(Type) \
  auto action_client = get_pointer<rcl_action_client_t *>(pyaction_client, "rcl_action_client_t"); \
  destroy_ros_message_signature * destroy_ros_message = NULL; \
  /* taken_msg is always destroyed in this function */ \
  void * taken_msg = rclpy_create_from_py(pymsg_type.ptr(), &destroy_ros_message); \
  if (!taken_msg) { \
    throw py::error_already_set(); \
  } \
  auto taken_msg_ptr = \
    std::unique_ptr<void, destroy_ros_message_signature *>(taken_msg, destroy_ros_message); \
  rmw_request_id_t header; \
  rcl_ret_t ret = rcl_action_take_ ## Type ## _response(action_client, &header, taken_msg); \
  int64_t sequence = header.sequence_number; \
  /* Create the tuple to return */ \
  py::tuple pytuple(2); \
  if (ret == RCL_RET_ACTION_CLIENT_TAKE_FAILED || ret == RCL_RET_ACTION_SERVER_TAKE_FAILED) { \
    pytuple[0] = py::none(); \
    pytuple[1] = py::none(); \
    return pytuple; \
  } else if (ret != RCL_RET_OK) { \
    throw rclpy::RCLError("Failed to take " #Type); \
  } \
  pytuple[0] = py::int_(sequence); \
  pytuple[1] = py::reinterpret_steal<py::object>( \
    rclpy_convert_to_py(taken_msg_ptr.release(), pymsg_type.ptr())); \
  return pytuple; \


/// Send an action goal request.
/**
 * Raises AttributeError if there is an issue parsing the pygoal_request.
 * Raises RuntimeError on failure.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pygoal_request The request message to send.
 * \return sequence_number PyLong object representing the index of the sent request, or
 * \return NULL if there is a failure.
 */
int64_t
rclpy_action_send_goal_request(py::capsule pyaction_client, py::object pyrequest)
{
  SEND_SERVICE_REQUEST(goal)
}

/// Take an action goal response.
/**
 * Raises AttributeError if there is an issue parsing the pygoal_response_type.
 * Raises RuntimeError if the underlying rcl library returns an error when taking the response.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pygoal_response_type An instance of the response message type to take.
 * \return 2-tuple (sequence number, received response), or
 * \return 2-tuple (None, None) if there is no response, or
 * \return NULL if there is a failure.
 */
py::tuple
rclpy_action_take_goal_response(py::capsule pyaction_client, py::object pymsg_type)
{
  TAKE_SERVICE_RESPONSE(goal)
}

/// Send an action result request.
/**
 * Raises AttributeError if there is an issue parsing the pyresult_request.
 * Raises RuntimeError if the underlying rcl library returns an error when sending the request.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pyresult_request The request message to send.
 * \return sequence_number PyLong object representing the index of the sent request, or
 * \return NULL if there is a failure.
 */
int64_t
rclpy_action_send_result_request(py::capsule pyaction_client, py::object pyrequest)
{
  SEND_SERVICE_REQUEST(result);
}

/// Take an action result response.
/**
 * Raises AttributeError if there is an issue parsing the pyresult_response_type.
 * Raises RuntimeError if the underlying rcl library returns an error when taking the response.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pyresult_response_type An instance of the response message type to take.
 * \return 2-tuple (sequence number, received response), or
 * \return 2-tuple (None, None) if there is no response, or
 * \return NULL if there is a failure.
 */
py::tuple
rclpy_action_take_result_response(py::capsule pyaction_client, py::object pymsg_type)
{
  TAKE_SERVICE_RESPONSE(result);
}

/// Send an action cancel request.
/**
 * Raises AttributeError if there is an issue parsing the pycancel_request.
 * Raises RuntimeError if the underlying rcl library returns an error when sending the request.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pycancel_request The request message to send.
 * \return sequence_number PyLong object representing the index of the sent request, or
 * \return NULL if there is a failure.
 */
int64_t
rclpy_action_send_cancel_request(py::capsule pyaction_client, py::object pyrequest)
{
  SEND_SERVICE_REQUEST(cancel)
}

/// Take an action cancel response.
/**
 * Raises AttributeError if there is an issue parsing the pycancel_response_type.
 * Raises RuntimeError if the underlying rcl library returns an error when taking the response.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pycancel_response_type An instance of the response message type to take.
 * \return 2-tuple (sequence number, received response), or
 * \return 2-tuple (None, None) if there is no response, or
 * \return NULL if there is a failure.
 */
py::tuple
rclpy_action_take_cancel_response(py::capsule pyaction_client, py::object pymsg_type)
{
  TAKE_SERVICE_RESPONSE(cancel)
}

#define TAKE_MESSAGE(Type) \
  auto action_client = get_pointer<rcl_action_client_t *>(pyaction_client, "rcl_action_client_t"); \
  destroy_ros_message_signature * destroy_ros_message = NULL; \
  void * taken_msg = rclpy_create_from_py(pymsg_type.ptr(), &destroy_ros_message); \
  if (!taken_msg) { \
    throw py::error_already_set(); \
  } \
  auto taken_msg_ptr = std::unique_ptr<void, decltype(destroy_ros_message)>( \
    taken_msg, destroy_ros_message); \
  rcl_ret_t ret = rcl_action_take_ ## Type(action_client, taken_msg); \
  if (ret != RCL_RET_OK) { \
    if (ret == RCL_RET_ACTION_CLIENT_TAKE_FAILED) { \
      /* if take failed, just do nothing */ \
      return py::none(); \
    } \
    throw rclpy::RCLError("Failed to take " #Type " with an action client"); \
  } \
  return py::reinterpret_steal<py::object>(rclpy_convert_to_py(taken_msg, pymsg_type.ptr()));

/// Take a feedback message from a given action client.
/**
 * Raises AttributeError if there is an issue parsing the pyfeedback_type.
 * Raises RuntimeError on failure while taking a feedback message. Note, this does not include
 * the case where there are no messages available.
 *
 * \param[in] pyaction_client Capsule pointing to the action client to process the message.
 * \param[in] pyfeedback_type Instance of the feedback message type to take.
 * \return Python message with all fields populated with received message, or
 * \return None if there is nothing to take, or
 * \return NULL if there is a failure.
 */
py::object
rclpy_action_take_feedback(py::capsule pyaction_client, py::object pymsg_type)
{
  TAKE_MESSAGE(feedback)
}

/// Take a status message from a given action client.
/**
 * Raises AttributeError if there is an issue parsing the pystatus_type.
 * Raises RuntimeError on failure while taking a status message. Note, this does not include
 * the case where there are no messages available.
 *
 * \param[in] pyaction_client Capsule pointing to the action client to process the message.
 * \param[in] pystatus_type Instance of the status message type to take.
 * \return Python message with all fields populated with received message, or
 * \return None if there is nothing to take, or
 * \return NULL if there is a failure.
 */
py::object
rclpy_action_take_status(py::capsule pyaction_client, py::object pymsg_type)
{
  TAKE_MESSAGE(status)
}

py::object
rclpy_action_get_client_names_and_types_by_node(
  py::capsule pynode, const char * remote_node_name, const char * remote_node_namespace)
{
  rcl_node_t * node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_client_names_and_types_by_node(
    node,
    &allocator,
    remote_node_name,
    remote_node_namespace,
    &names_and_types);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get action client names and type");
  }

  py::object pynames_and_types = py::reinterpret_steal<py::object>(
    rclpy_convert_to_py_names_and_types(&names_and_types));
  if (!rclpy_names_and_types_fini(&names_and_types)) {
    throw py::error_already_set();
  }
  return pynames_and_types;
}

py::object
rclpy_action_get_server_names_and_types_by_node(
  py::capsule pynode, const char * remote_node_name, const char * remote_node_namespace)
{
  rcl_node_t * node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_server_names_and_types_by_node(
    node,
    &allocator,
    remote_node_name,
    remote_node_namespace,
    &names_and_types);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get action server names and type");
  }

  py::object pynames_and_types = py::reinterpret_steal<py::object>(
    rclpy_convert_to_py_names_and_types(&names_and_types));
  if (!rclpy_names_and_types_fini(&names_and_types)) {
    throw py::error_already_set();
  }
  return pynames_and_types;
}

py::object
rclpy_action_get_names_and_types(py::capsule pynode)
{
  rcl_node_t * node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_names_and_types(node, &allocator, &names_and_types);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get action names and type");
  }

  py::object pynames_and_types = py::reinterpret_steal<py::object>(
    rclpy_convert_to_py_names_and_types(&names_and_types));
  if (!rclpy_names_and_types_fini(&names_and_types)) {
    throw py::error_already_set();
  }
  return pynames_and_types;
}


namespace rclpy
{
void
define_action_api(py::module m)
{
  m.def(
    "rclpy_action_destroy_entity", &rclpy_action_destroy_entity,
    "Destroy a rclpy_action entity.");
  m.def(
    "rclpy_action_get_rmw_qos_profile", &rclpy_action_get_rmw_qos_profile,
    "Get an action RMW QoS profile.");
  m.def(
    "rclpy_action_wait_set_add", &rclpy_action_wait_set_add,
    "Add an action entitiy to a wait set.");
  m.def(
    "rclpy_action_wait_set_get_num_entities", &rclpy_action_wait_set_get_num_entities,
    "Get the number of wait set entities for an action entitity.");
  m.def(
    "rclpy_action_wait_set_is_ready", &rclpy_action_wait_set_is_ready,
    "Check if an action entity has any sub-entities ready in a wait set.");
  m.def(
    "rclpy_action_create_client", &rclpy_action_create_client,
    "Create an action client.");
  m.def(
    "rclpy_action_server_is_available", &rclpy_action_server_is_available,
    "Check if an action server is available for a given client.");
  m.def(
    "rclpy_action_send_goal_request", &rclpy_action_send_goal_request,
    "Send a goal request.");
  m.def(
    "rclpy_action_take_goal_response", &rclpy_action_take_goal_response,
    "Take a goal response.");
  m.def(
    "rclpy_action_send_result_request", &rclpy_action_send_result_request,
    "Send a result request.");
  m.def(
    "rclpy_action_send_cancel_request", &rclpy_action_send_cancel_request,
    "Send a cancel request.");
  m.def(
    "rclpy_action_take_cancel_response", &rclpy_action_take_cancel_response,
    "Take a cancel response.");
  m.def(
    "rclpy_action_take_feedback", &rclpy_action_take_feedback,
    "Take a feedback message.");
  m.def(
    "rclpy_action_take_status", &rclpy_action_take_status,
    "Take a status message.");

  define_action_goal_handle(m);
  define_action_server(m);

  m.def(
    "rclpy_action_get_client_names_and_types_by_node",
    &rclpy_action_get_client_names_and_types_by_node,
    "Get action client names and types by node.");
  m.def(
    "rclpy_action_get_server_names_and_types_by_node",
    &rclpy_action_get_server_names_and_types_by_node,
    "Get action server names and types by node.");
  m.def(
    "rclpy_action_get_names_and_types", &rclpy_action_get_names_and_types,
    "Get action names and types.");
}
}  // namespace rclpy
