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

extern "C" {
#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"
}

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
  } else if (0 == std::strcmp("rcl_action_server_t", pyentity.name())) {
    auto action_server = get_pointer<rcl_action_server_t *>(pyentity, "rcl_action_server_t");
    ret = rcl_action_server_fini(action_server, node);
    PyMem_Free(action_server);
  } else {
    std::string entity_name = pyentity.name();
    throw std::runtime_error(entity_name + " is not a known entity");
  }

  if (ret != RCL_RET_OK) {
    std::string error_text = "Failed to fini '";
    error_text += pyentity.name();
    error_text += "': ";
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }

  if (PyCapsule_SetName(pyentity.ptr(), "_destroyed_by_rclpy_action_destroy_entity_")) {
    throw py::error_already_set();
  }
}

void
rclpy_action_destroy_server_goal_handle(py::capsule pygoal_handle)
{
  if (strcmp("rcl_action_goal_handle_t", pygoal_handle.name())) {
    throw py::value_error("Capsule must be an rcl_action_goal_handle_t");
  }

  auto goal_handle =
    get_pointer<rcl_action_goal_handle_t *>(pygoal_handle, "rcl_action_goal_handle_t");

  rcl_ret_t ret = rcl_action_goal_handle_fini(goal_handle);
  if (RCL_RET_OK != ret) {
    std::string error_text = "Error destroying action goal handle: ";
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
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
    rcutils_reset_error();
    throw std::runtime_error(error_text);
  }

  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to add '"};
    error_text += pyentity.name();
    error_text += "' to wait set: ";
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
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
  } else if (0 == strcmp(pyentity.name(), "rcl_action_server_t")) {
    auto action_server = get_pointer<rcl_action_server_t *>(pyentity, "rcl_action_server_t");

    ret = rcl_action_server_wait_set_get_num_entities(
      action_server,
      &num_subscriptions,
      &num_guard_conditions,
      &num_timers,
      &num_clients,
      &num_services);
  } else {
    std::string error_text{"Unknown entity: "};
    error_text += pyentity.name();
    rcutils_reset_error();
    throw std::runtime_error(error_text);
  }

  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to get number of entities for '"};
    error_text += pyentity.name();
    error_text += "': ";
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
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
      std::string error_text{"Failed to get number of ready entities for action client: "};
      error_text += rcl_get_error_string().str;
      rcl_reset_error();
      throw std::runtime_error(error_text);
    }

    py::tuple result_tuple(5);
    result_tuple[0] = py::bool_(is_feedback_ready);
    result_tuple[1] = py::bool_(is_status_ready);
    result_tuple[2] = py::bool_(is_goal_response_ready);
    result_tuple[3] = py::bool_(is_cancel_response_ready);
    result_tuple[4] = py::bool_(is_result_response_ready);
    return result_tuple;
  } else if (0 == strcmp(pyentity.name(), "rcl_action_server_t")) {
    auto action_server = get_pointer<rcl_action_server_t *>(pyentity, "rcl_action_server_t");
    bool is_goal_request_ready = false;
    bool is_cancel_request_ready = false;
    bool is_result_request_ready = false;
    bool is_goal_expired = false;
    rcl_ret_t ret = rcl_action_server_wait_set_get_entities_ready(
      wait_set,
      action_server,
      &is_goal_request_ready,
      &is_cancel_request_ready,
      &is_result_request_ready,
      &is_goal_expired);
    if (RCL_RET_OK != ret) {
      std::string error_text{"Failed to get number of ready entities for action server: "};
      error_text += rcl_get_error_string().str;
      rcl_reset_error();
      throw std::runtime_error(error_text);
    }

    py::tuple result_tuple(4);
    result_tuple[0] = py::bool_(is_goal_request_ready);
    result_tuple[1] = py::bool_(is_cancel_request_ready);
    result_tuple[2] = py::bool_(is_result_request_ready);
    result_tuple[3] = py::bool_(is_goal_expired);
    return result_tuple;
  }

  std::string error_text{"Unknown entity: "};
  error_text += pyentity.name();
  rcutils_reset_error();
  throw std::runtime_error(error_text);
}

void
copy_qos_profile(rmw_qos_profile_t & profile, py::capsule pyprofile)
{
  auto qos_profile = get_pointer<rmw_qos_profile_t *>(pyprofile, "rmw_qos_profile_t");
  profile = *qos_profile;
  PyMem_Free(qos_profile);
  if (PyCapsule_SetName(pyprofile.ptr(), "_destructed_by_copy_qos_profile_")) {
    throw py::error_already_set();
  }
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
 * \param[in] pygoal_service_qos Capsule pointing to a rmw_qos_profile_t object
 *   for the goal service.
 * \param[in] pyresult_service_qos Capsule pointing to a rmw_qos_profile_t object
 *   for the result service.
 * \param[in] pycancel_service_qos Capsule pointing to a rmw_qos_profile_t object
 *   for the cancel service.
 * \param[in] pyfeedback_qos Capsule pointing to a rmw_qos_profile_t object
 *   for the feedback subscriber.
 * \param[in] pystatus_qos Capsule pointing to a rmw_qos_profile_t object for the
 *   status subscriber.
 * \return Capsule named 'rcl_action_client_t', or
 * \return NULL on failure.
 */
py::capsule
rclpy_action_create_client(
  py::capsule pynode,
  py::object pyaction_type,
  const char * action_name,
  py::capsule pygoal_service_qos,
  py::capsule pyresult_service_qos,
  py::capsule pycancel_service_qos,
  py::capsule pyfeedback_topic_qos,
  py::capsule pystatus_topic_qos)
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

  copy_qos_profile(action_client_ops.goal_service_qos, pygoal_service_qos);
  copy_qos_profile(action_client_ops.result_service_qos, pyresult_service_qos);
  copy_qos_profile(action_client_ops.cancel_service_qos, pycancel_service_qos);
  copy_qos_profile(action_client_ops.feedback_topic_qos, pyfeedback_topic_qos);
  copy_qos_profile(action_client_ops.status_topic_qos, pystatus_topic_qos);

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
    rcutils_reset_error();
    throw py::value_error(error_text);
  } else if (ret != RCL_RET_OK) {
    std::string error_text{"Failed to create action client: "};
    error_text += rcl_get_error_string().str;
    rcutils_reset_error();
    throw py::value_error(error_text);
  }

  return py::capsule(action_client.release(), "rcl_action_client_t");
}

/// Create an action server.
/**
 * This function will create an action server for the given action name.
 * This server will use the typesupport defined in the action module
 * provided as pyaction_type to send messages over the wire.
 *
 * On a successful call a capsule referencing the created rcl_action_server_t structure
 * is returned.
 *
 * Raises AttributeError if action type is invalid
 * Raises ValueError if action name is invalid
 * Raises RuntimeError if the action server could not be created.
 *
 * \remark Call rclpy_action_destroy_entity() to destroy an action server.
 * \param[in] pynode Capsule pointing to the node to add the action server to.
 * \param[in] pyaction_type Action module associated with the action server.
 * \param[in] pyaction_name Python object containing the action name.
 * \param[in] pygoal_service_qos Capsule pointing to a rmw_qos_profile_t object
 *   for the goal service.
 * \param[in] pyresult_service_qos Capsule pointing to a rmw_qos_profile_t object
 *   for the result service.
 * \param[in] pycancel_service_qos Capsule pointing to a rmw_qos_profile_t object
 *   for the cancel service.
 * \param[in] pyfeedback_qos Capsule pointing to a rmw_qos_profile_t object
 *   for the feedback subscriber.
 * \param[in] pystatus_qos Capsule pointing to a rmw_qos_profile_t object for the
 *   status subscriber.
 * \return Capsule named 'rcl_action_server_t', or
 * \return NULL on failure.
 */
py::capsule
rclpy_action_create_server(
  py::capsule pynode,
  py::capsule pyclock,
  py::object pyaction_type,
  const char * action_name,
  py::capsule pygoal_service_qos,
  py::capsule pyresult_service_qos,
  py::capsule pycancel_service_qos,
  py::capsule pyfeedback_topic_qos,
  py::capsule pystatus_topic_qos,
  double result_timeout)
{
  rcl_node_t * node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_clock_t * clock = static_cast<rcl_clock_t *>(
    rclpy_handle_get_pointer_from_capsule(pyclock.ptr(), "rcl_clock_t"));
  if (!clock) {
    throw py::error_already_set();
  }

  rosidl_action_type_support_t * ts = static_cast<rosidl_action_type_support_t *>(
    rclpy_common_get_type_support(pyaction_type.ptr()));
  if (!ts) {
    throw py::error_already_set();
  }

  rcl_action_server_options_t action_server_ops = rcl_action_server_get_default_options();

  copy_qos_profile(action_server_ops.goal_service_qos, pygoal_service_qos);
  copy_qos_profile(action_server_ops.result_service_qos, pyresult_service_qos);
  copy_qos_profile(action_server_ops.cancel_service_qos, pycancel_service_qos);
  copy_qos_profile(action_server_ops.feedback_topic_qos, pyfeedback_topic_qos);
  copy_qos_profile(action_server_ops.status_topic_qos, pystatus_topic_qos);
  action_server_ops.result_timeout.nanoseconds = (rcl_duration_value_t)RCL_S_TO_NS(result_timeout);

  auto deleter = [](rcl_action_server_t * ptr) {PyMem_Free(ptr);};
  auto action_server = std::unique_ptr<rcl_action_server_t, decltype(deleter)>(
    static_cast<rcl_action_server_t *>(PyMem_Malloc(sizeof(rcl_action_server_t))),
    deleter);
  if (!action_server) {
    throw std::bad_alloc();
  }

  *action_server = rcl_action_get_zero_initialized_server();
  rcl_ret_t ret = rcl_action_server_init(
    action_server.get(),
    node,
    clock,
    ts,
    action_name,
    &action_server_ops);
  if (ret == RCL_RET_ACTION_NAME_INVALID) {
    std::string error_text{"Failed to create action server due to invalid topic name '"};
    error_text += action_name;
    error_text += "' : ";
    error_text += rcl_get_error_string().str;
    rcutils_reset_error();
    throw py::value_error(error_text);
  } else if (ret != RCL_RET_OK) {
    std::string error_text{"Failed to create action server: "};
    error_text += rcl_get_error_string().str;
    rcutils_reset_error();
    throw py::value_error(error_text);
  }

  return py::capsule(action_server.release(), "rcl_action_server_t");
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
    std::string error_text{"Failed to check if action server is available: "};
    error_text += rcl_get_error_string().str;
    rcutils_reset_error();
    throw std::runtime_error(error_text);
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
    std::string error_text{"Failed to send " #Type " request: "}; \
    error_text += rcl_get_error_string().str; \
    rcl_reset_error(); \
    throw std::runtime_error(error_text); \
  } \
  return sequence_number;

#define SEND_SERVICE_RESPONSE(Type) \
  auto action_server = get_pointer<rcl_action_server_t *>(pyaction_server, "rcl_action_server_t"); \
  auto header = get_pointer<rmw_request_id_t *>(pyheader, "rmw_request_id_t"); \
  destroy_ros_message_signature * destroy_ros_message = NULL; \
  void * raw_ros_response = rclpy_convert_from_py(pyresponse.ptr(), &destroy_ros_message); \
  if (!raw_ros_response) { \
    throw py::error_already_set(); \
  } \
  rcl_ret_t ret = rcl_action_send_ ## Type ## _response(action_server, header, raw_ros_response); \
  destroy_ros_message(raw_ros_response); \
  if (ret != RCL_RET_OK) { \
    std::string error_text{"Failed to send " #Type " response: "}; \
    error_text += rcl_get_error_string().str; \
    rcl_reset_error(); \
    throw std::runtime_error(error_text); \
  }

#define TAKE_SERVICE_REQUEST(Type) \
  auto action_server = get_pointer<rcl_action_server_t *>(pyaction_server, "rcl_action_server_t"); \
  destroy_ros_message_signature * destroy_ros_message = NULL; \
  /* taken_msg is always destroyed in this function */ \
  void * taken_msg = rclpy_create_from_py(pymsg_type.ptr(), &destroy_ros_message); \
  if (!taken_msg) { \
    throw py::error_already_set(); \
  } \
  auto taken_msg_ptr = \
    std::unique_ptr<void, destroy_ros_message_signature *>(taken_msg, destroy_ros_message); \
  auto header_deleter = [](rmw_request_id_t * ptr) {PyMem_Free(static_cast<void *>(ptr));}; \
  /* header only destroyed on error in this function */ \
  auto header = std::unique_ptr<rmw_request_id_t, decltype(header_deleter)>( \
    static_cast<rmw_request_id_t *>(PyMem_Malloc(sizeof(rmw_request_id_t))), header_deleter); \
  rcl_ret_t ret = \
    rcl_action_take_ ## Type ## _request(action_server, header.get(), taken_msg_ptr.get()); \
  /* Create the tuple to return */ \
  py::tuple pytuple(2); \
  if (ret == RCL_RET_ACTION_CLIENT_TAKE_FAILED || ret == RCL_RET_ACTION_SERVER_TAKE_FAILED) { \
    pytuple[0] = py::none(); \
    pytuple[1] = py::none(); \
    return pytuple; \
  } else if (ret != RCL_RET_OK) { \
    std::string error_text{"Failed to take " #Type ": "}; \
    error_text += rcl_get_error_string().str; \
    rcl_reset_error(); \
    throw std::runtime_error(error_text); \
  } \
  /* TODO(sloretz) This looks suspicious, what is currently deleting header? */ \
  pytuple[0] = py::capsule(header.release(), "rmw_request_id_t"); \
  pytuple[1] = py::reinterpret_steal<py::object>( \
    rclpy_convert_to_py(taken_msg_ptr.get(), pymsg_type.ptr())); \
  return pytuple;

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
  auto header_deleter = [](rmw_request_id_t * ptr) {PyMem_Free(static_cast<void *>(ptr));}; \
  auto header = std::unique_ptr<rmw_request_id_t, decltype(header_deleter)>( \
    static_cast<rmw_request_id_t *>(PyMem_Malloc(sizeof(rmw_request_id_t))), header_deleter); \
  rcl_ret_t ret = rcl_action_take_ ## Type ## _response(action_client, header.get(), taken_msg); \
  int64_t sequence = header->sequence_number; \
  /* Create the tuple to return */ \
  py::tuple pytuple(2); \
  if (ret == RCL_RET_ACTION_CLIENT_TAKE_FAILED || ret == RCL_RET_ACTION_SERVER_TAKE_FAILED) { \
    pytuple[0] = py::none(); \
    pytuple[1] = py::none(); \
    return pytuple; \
  } else if (ret != RCL_RET_OK) { \
    std::string error_text{"Failed to take " #Type ": "}; \
    error_text += rcl_get_error_string().str; \
    rcl_reset_error(); \
    throw std::runtime_error(error_text); \
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

/// Take an action goal request.
/**
 * Raises AttributeError if there is an issue parsing the pygoal_request_type.
 * Raises RuntimeError on failure.
 *
 * \param[in] pyaction_server The action server to use when taking the request.
 * \param[in] pygoal_request_type An instance of the type of request message to take.
 * \return 2-tuple (header, received request message) where the header is a Capsule of
 *   type "rmw_request_id_t", or
 * \return 2-tuple (None, None) if there as no message to take, or
 * \return NULL if there is a failure.
 */
py::tuple
rclpy_action_take_goal_request(py::capsule pyaction_server, py::object pymsg_type)
{
  TAKE_SERVICE_REQUEST(goal)
}

/// Send an action goal response.
/**
 * Raises AttributeError if there is an issue parsing the pygoal_response.
 * Raises RuntimeError on failure.
 *
 * \param[in] pyaction_server The action server to use when sending the response.
 * \param[in] pyheader Capsule pointer to the message header of type "rmw_request_id_t".
 * \param[in] pygoal_response The response message to send.
 * \return None
 * \return NULL if there is a failure.
 */
void
rclpy_action_send_goal_response(
  py::capsule pyaction_server, py::capsule pyheader, py::object pyresponse)
{
  SEND_SERVICE_RESPONSE(goal)
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

/// Take an action result request.
/**
 * Raises AttributeError if there is an issue parsing the pyresult_request_type.
 * Raises RuntimeError on failure.
 *
 * \param[in] pyaction_server The action server to use when taking the request.
 * \param[in] pyresult_request_type An instance of the type of request message to take.
 * \return 2-tuple (header, received request message) where the header is a Capsule of
 *   type "rmw_request_id_t", or
 * \return 2-tuple (None, None) if there as no message to take, or
 * \return NULL if there is a failure.
 */
py::tuple
rclpy_action_take_result_request(py::capsule pyaction_server, py::object pymsg_type)
{
  TAKE_SERVICE_REQUEST(result)
}

/// Send an action result response.
/**
 * Raises AttributeError if there is an issue parsing the pyresult_response.
 * Raises RuntimeError on failure.
 *
 * \param[in] pyaction_server The action server to use when sending the response.
 * \param[in] pyheader Capsule pointer to the message header of type "rmw_request_id_t".
 * \param[in] pyresult_response The response message to send.
 * \return None
 * \return NULL if there is a failure.
 */
void
rclpy_action_send_result_response(
  py::capsule pyaction_server, py::capsule pyheader, py::object pyresponse)
{
  SEND_SERVICE_RESPONSE(result)
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

/// Take an action cancel request.
/**
 * Raises AttributeError if there is an issue parsing the pycancel_request_type.
 * Raises RuntimeError on failure.
 *
 * \param[in] pyaction_server The action server to use when taking the request.
 * \param[in] pycancel_request_type An instance of the type of request message to take.
 * \return 2-tuple (header, received request message) where the header is a Capsule of
 *   type "rmw_request_id_t", or
 * \return 2-tuple (None, None) if there as no message to take, or
 * \return NULL if there is a failure.
 */
py::tuple
rclpy_action_take_cancel_request(py::capsule pyaction_server, py::object pymsg_type)
{
  TAKE_SERVICE_REQUEST(cancel)
}

/// Send an action cancel response.
/**
 * Raises AttributeError if there is an issue parsing the pycancel_response.
 * Raises RuntimeError on failure.
 *
 * \param[in] pyaction_server The action server to use when sending the response.
 * \param[in] pyheader Capsule pointer to the message header of type "rmw_request_id_t".
 * \param[in] pycancel_response The response message to send.
 * \return sequence_number PyLong object representing the index of the sent response, or
 * \return NULL if there is a failure.
 */
void
rclpy_action_send_cancel_response(
  py::capsule pyaction_server, py::capsule pyheader, py::object pyresponse)
{
  SEND_SERVICE_RESPONSE(cancel)
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
    std::string error_text{"Failed to take " #Type " with an action client: "}; \
    error_text += rcl_get_error_string().str; \
    rcl_reset_error(); \
    throw std::runtime_error(error_text); \
  } \
  return py::reinterpret_steal<py::object>(rclpy_convert_to_py(taken_msg, pymsg_type.ptr()));

/// Publish a feedback message from a given action server.
/**
 * Raises AttributeError if there is an issue parsing the pyfeedback_msg.
 * Raises RuntimeError on failure while publishing a feedback message.
 *
 * \param[in] pyaction_server Capsule pointing to the action server to publish the message.
 * \param[in] pyfeedback_msg The feedback message to publish.
 * \return None
 */
void
rclpy_action_publish_feedback(py::capsule pyaction_server, py::object pymsg)
{
  auto action_server = get_pointer<rcl_action_server_t *>(pyaction_server, "rcl_action_server_t");
  destroy_ros_message_signature * destroy_ros_message = NULL;
  void * raw_ros_message = rclpy_convert_from_py(pymsg.ptr(), &destroy_ros_message);
  if (!raw_ros_message) {
    throw py::error_already_set();
  }
  auto raw_ros_message_ptr = std::unique_ptr<void, decltype(destroy_ros_message)>(
    raw_ros_message, destroy_ros_message);
  rcl_ret_t ret = rcl_action_publish_feedback(action_server, raw_ros_message);
  if (ret != RCL_RET_OK) {
    std::string error_text{"Failed to publish feedback with an action server: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }
}

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

/// Publish a status message from a given action server.
/**
 * Raises RuntimeError on failure while publishing a status message.
 *
 * \param[in] pyaction_server Capsule pointing to the action server to publish the message.
 * \return None
 */
void
rclpy_action_publish_status(py::capsule pyaction_server)
{
  auto action_server = get_pointer<rcl_action_server_t *>(pyaction_server, "rcl_action_server_t");
  rcl_action_goal_status_array_t status_message =
    rcl_action_get_zero_initialized_goal_status_array();
  rcl_ret_t ret = rcl_action_get_goal_status_array(action_server, &status_message);
  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed get goal status array: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }

  ret = rcl_action_publish_status(action_server, &status_message);

  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed publish goal status array: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }
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

py::capsule
rclpy_action_accept_new_goal(py::capsule pyaction_server, py::object pygoal_info_msg)
{
  auto action_server = get_pointer<rcl_action_server_t *>(pyaction_server, "rcl_action_server_t");
  destroy_ros_message_signature * destroy_ros_message = NULL;
  auto goal_info_msg = static_cast<rcl_action_goal_info_t *>(
    rclpy_convert_from_py(pygoal_info_msg.ptr(), &destroy_ros_message));
  if (!goal_info_msg) {
    throw py::error_already_set();
  }

  auto goal_info_msg_ptr = std::unique_ptr<rcl_action_goal_info_t, decltype(destroy_ros_message)>(
    goal_info_msg, destroy_ros_message);

  rcl_action_goal_handle_t * goal_handle = rcl_action_accept_new_goal(
    action_server, goal_info_msg);
  if (!goal_handle) {
    std::string error_text{"Failed to accept new goal: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }
  // TODO(sloretz) capsule destructor instead of rclpy_action_destroy_server_goal_handle()
  return py::capsule(goal_handle, "rcl_action_goal_handle_t");
}

void
rclpy_action_notify_goal_done(py::capsule pyaction_server)
{
  auto action_server = get_pointer<rcl_action_server_t *>(pyaction_server, "rcl_action_server_t");
  rcl_ret_t ret = rcl_action_notify_goal_done(action_server);
  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to notfiy action server of goal done: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }
}

/// Convert from a Python GoalEvent code to an rcl goal event code.
/**
 *  Raises std::runtime_error if conversion fails
 *
 *  \param[in] pyevent The Python GoalEvent code.
 *  \return The rcl equivalent of the Python GoalEvent code
 */
static rcl_action_goal_event_t
convert_from_py_goal_event(const int64_t event)
{
  py::module server_module = py::module::import("rclpy.action.server");
  py::object goal_event_class = server_module.attr("GoalEvent");
  py::int_ pyevent(event);

  if (goal_event_class.attr("EXECUTE").attr("value").cast<py::int_>().is(pyevent)) {
    return GOAL_EVENT_EXECUTE;
  }
  if (goal_event_class.attr("CANCEL_GOAL").attr("value").cast<py::int_>().is(pyevent)) {
    return GOAL_EVENT_CANCEL_GOAL;
  }
  if (goal_event_class.attr("SUCCEED").attr("value").cast<py::int_>().is(pyevent)) {
    return GOAL_EVENT_SUCCEED;
  }
  if (goal_event_class.attr("ABORT").attr("value").cast<py::int_>().is(pyevent)) {
    return GOAL_EVENT_ABORT;
  }
  if (goal_event_class.attr("CANCELED").attr("value").cast<py::int_>().is(pyevent)) {
    return GOAL_EVENT_CANCELED;
  }
  throw std::runtime_error("Error converting goal event type: unknown goal event");
}

void
rclpy_action_update_goal_state(py::capsule pygoal_handle, int64_t pyevent)
{
  auto goal_handle = get_pointer<rcl_action_goal_handle_t *>(
    pygoal_handle, "rcl_action_goal_handle_t");

  rcl_action_goal_event_t event = convert_from_py_goal_event(pyevent);

  rcl_ret_t ret = rcl_action_update_goal_state(goal_handle, event);
  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to update goal state: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }
}

bool
rclpy_action_goal_handle_is_active(py::capsule pygoal_handle)
{
  auto goal_handle = get_pointer<rcl_action_goal_handle_t *>(
    pygoal_handle, "rcl_action_goal_handle_t");
  return rcl_action_goal_handle_is_active(goal_handle);
}

bool
rclpy_action_server_goal_exists(py::capsule pyaction_server, py::object pygoal_info)
{
  auto action_server = get_pointer<rcl_action_server_t *>(pyaction_server, "rcl_action_server_t");
  destroy_ros_message_signature * destroy_ros_message = NULL;
  rcl_action_goal_info_t * goal_info = static_cast<rcl_action_goal_info_t *>(
    rclpy_convert_from_py(pygoal_info.ptr(), &destroy_ros_message));
  if (!goal_info) {
    throw py::error_already_set();
  }

  auto goal_info_ptr = std::unique_ptr<rcl_action_goal_info_t, decltype(destroy_ros_message)>(
    goal_info, destroy_ros_message);

  return rcl_action_server_goal_exists(action_server, goal_info);
}

rcl_action_goal_state_t
rclpy_action_goal_handle_get_status(py::capsule pygoal_handle)
{
  auto goal_handle = get_pointer<rcl_action_goal_handle_t *>(
    pygoal_handle, "rcl_action_goal_handle_t");

  rcl_action_goal_state_t status;
  rcl_ret_t ret = rcl_action_goal_handle_get_status(goal_handle, &status);
  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to get goal status: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }

  return status;
}

py::object
rclpy_action_process_cancel_request(
  py::capsule pyaction_server, py::object pycancel_request, py::object pycancel_response_type)
{
  auto action_server = get_pointer<rcl_action_server_t *>(pyaction_server, "rcl_action_server_t");

  destroy_ros_message_signature * destroy_cancel_request = NULL;
  rcl_action_cancel_request_t * cancel_request = static_cast<rcl_action_cancel_request_t *>(
    rclpy_convert_from_py(pycancel_request.ptr(), &destroy_cancel_request));
  if (!cancel_request) {
    throw py::error_already_set();
  }
  auto cancel_request_ptr =
    std::unique_ptr<rcl_action_cancel_request_t, decltype(destroy_cancel_request)>(
    cancel_request, destroy_cancel_request);

  rcl_action_cancel_response_t cancel_response = rcl_action_get_zero_initialized_cancel_response();
  rcl_ret_t ret = rcl_action_process_cancel_request(
    action_server, cancel_request, &cancel_response);

  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to process cancel request: "};
    error_text += rcl_get_error_string().str;

    ret = rcl_action_cancel_response_fini(&cancel_response);
    if (RCL_RET_OK != ret) {
      error_text += ".  Also failed to cleanup response: ";
      error_text += rcl_get_error_string().str;
    }
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }

  PyObject * pycancel_response =
    rclpy_convert_to_py(&cancel_response.msg, pycancel_response_type.ptr());
  if (!pycancel_response) {
    rcl_ret_t ignore = rcl_action_cancel_response_fini(&cancel_response);
    (void) ignore;
    throw py::error_already_set();
  }
  py::object return_value = py::reinterpret_steal<py::object>(pycancel_response);

  ret = rcl_action_cancel_response_fini(&cancel_response);

  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to finalize cancel response: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }
  return return_value;
}

py::tuple
rclpy_action_expire_goals(py::capsule pyaction_server, int64_t max_num_goals)
{
  auto action_server = get_pointer<rcl_action_server_t *>(pyaction_server, "rcl_action_server_t");

  auto expired_goals =
    std::unique_ptr<rcl_action_goal_info_t>(new rcl_action_goal_info_t[max_num_goals]);
  size_t num_expired;
  rcl_ret_t ret = rcl_action_expire_goals(
    action_server, expired_goals.get(), max_num_goals, &num_expired);
  if (RCL_RET_OK != ret) {
    std::string error_text{"Failed to expire goals: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }

  // Get Python GoalInfo type
  py::module pyaction_msgs_module = py::module::import("action_msgs.msg");
  py::object pygoal_info_class = pyaction_msgs_module.attr("GoalInfo");
  py::object pygoal_info_type = pygoal_info_class();

  // Create a tuple of GoalInfo instances to return
  py::tuple result_tuple(num_expired);

  for (size_t i = 0; i < num_expired; ++i) {
    result_tuple[i] = py::reinterpret_steal<py::object>(
      rclpy_convert_to_py(&(expired_goals.get()[i]), pygoal_info_type.ptr()));
  }

  return result_tuple;
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
    std::string error_text{"Failed to get action client names and type: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
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
    std::string error_text{"Failed to get action server names and type: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
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
    std::string error_text{"Failed to get action names and type: "};
    error_text += rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_text);
  }

  py::object pynames_and_types = py::reinterpret_steal<py::object>(
    rclpy_convert_to_py_names_and_types(&names_and_types));
  if (!rclpy_names_and_types_fini(&names_and_types)) {
    throw py::error_already_set();
  }
  return pynames_and_types;
}


PYBIND11_MODULE(_rclpy_action, m) {
  m.doc() = "ROS 2 Python Action library.";

  m.def(
    "rclpy_action_destroy_entity", &rclpy_action_destroy_entity,
    "Destroy a rclpy_action entity.");
  m.def(
    "rclpy_action_destroy_server_goal_handle", &rclpy_action_destroy_server_goal_handle,
    "Destroy a ServerGoalHandle.");
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
    "rclpy_action_create_server", &rclpy_action_create_server,
    "Create an action server.");
  m.def(
    "rclpy_action_server_is_available", &rclpy_action_server_is_available,
    "Check if an action server is available for a given client.");
  m.def(
    "rclpy_action_send_goal_request", &rclpy_action_send_goal_request,
    "Send a goal request.");
  m.def(
    "rclpy_action_take_goal_request", &rclpy_action_take_goal_request,
    "Take a goal request.");
  m.def(
    "rclpy_action_send_goal_response", &rclpy_action_send_goal_response,
    "Send a goal response.");
  m.def(
    "rclpy_action_take_goal_response", &rclpy_action_take_goal_response,
    "Take a goal response.");
  m.def(
    "rclpy_action_send_result_request", &rclpy_action_send_result_request,
    "Send a result request.");
  m.def(
    "rclpy_action_take_result_request", &rclpy_action_take_result_request,
    "Take a result request.");
  m.def(
    "rclpy_action_send_result_response", &rclpy_action_send_result_response,
    "Send a result response.");
  m.def(
    "rclpy_action_take_result_response", &rclpy_action_take_result_response,
    "Take a result response.");
  m.def(
    "rclpy_action_send_cancel_request", &rclpy_action_send_cancel_request,
    "Send a cancel request.");
  m.def(
    "rclpy_action_take_cancel_request", &rclpy_action_take_cancel_request,
    "Take a cancel request.");
  m.def(
    "rclpy_action_send_cancel_response", &rclpy_action_send_cancel_response,
    "Send a cancel response.");
  m.def(
    "rclpy_action_take_cancel_response", &rclpy_action_take_cancel_response,
    "Take a cancel response.");
  m.def(
    "rclpy_action_publish_feedback", &rclpy_action_publish_feedback,
    "Publish a feedback message.");
  m.def(
    "rclpy_action_take_feedback", &rclpy_action_take_feedback,
    "Take a feedback message.");
  m.def(
    "rclpy_action_publish_status", &rclpy_action_publish_status,
    "Publish a status message.");
  m.def(
    "rclpy_action_take_status", &rclpy_action_take_status,
    "Take a status message.");
  m.def(
    "rclpy_action_accept_new_goal", &rclpy_action_accept_new_goal,
    "Accept a new goal using an action server.");
  m.def(
    "rclpy_action_notify_goal_done", &rclpy_action_notify_goal_done,
    "Notify and action server that a goal has reached a terminal state.");
  m.def(
    "rclpy_action_update_goal_state", &rclpy_action_update_goal_state,
    "Update a goal state.");
  m.def(
    "rclpy_action_goal_handle_is_active", &rclpy_action_goal_handle_is_active,
    "Check if a goal is active.");
  m.def(
    "rclpy_action_server_goal_exists", &rclpy_action_server_goal_exists,
    "Check if a goal being tracked by an action server.");
  m.def(
    "rclpy_action_goal_handle_get_status", &rclpy_action_goal_handle_get_status,
    "Get the status of a goal.");
  m.def(
    "rclpy_action_process_cancel_request", &rclpy_action_process_cancel_request,
    "Process a cancel request to determine what goals should be canceled.");
  m.def(
    "rclpy_action_expire_goals", &rclpy_action_expire_goals,
    "Expire goals associated with an action server.");
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
