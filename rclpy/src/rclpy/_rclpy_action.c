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

#include <Python.h>

#include <rcl/error_handling.h>
#include <rcl_action/rcl_action.h>

#include "rclpy_common/common.h"

/// Destroy an rcl_action entity.
/**
 * Raises RuntimeError on failure.
 *
 * \param[in] pyentity Capsule pointing to the entity to destroy.
 * \param[in] pynode Capsule pointing to the node the action client was added to.
 */
static PyObject *
rclpy_action_destroy_entity(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyentity;
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "OO", &pyentity, &pynode)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  rcl_ret_t ret;
  if (PyCapsule_IsValid(pyentity, "rcl_action_client_t")) {
    rcl_action_client_t * action_client =
      (rcl_action_client_t *)PyCapsule_GetPointer(pyentity, "rcl_action_client_t");
    ret = rcl_action_client_fini(action_client, node);
    PyMem_Free(action_client);
  } else if (PyCapsule_IsValid(pyentity, "rcl_action_server_t")) {
    rcl_action_server_t * action_server =
      (rcl_action_server_t *)PyCapsule_GetPointer(pyentity, "rcl_action_server_t");
    ret = rcl_action_server_fini(action_server, node);
    PyMem_Free(action_server);
  } else {
    ret = RCL_RET_ERROR;  // to avoid a linter warning
    const char * entity_name = PyCapsule_GetName(pyentity);
    if (!entity_name) {
      return NULL;
    }
    return PyErr_Format(PyExc_RuntimeError, "'%s' is not a known entity", entity_name);
  }

  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to fini '%s': %s", PyCapsule_GetName(pyentity), rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  if (PyCapsule_SetPointer(pyentity, Py_None)) {
    // exception set by PyCapsule_SetPointer
    return NULL;
  }

  Py_RETURN_NONE;
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
static PyObject *
rclpy_action_get_rmw_qos_profile(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * rmw_profile;
  if (!PyArg_ParseTuple(args, "s", &rmw_profile)) {
    return NULL;
  }

  PyObject * pyqos_profile = NULL;
  if (0 == strcmp(rmw_profile, "rcl_action_qos_profile_status_default")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rcl_action_qos_profile_status_default);
  } else {
    return PyErr_Format(PyExc_RuntimeError,
             "Requested unknown rmw_qos_profile: '%s'", rmw_profile);
  }
  return pyqos_profile;
}

/// Add an action entitiy to a wait set.
/**
 * Raises RuntimeError on failure.
 * \param[in] pyentity Capsule pointer to an action entity
 *   (rcl_action_client_t or rcl_action_server_t).
 * \param[in] pywait_set Capsule pointer to an rcl_wait_set_t.
 */
static PyObject *
rclpy_action_wait_set_add(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyentity;
  PyObject * pywait_set;

  if (!PyArg_ParseTuple(args, "OO", &pyentity, &pywait_set)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, "rcl_wait_set_t");
  if (!wait_set) {
    return NULL;
  }

  rcl_ret_t ret;
  if (PyCapsule_IsValid(pyentity, "rcl_action_client_t")) {
    rcl_action_client_t * action_client =
      (rcl_action_client_t *)PyCapsule_GetPointer(pyentity, "rcl_action_client_t");

    ret = rcl_action_wait_set_add_action_client(wait_set, action_client, NULL, NULL);
  } else if (PyCapsule_IsValid(pyentity, "rcl_action_server_t")) {
    rcl_action_server_t * action_server =
      (rcl_action_server_t *)PyCapsule_GetPointer(pyentity, "rcl_action_server_t");
    ret = rcl_action_wait_set_add_action_server(wait_set, action_server, NULL);
  } else {
    ret = RCL_RET_ERROR;  // to avoid linter warning
    const char * entity_name = PyCapsule_GetName(pyentity);
    if (!entity_name) {
      return NULL;
    }
    return PyErr_Format(PyExc_RuntimeError, "'%s' is not a known entity", entity_name);
  }

  if (RCL_RET_OK != ret) {
    PyErr_Format(PyExc_RuntimeError, "Failed to add '%s' to wait set: %s",
      PyCapsule_GetName(pyentity), rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  Py_RETURN_NONE;
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
static PyObject *
rclpy_action_wait_set_get_num_entities(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyentity;

  if (!PyArg_ParseTuple(args, "O", &pyentity)) {
    return NULL;
  }

  size_t num_subscriptions = 0u;
  size_t num_guard_conditions = 0u;
  size_t num_timers = 0u;
  size_t num_clients = 0u;
  size_t num_services = 0u;

  rcl_ret_t ret;
  if (PyCapsule_IsValid(pyentity, "rcl_action_client_t")) {
    rcl_action_client_t * action_client =
      (rcl_action_client_t *)PyCapsule_GetPointer(pyentity, "rcl_action_client_t");

    ret = rcl_action_client_wait_set_get_num_entities(
      action_client,
      &num_subscriptions,
      &num_guard_conditions,
      &num_timers,
      &num_clients,
      &num_services);
  } else if (PyCapsule_IsValid(pyentity, "rcl_action_server_t")) {
    rcl_action_server_t * action_server =
      (rcl_action_server_t *)PyCapsule_GetPointer(pyentity, "rcl_action_server_t");

    ret = rcl_action_server_wait_set_get_num_entities(
      action_server,
      &num_subscriptions,
      &num_guard_conditions,
      &num_timers,
      &num_clients,
      &num_services);
  } else {
    ret = RCL_RET_ERROR;  // to avoid linter warning
    const char * entity_name = PyCapsule_GetName(pyentity);
    if (!entity_name) {
      return NULL;
    }
    return PyErr_Format(PyExc_RuntimeError, "'%s' is not a known entity", entity_name);
  }

  if (RCL_RET_OK != ret) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get number of entities for '%s': %s",
      PyCapsule_GetName(pyentity),
      rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyObject * result_tuple = PyTuple_New(5);
  if (!result_tuple) {
    return NULL;
  }

  // PyTuple_SetItem() returns 0 on success
  int set_result = 0;
  set_result += PyTuple_SetItem(result_tuple, 0, PyLong_FromSize_t(num_subscriptions));
  set_result += PyTuple_SetItem(result_tuple, 1, PyLong_FromSize_t(num_guard_conditions));
  set_result += PyTuple_SetItem(result_tuple, 2, PyLong_FromSize_t(num_timers));
  set_result += PyTuple_SetItem(result_tuple, 3, PyLong_FromSize_t(num_clients));
  set_result += PyTuple_SetItem(result_tuple, 4, PyLong_FromSize_t(num_services));

  if (0 != set_result) {
    Py_DECREF(result_tuple);
    return NULL;
  }
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
static PyObject *
rclpy_action_wait_set_is_ready(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyentity;
  PyObject * pywait_set;

  if (!PyArg_ParseTuple(args, "OO", &pyentity, &pywait_set)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, "rcl_wait_set_t");
  if (!wait_set) {
    return NULL;
  }

  if (PyCapsule_IsValid(pyentity, "rcl_action_client_t")) {
    rcl_action_client_t * action_client =
      (rcl_action_client_t *)PyCapsule_GetPointer(pyentity, "rcl_action_client_t");
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
      PyErr_Format(PyExc_RuntimeError,
        "Failed to get number of ready entities for action client: %s",
        rcl_get_error_string().str);
      rcl_reset_error();
      return NULL;
    }

    PyObject * result_tuple = PyTuple_New(5);
    if (!result_tuple) {
      return NULL;
    }

    // PyTuple_SetItem() returns 0 on success
    int set_result = 0;
    set_result += PyTuple_SetItem(result_tuple, 0, PyBool_FromLong(is_feedback_ready));
    set_result += PyTuple_SetItem(result_tuple, 1, PyBool_FromLong(is_status_ready));
    set_result += PyTuple_SetItem(result_tuple, 2, PyBool_FromLong(is_goal_response_ready));
    set_result += PyTuple_SetItem(result_tuple, 3, PyBool_FromLong(is_cancel_response_ready));
    set_result += PyTuple_SetItem(result_tuple, 4, PyBool_FromLong(is_result_response_ready));
    if (0 != set_result) {
      Py_DECREF(result_tuple);
      return NULL;
    }
    return result_tuple;
  } else if (PyCapsule_IsValid(pyentity, "rcl_action_server_t")) {
    rcl_action_server_t * action_server =
      (rcl_action_server_t *)PyCapsule_GetPointer(pyentity, "rcl_action_server_t");
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
      PyErr_Format(PyExc_RuntimeError,
        "Failed to get number of ready entities for action server: %s",
        rcl_get_error_string().str);
      rcl_reset_error();
      return NULL;
    }

    PyObject * result_tuple = PyTuple_New(4);
    if (!result_tuple) {
      return NULL;
    }

    // PyTuple_SetItem() returns 0 on success
    int set_result = 0;
    set_result += PyTuple_SetItem(result_tuple, 0, PyBool_FromLong(is_goal_request_ready));
    set_result += PyTuple_SetItem(result_tuple, 1, PyBool_FromLong(is_cancel_request_ready));
    set_result += PyTuple_SetItem(result_tuple, 2, PyBool_FromLong(is_result_request_ready));
    set_result += PyTuple_SetItem(result_tuple, 3, PyBool_FromLong(is_goal_expired));
    if (0 != set_result) {
      Py_DECREF(result_tuple);
      return NULL;
    }
    return result_tuple;
  } else {
    const char * entity_name = PyCapsule_GetName(pyentity);
    if (!entity_name) {
      return NULL;
    }
    return PyErr_Format(PyExc_RuntimeError, "'%s' is not a known entity", entity_name);
  }
}

#define OPTIONS_COPY_QOS_PROFILE(Options, Profile) \
  { \
    void * p = PyCapsule_GetPointer(py ## Profile, "rmw_qos_profile_t"); \
    if (!p) { \
      return NULL; \
    } \
    rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)p; \
    Options.Profile = * qos_profile; \
    PyMem_Free(p); \
    if (PyCapsule_SetPointer(py ## Profile, Py_None)) { \
      /* exception set by PyCapsule_SetPointer */ \
      return NULL; \
    } \
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
static PyObject *
rclpy_action_create_client(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pyaction_type;
  PyObject * pyaction_name;
  PyObject * pygoal_service_qos;
  PyObject * pyresult_service_qos;
  PyObject * pycancel_service_qos;
  PyObject * pyfeedback_topic_qos;
  PyObject * pystatus_topic_qos;

  int parse_tuple_result = PyArg_ParseTuple(
    args,
    "OOOOOOOO",
    &pynode,
    &pyaction_type,
    &pyaction_name,
    &pygoal_service_qos,
    &pyresult_service_qos,
    &pycancel_service_qos,
    &pyfeedback_topic_qos,
    &pystatus_topic_qos);

  if (!parse_tuple_result) {
    return NULL;
  }

  const char * action_name = PyUnicode_AsUTF8(pyaction_name);
  if (!action_name) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  PyObject * pymetaclass = PyObject_GetAttrString(pyaction_type, "__class__");
  if (!pymetaclass) {
    return NULL;
  }

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");
  Py_DECREF(pymetaclass);
  if (!pyts) {
    return NULL;
  }

  rosidl_action_type_support_t * ts =
    (rosidl_action_type_support_t *)PyCapsule_GetPointer(pyts, NULL);
  Py_DECREF(pyts);
  if (!ts) {
    return NULL;
  }

  rcl_action_client_options_t action_client_ops = rcl_action_client_get_default_options();

  OPTIONS_COPY_QOS_PROFILE(action_client_ops, goal_service_qos);
  OPTIONS_COPY_QOS_PROFILE(action_client_ops, result_service_qos);
  OPTIONS_COPY_QOS_PROFILE(action_client_ops, cancel_service_qos);
  OPTIONS_COPY_QOS_PROFILE(action_client_ops, feedback_topic_qos);
  OPTIONS_COPY_QOS_PROFILE(action_client_ops, status_topic_qos);

  rcl_action_client_t * action_client =
    (rcl_action_client_t *)PyMem_Malloc(sizeof(rcl_action_client_t));
  if (!action_client) {
    return PyErr_NoMemory();
  }
  *action_client = rcl_action_get_zero_initialized_client();
  rcl_ret_t ret = rcl_action_client_init(
    action_client,
    node,
    ts,
    action_name,
    &action_client_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_ACTION_NAME_INVALID) {
      PyErr_Format(PyExc_ValueError,
        "Failed to create action client due to invalid topic name '%s': %s",
        action_name, rcl_get_error_string().str);
    } else {
      PyErr_Format(PyExc_RuntimeError,
        "Failed to create action client: %s", rcl_get_error_string().str);
    }
    PyMem_Free(action_client);
    rcl_reset_error();
    return NULL;
  }

  return PyCapsule_New(action_client, "rcl_action_client_t", NULL);
}

/// Check if an action server is available for the given action client.
/**
 * Raises RuntimeError on failure.
 *
 * \param[in] pynode Capsule pointing to the node to associated with the action client.
 * \param[in] pyaction_client The action client to use when checking for an available server.
 * \return True if an action server is available, False otherwise.
 */
static PyObject *
rclpy_action_server_is_available(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pyaction_client;

  if (!PyArg_ParseTuple(args, "OO", &pynode, &pyaction_client)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }
  rcl_action_client_t * action_client = (rcl_action_client_t *)PyCapsule_GetPointer(
    pyaction_client, "rcl_action_client_t");
  if (!action_client) {
    return NULL;
  }

  bool is_available = false;
  rcl_ret_t ret = rcl_action_server_is_available(node, action_client, &is_available);
  if (RCL_RET_OK != ret) {
    return PyErr_Format(PyExc_RuntimeError,
             "Failed to check if action server is available: %s", rcl_get_error_string().str);
  }

  if (is_available) {
    Py_RETURN_TRUE;
  }
  Py_RETURN_FALSE;
}

#define SEND_SERVICE_REQUEST(Type) \
  PyObject * pyaction_client; \
  PyObject * pyrequest; \
  if (!PyArg_ParseTuple(args, "OO", & pyaction_client, & pyrequest)) { \
    return NULL; \
  } \
  rcl_action_client_t * action_client = (rcl_action_client_t *)PyCapsule_GetPointer( \
    pyaction_client, "rcl_action_client_t"); \
  if (!action_client) { \
    return NULL; \
  } \
  PyObject * pyrequest_type = PyObject_GetAttrString(pyrequest, "__class__"); \
  if (!pyrequest_type) { \
    return NULL; \
  } \
  PyObject * pymetaclass = PyObject_GetAttrString(pyrequest_type, "__class__"); \
  Py_DECREF(pyrequest_type); \
  if (!pymetaclass) { \
    return NULL; \
  } \
  create_ros_message_signature * create_ros_message = get_capsule_pointer( \
    pymetaclass, "_CREATE_ROS_MESSAGE"); \
  assert(create_ros_message != NULL && \
    "unable to retrieve create_ros_message function, type_support must not have been imported"); \
  destroy_ros_message_signature * destroy_ros_message = get_capsule_pointer( \
    pymetaclass, "_DESTROY_ROS_MESSAGE"); \
  assert(destroy_ros_message != NULL && \
    "unable to retrieve destroy_ros_message function, type_support must not have been imported"); \
  convert_from_py_signature * convert_from_py = get_capsule_pointer( \
    pymetaclass, "_CONVERT_FROM_PY"); \
  assert(convert_from_py != NULL && \
    "unable to retrieve convert_from_py function, type_support must not have been imported"); \
  Py_DECREF(pymetaclass); \
  void * raw_ros_request = create_ros_message(); \
  if (!raw_ros_request) { \
    return PyErr_NoMemory(); \
  } \
  if (!convert_from_py(pyrequest, raw_ros_request)) { \
    /* the function has set the Python error */ \
    destroy_ros_message(raw_ros_request); \
    return NULL; \
  } \
  int64_t sequence_number; \
  rcl_ret_t ret = rcl_action_send_ ## Type ## _request( \
    action_client, raw_ros_request, & sequence_number); \
  destroy_ros_message(raw_ros_request); \
  if (ret != RCL_RET_OK) { \
    PyErr_Format(PyExc_RuntimeError, \
      "Failed to send " #Type " request: %s", rcl_get_error_string().str); \
    rcl_reset_error(); \
    return NULL; \
  } \
  return PyLong_FromLongLong(sequence_number);

#define TAKE_SERVICE_RESPONSE(Type) \
  PyObject * pyaction_client; \
  PyObject * pyresponse_type; \
  if (!PyArg_ParseTuple(args, "OO", & pyaction_client, & pyresponse_type)) { \
    return NULL; \
  } \
  rcl_action_client_t * action_client = (rcl_action_client_t *)PyCapsule_GetPointer( \
    pyaction_client, "rcl_action_client_t"); \
  if (!action_client) { \
    return NULL; \
  } \
  PyObject * pymetaclass = PyObject_GetAttrString(pyresponse_type, "__class__"); \
  if (!pymetaclass) { \
    return NULL; \
  } \
  create_ros_message_signature * create_ros_message = get_capsule_pointer( \
    pymetaclass, "_CREATE_ROS_MESSAGE"); \
  assert(create_ros_message != NULL && \
    "unable to retrieve create_ros_message function, type_support mustn't have been imported"); \
  destroy_ros_message_signature * destroy_ros_message = get_capsule_pointer( \
    pymetaclass, "_DESTROY_ROS_MESSAGE"); \
  assert(destroy_ros_message != NULL && \
    "unable to retrieve destroy_ros_message function, type_support mustn't have been imported"); \
  void * taken_response = create_ros_message(); \
  if (!taken_response) { \
    /* the function has set the Python error */ \
    Py_DECREF(pymetaclass); \
    return NULL; \
  } \
  rmw_request_id_t * header = (rmw_request_id_t *)PyMem_Malloc(sizeof(rmw_request_id_t)); \
  if (!header) { \
    Py_DECREF(pymetaclass); \
    return PyErr_NoMemory(); \
  } \
  rcl_ret_t ret = rcl_action_take_ ## Type ## _response(action_client, header, taken_response); \
  int64_t sequence = header->sequence_number; \
  PyMem_Free(header); \
  /* Create the tuple to return */ \
  PyObject * pytuple = PyTuple_New(2); \
  if (!pytuple) { \
    Py_DECREF(pymetaclass); \
    return NULL; \
  } \
  if (ret != RCL_RET_OK) { \
    Py_INCREF(Py_None); \
    PyTuple_SET_ITEM(pytuple, 0, Py_None); \
    Py_INCREF(Py_None); \
    PyTuple_SET_ITEM(pytuple, 1, Py_None); \
    Py_DECREF(pymetaclass); \
    destroy_ros_message(taken_response); \
    return pytuple; \
  } \
  convert_to_py_signature * convert_to_py = get_capsule_pointer(pymetaclass, "_CONVERT_TO_PY"); \
  Py_DECREF(pymetaclass); \
  PyObject * pytaken_response = convert_to_py(taken_response); \
  destroy_ros_message(taken_response); \
  if (!pytaken_response) { \
    /* the function has set the Python error */ \
    Py_DECREF(pytuple); \
    return NULL; \
  } \
  PyObject * pysequence = PyLong_FromLongLong(sequence); \
  if (!pysequence) { \
    Py_DECREF(pytaken_response); \
    Py_DECREF(pytuple); \
    return NULL; \
  } \
  PyTuple_SET_ITEM(pytuple, 0, pysequence); \
  PyTuple_SET_ITEM(pytuple, 1, pytaken_response); \
  return pytuple; \

/// Send an action goal request.
/**
 * Raises AttributeError if there is an issue parsing the pygoal_request type.
 * Raises RuntimeError on failure.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pygoal_request The request message to send.
 * \return sequence_number PyLong object representing the index of the sent request, or
 * \return NULL if there is a failure.
 */
static PyObject *
rclpy_action_send_goal_request(PyObject * Py_UNUSED(self), PyObject * args)
{
  SEND_SERVICE_REQUEST(goal)
}

/// Take an action goal response.
/**
 * Raises AttributeError if there is an issue parsing the pygoal_response type.
 * Raises RuntimeError if the underlying rcl library returns an error when taking the response.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pygoal_response_type An instance of the response message type to take.
 * \return 2-tuple sequence number and received response, or
 * \return None if there is no response, or
 * \return NULL if there is a failure.
 */
static PyObject *
rclpy_action_take_goal_response(PyObject * Py_UNUSED(self), PyObject * args)
{
  TAKE_SERVICE_RESPONSE(goal)
}

/// Send an action result request.
/**
 * Raises AttributeError if there is an issue parsing the pyresult_request type.
 * Raises RuntimeError if the underlying rcl library returns an error when sending the request.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pyresult_request The request message to send.
 * \return sequence_number PyLong object representing the index of the sent request, or
 * \return NULL if there is a failure.
 */
static PyObject *
rclpy_action_send_result_request(PyObject * Py_UNUSED(self), PyObject * args)
{
  SEND_SERVICE_REQUEST(result);
}

/// Take an action result response.
/**
 * Raises AttributeError if there is an issue parsing the pyresult_response type.
 * Raises RuntimeError if the underlying rcl library returns an error when taking the response.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pyresult_response_type An instance of the response message type to take.
 * \return 2-tuple sequence number and received response, or
 * \return None if there is no response, or
 * \return NULL if there is a failure.
 */
static PyObject *
rclpy_action_take_result_response(PyObject * Py_UNUSED(self), PyObject * args)
{
  TAKE_SERVICE_RESPONSE(result);
}

/// Send an action cancel request.
/**
 * Raises AttributeError if there is an issue parsing the pycancel_request type.
 * Raises RuntimeError if the underlying rcl library returns an error when sending the request.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pycancel_request The request message to send.
 * \return sequence_number PyLong object representing the index of the sent request, or
 * \return NULL if there is a failure.
 */
static PyObject *
rclpy_action_send_cancel_request(PyObject * Py_UNUSED(self), PyObject * args)
{
  SEND_SERVICE_REQUEST(cancel)
}

/// Take an action cancel response.
/**
 * Raises AttributeError if there is an issue parsing the pycancel_response type.
 * Raises RuntimeError if the underlying rcl library returns an error when taking the response.
 *
 * \param[in] pyaction_client The action client to use when sending the request.
 * \param[in] pycancel_response_type An instance of the response message type to take.
 * \return 2-tuple sequence number and received response, or
 * \return None if there is no response, or
 * \return NULL if there is a failure.
 */
static PyObject *
rclpy_action_take_cancel_response(PyObject * Py_UNUSED(self), PyObject * args)
{
  TAKE_SERVICE_RESPONSE(cancel)
}

#define TAKE_MESSAGE(Type) \
  PyObject * pyaction_client; \
  PyObject * pymsg_type; \
  if (!PyArg_ParseTuple(args, "OO", & pyaction_client, & pymsg_type)) { \
    return NULL; \
  } \
  rcl_action_client_t * action_client = (rcl_action_client_t *)PyCapsule_GetPointer( \
    pyaction_client, "rcl_action_client_t"); \
  if (!action_client) { \
    return NULL; \
  } \
  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__"); \
  if (!pymetaclass) { \
    return NULL; \
  } \
  create_ros_message_signature * create_ros_message = get_capsule_pointer( \
    pymetaclass, "_CREATE_ROS_MESSAGE"); \
  assert(create_ros_message != NULL && \
    "unable to retrieve create_ros_message function, type_support must not have been imported"); \
  destroy_ros_message_signature * destroy_ros_message = get_capsule_pointer( \
    pymetaclass, "_DESTROY_ROS_MESSAGE"); \
  assert(destroy_ros_message != NULL && \
    "unable to retrieve destroy_ros_message function, type_support must not have been imported"); \
  convert_to_py_signature * convert_to_py = get_capsule_pointer(pymetaclass, "_CONVERT_TO_PY"); \
  Py_DECREF(pymetaclass); \
  void * taken_msg = create_ros_message(); \
  if (!taken_msg) { \
    return PyErr_NoMemory(); \
  } \
  rcl_ret_t ret = rcl_action_take_ ## Type(action_client, taken_msg); \
  if (ret != RCL_RET_OK) { \
    if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) { \
      /* if take failed, just do nothing */ \
      destroy_ros_message(taken_msg); \
      Py_RETURN_NONE; \
    } \
    PyErr_Format(PyExc_RuntimeError, \
      "Failed to take " #Type " with an action client: %s", rcl_get_error_string().str); \
    rcl_reset_error(); \
    destroy_ros_message(taken_msg); \
    return NULL; \
  } \
  PyObject * pytaken_msg = convert_to_py(taken_msg); \
  destroy_ros_message(taken_msg); \
  return pytaken_msg;

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
static PyObject *
rclpy_action_take_feedback(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_take_status(PyObject * Py_UNUSED(self), PyObject * args)
{
  TAKE_MESSAGE(status)
}

/// Define the public methods of this module
static PyMethodDef rclpy_action_methods[] = {
  {
    "rclpy_action_destroy_entity", rclpy_action_destroy_entity, METH_VARARGS,
    "Destroy a rclpy_action entity."
  },
  {
    "rclpy_action_get_rmw_qos_profile", rclpy_action_get_rmw_qos_profile, METH_VARARGS,
    "Get an action RMW QoS profile."
  },
  {
    "rclpy_action_wait_set_add", rclpy_action_wait_set_add, METH_VARARGS,
    "Add an action entitiy to a wait set."
  },
  {
    "rclpy_action_wait_set_get_num_entities", rclpy_action_wait_set_get_num_entities, METH_VARARGS,
    "Get the number of wait set entities for an action entitity."
  },
  {
    "rclpy_action_wait_set_is_ready", rclpy_action_wait_set_is_ready, METH_VARARGS,
    "Check if an action entity has any sub-entities ready in a wait set."
  },
  {
    "rclpy_action_create_client", rclpy_action_create_client, METH_VARARGS,
    "Create an action client."
  },
  {
    "rclpy_action_server_is_available", rclpy_action_server_is_available, METH_VARARGS,
    "Check if an action server is available for a given client."
  },
  {
    "rclpy_action_send_goal_request", rclpy_action_send_goal_request, METH_VARARGS,
    "Send a goal request."
  },
  {
    "rclpy_action_take_goal_response", rclpy_action_take_goal_response, METH_VARARGS,
    "Take a goal response."
  },
  {
    "rclpy_action_send_result_request", rclpy_action_send_result_request, METH_VARARGS,
    "Send a result request."
  },
  {
    "rclpy_action_take_result_response", rclpy_action_take_result_response, METH_VARARGS,
    "Take a result response."
  },
  {
    "rclpy_action_send_cancel_request", rclpy_action_send_cancel_request, METH_VARARGS,
    "Send a cancel request."
  },
  {
    "rclpy_action_take_cancel_response", rclpy_action_take_cancel_response, METH_VARARGS,
    "Take a cancel response."
  },
  {
    "rclpy_action_take_feedback", rclpy_action_take_feedback, METH_VARARGS,
    "Take a feedback message."
  },
  {
    "rclpy_action_take_status", rclpy_action_take_status, METH_VARARGS,
    "Take a status message."
  },

  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rclpy_action__doc__,
  "ROS 2 Python Action library.");

/// Define the Python module
static struct PyModuleDef _rclpy_action_module = {
  PyModuleDef_HEAD_INIT,
  "_rclpy_action",
  rclpy_action__doc__,
  -1,   /* -1 means that the module keeps state in global variables */
  rclpy_action_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy_action(void)
{
  return PyModule_Create(&_rclpy_action_module);
}
