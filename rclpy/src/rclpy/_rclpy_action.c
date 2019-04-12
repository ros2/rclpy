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

static PyObject *
rclpy_action_destroy_server_goal_handle(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pygoal_handle;

  if (!PyArg_ParseTuple(args, "O", &pygoal_handle)) {
    return NULL;
  }

  rcl_action_goal_handle_t * goal_handle = (rcl_action_goal_handle_t *)PyCapsule_GetPointer(
    pygoal_handle, "rcl_action_goal_handle_t");
  if (!goal_handle) {
    return NULL;
  }

  rcl_ret_t ret = rcl_action_goal_handle_fini(goal_handle);
  if (RCL_RET_OK != ret) {
    PyErr_Format(
      PyExc_RuntimeError, "Error destroying action goal handle: %s", rcl_get_error_string().str);
    rcl_reset_error();
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
static PyObject *
rclpy_action_create_server(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pyclock;
  PyObject * pyaction_type;
  PyObject * pyaction_name;
  PyObject * pygoal_service_qos;
  PyObject * pyresult_service_qos;
  PyObject * pycancel_service_qos;
  PyObject * pyfeedback_topic_qos;
  PyObject * pystatus_topic_qos;
  double result_timeout = 0.0;

  int parse_tuple_result = PyArg_ParseTuple(
    args,
    "OOOOOOOOOd",
    &pynode,
    &pyclock,
    &pyaction_type,
    &pyaction_name,
    &pygoal_service_qos,
    &pyresult_service_qos,
    &pycancel_service_qos,
    &pyfeedback_topic_qos,
    &pystatus_topic_qos,
    &result_timeout);

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

  rcl_clock_t * clock = (rcl_clock_t *) PyCapsule_GetPointer(pyclock, "rcl_clock_t");
  if (!clock) {
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

  rcl_action_server_options_t action_server_ops = rcl_action_server_get_default_options();

  OPTIONS_COPY_QOS_PROFILE(action_server_ops, goal_service_qos);
  OPTIONS_COPY_QOS_PROFILE(action_server_ops, result_service_qos);
  OPTIONS_COPY_QOS_PROFILE(action_server_ops, cancel_service_qos);
  OPTIONS_COPY_QOS_PROFILE(action_server_ops, feedback_topic_qos);
  OPTIONS_COPY_QOS_PROFILE(action_server_ops, status_topic_qos);
  action_server_ops.result_timeout.nanoseconds = (rcl_duration_value_t)RCL_S_TO_NS(result_timeout);

  rcl_action_server_t * action_server =
    (rcl_action_server_t *)PyMem_Malloc(sizeof(rcl_action_server_t));
  if (!action_server) {
    return PyErr_NoMemory();
  }
  *action_server = rcl_action_get_zero_initialized_server();
  rcl_ret_t ret = rcl_action_server_init(
    action_server,
    node,
    clock,
    ts,
    action_name,
    &action_server_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_ACTION_NAME_INVALID) {
      PyErr_Format(PyExc_ValueError,
        "Failed to create action server due to invalid topic name '%s': %s",
        action_name, rcl_get_error_string().str);
    } else {
      PyErr_Format(PyExc_RuntimeError,
        "Failed to create action server: %s", rcl_get_error_string().str);
    }
    PyMem_Free(action_server);
    rcl_reset_error();
    return NULL;
  }

  return PyCapsule_New(action_server, "rcl_action_server_t", NULL);
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
  destroy_ros_message_signature * destroy_ros_message = NULL; \
  void * raw_ros_request = rclpy_convert_from_py(pyrequest, & destroy_ros_message); \
  if (!raw_ros_request) { \
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

#define SEND_SERVICE_RESPONSE(Type) \
  PyObject * pyaction_server; \
  PyObject * pyheader; \
  PyObject * pyresponse; \
  if (!PyArg_ParseTuple(args, "OOO", & pyaction_server, & pyheader, & pyresponse)) { \
    return NULL; \
  } \
  rcl_action_server_t * action_server = (rcl_action_server_t *)PyCapsule_GetPointer( \
    pyaction_server, "rcl_action_server_t"); \
  if (!action_server) { \
    return NULL; \
  } \
  rmw_request_id_t * header = (rmw_request_id_t *)PyCapsule_GetPointer( \
    pyheader, "rmw_request_id_t"); \
  if (!header) { \
    return NULL; \
  } \
  destroy_ros_message_signature * destroy_ros_message = NULL; \
  void * raw_ros_response = rclpy_convert_from_py(pyresponse, & destroy_ros_message); \
  if (!raw_ros_response) { \
    return NULL; \
  } \
  rcl_ret_t ret = rcl_action_send_ ## Type ## _response(action_server, header, raw_ros_response); \
  destroy_ros_message(raw_ros_response); \
  if (ret != RCL_RET_OK) { \
    PyErr_Format(PyExc_RuntimeError, \
      "Failed to send " #Type " response: %s", rcl_get_error_string().str); \
    rcl_reset_error(); \
    return NULL; \
  } \
  Py_RETURN_NONE;

#define TAKE_SERVICE_REQUEST(Type) \
  PyObject * pyaction_server; \
  PyObject * pymsg_type; \
  if (!PyArg_ParseTuple(args, "OO", & pyaction_server, & pymsg_type)) { \
    return NULL; \
  } \
  rcl_action_server_t * action_server = (rcl_action_server_t *)PyCapsule_GetPointer( \
    pyaction_server, "rcl_action_server_t"); \
  if (!action_server) { \
    return NULL; \
  } \
  destroy_ros_message_signature * destroy_ros_message = NULL; \
  void * taken_msg = rclpy_create_from_py(pymsg_type, & destroy_ros_message); \
  if (!taken_msg) { \
    /* the function has set the Python error */ \
    return NULL; \
  } \
  rmw_request_id_t * header = (rmw_request_id_t *)PyMem_Malloc(sizeof(rmw_request_id_t)); \
  if (!header) { \
    destroy_ros_message(taken_msg); \
    return PyErr_NoMemory(); \
  } \
  rcl_ret_t ret = rcl_action_take_ ## Type ## _request(action_server, header, taken_msg); \
  /* Create the tuple to return */ \
  PyObject * pytuple = PyTuple_New(2); \
  if (!pytuple) { \
    destroy_ros_message(taken_msg); \
    PyMem_Free(header); \
    return NULL; \
  } \
  if (ret != RCL_RET_OK) { \
    Py_INCREF(Py_None); \
    PyTuple_SET_ITEM(pytuple, 0, Py_None); \
    Py_INCREF(Py_None); \
    PyTuple_SET_ITEM(pytuple, 1, Py_None); \
    destroy_ros_message(taken_msg); \
    PyMem_Free(header); \
    if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED && ret != RCL_RET_ACTION_SERVER_TAKE_FAILED) { \
      PyErr_Format(PyExc_RuntimeError, \
        "Failed to take " #Type ": %s", rcl_get_error_string().str); \
      rcl_reset_error(); \
      return NULL; \
    } \
    return pytuple; \
  } \
  PyObject * pytaken_msg = rclpy_convert_to_py(taken_msg, pymsg_type); \
  destroy_ros_message(taken_msg); \
  if (!pytaken_msg) { \
    Py_DECREF(pytuple); \
    PyMem_Free(header); \
    return NULL; \
  } \
  PyObject * pyheader = PyCapsule_New(header, "rmw_request_id_t", NULL); \
  if (!pyheader) { \
    Py_DECREF(pytaken_msg); \
    Py_DECREF(pytuple); \
    PyMem_Free(header); \
    return NULL; \
  } \
  PyTuple_SET_ITEM(pytuple, 0, pyheader); \
  PyTuple_SET_ITEM(pytuple, 1, pytaken_msg); \
  return pytuple; \

#define TAKE_SERVICE_RESPONSE(Type) \
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
  destroy_ros_message_signature * destroy_ros_message = NULL; \
  void * taken_msg = rclpy_create_from_py(pymsg_type, & destroy_ros_message); \
  if (!taken_msg) { \
    return NULL; \
  } \
  rmw_request_id_t * header = (rmw_request_id_t *)PyMem_Malloc(sizeof(rmw_request_id_t)); \
  if (!header) { \
    destroy_ros_message(taken_msg); \
    return PyErr_NoMemory(); \
  } \
  rcl_ret_t ret = rcl_action_take_ ## Type ## _response(action_client, header, taken_msg); \
  int64_t sequence = header->sequence_number; \
  PyMem_Free(header); \
  /* Create the tuple to return */ \
  PyObject * pytuple = PyTuple_New(2); \
  if (!pytuple) { \
    destroy_ros_message(taken_msg); \
    return NULL; \
  } \
  if (ret != RCL_RET_OK) { \
    Py_INCREF(Py_None); \
    PyTuple_SET_ITEM(pytuple, 0, Py_None); \
    Py_INCREF(Py_None); \
    PyTuple_SET_ITEM(pytuple, 1, Py_None); \
    destroy_ros_message(taken_msg); \
    if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED && ret != RCL_RET_ACTION_SERVER_TAKE_FAILED) { \
      PyErr_Format(PyExc_RuntimeError, \
        "Failed to take " #Type ": %s", rcl_get_error_string().str); \
      rcl_reset_error(); \
      return NULL; \
    } \
    return pytuple; \
  } \
  PyObject * pytaken_msg = rclpy_convert_to_py(taken_msg, pymsg_type); \
  destroy_ros_message(taken_msg); \
  if (!pytaken_msg) { \
    Py_DECREF(pytuple); \
    return NULL; \
  } \
  PyObject * pysequence = PyLong_FromLongLong(sequence); \
  if (!pysequence) { \
    Py_DECREF(pytaken_msg); \
    Py_DECREF(pytuple); \
    return NULL; \
  } \
  PyTuple_SET_ITEM(pytuple, 0, pysequence); \
  PyTuple_SET_ITEM(pytuple, 1, pytaken_msg); \
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
static PyObject *
rclpy_action_send_goal_request(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_take_goal_request(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_send_goal_response(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_take_goal_response(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_send_result_request(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_take_result_request(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_send_result_response(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_take_result_response(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_send_cancel_request(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_take_cancel_request(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_send_cancel_response(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_take_cancel_response(PyObject * Py_UNUSED(self), PyObject * args)
{
  TAKE_SERVICE_RESPONSE(cancel)
}

#define PUBLISH_MESSAGE(Type) \
  PyObject * pyaction_server; \
  PyObject * pymsg; \
  if (!PyArg_ParseTuple(args, "OO", & pyaction_server, & pymsg)) { \
    return NULL; \
  } \
  rcl_action_server_t * action_server = (rcl_action_server_t *)PyCapsule_GetPointer( \
    pyaction_server, "rcl_action_server_t"); \
  if (!action_server) { \
    return NULL; \
  } \
  destroy_ros_message_signature * destroy_ros_message = NULL; \
  void * raw_ros_message = rclpy_convert_from_py(pymsg, & destroy_ros_message); \
  if (!raw_ros_message) { \
    return NULL; \
  } \
  rcl_ret_t ret = rcl_action_publish_ ## Type(action_server, raw_ros_message); \
  destroy_ros_message(raw_ros_message); \
  if (ret != RCL_RET_OK) { \
    PyErr_Format(PyExc_RuntimeError, \
      "Failed to publish " #Type " with an action server: %s", rcl_get_error_string().str); \
    rcl_reset_error(); \
    return NULL; \
  } \
  Py_RETURN_NONE;

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
  destroy_ros_message_signature * destroy_ros_message = NULL; \
  void * taken_msg = rclpy_create_from_py(pymsg_type, & destroy_ros_message); \
  if (!taken_msg) { \
    return NULL; \
  } \
  rcl_ret_t ret = rcl_action_take_ ## Type(action_client, taken_msg); \
  if (ret != RCL_RET_OK) { \
    destroy_ros_message(taken_msg); \
    if (ret != RCL_RET_ACTION_CLIENT_TAKE_FAILED) { \
      /* if take failed, just do nothing */ \
      Py_RETURN_NONE; \
    } \
    PyErr_Format(PyExc_RuntimeError, \
      "Failed to take " #Type " with an action client: %s", rcl_get_error_string().str); \
    rcl_reset_error(); \
    return NULL; \
  } \
  PyObject * pytaken_msg = rclpy_convert_to_py(taken_msg, pymsg_type); \
  destroy_ros_message(taken_msg); \
  return pytaken_msg;

/// Publish a feedback message from a given action server.
/**
 * Raises AttributeError if there is an issue parsing the pyfeedback_msg.
 * Raises RuntimeError on failure while publishing a feedback message.
 *
 * \param[in] pyaction_server Capsule pointing to the action server to publish the message.
 * \param[in] pyfeedback_msg The feedback message to publish.
 * \return None
 */
static PyObject *
rclpy_action_publish_feedback(PyObject * Py_UNUSED(self), PyObject * args)
{
  PUBLISH_MESSAGE(feedback)
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
static PyObject *
rclpy_action_take_feedback(PyObject * Py_UNUSED(self), PyObject * args)
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
static PyObject *
rclpy_action_publish_status(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyaction_server;

  if (!PyArg_ParseTuple(args, "O", &pyaction_server)) {
    return NULL;
  }

  rcl_action_server_t * action_server = (rcl_action_server_t *)PyCapsule_GetPointer(
    pyaction_server, "rcl_action_server_t");
  if (!action_server) {
    return NULL;
  }

  rcl_action_goal_status_array_t status_message =
    rcl_action_get_zero_initialized_goal_status_array();
  rcl_ret_t ret = rcl_action_get_goal_status_array(action_server, &status_message);
  if (RCL_RET_OK != ret) {
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed get goal status array: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  ret = rcl_action_publish_status(action_server, &status_message);

  if (RCL_RET_OK != ret) {
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed publish goal status array: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  Py_RETURN_NONE;
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

static PyObject *
rclpy_action_accept_new_goal(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyaction_server;
  PyObject * pygoal_info_msg;

  if (!PyArg_ParseTuple(args, "OO", &pyaction_server, &pygoal_info_msg)) {
    return NULL;
  }

  rcl_action_server_t * action_server = (rcl_action_server_t *)PyCapsule_GetPointer(
    pyaction_server, "rcl_action_server_t");
  if (!action_server) {
    return NULL;
  }

  destroy_ros_message_signature * destroy_ros_message = NULL;
  rcl_action_goal_info_t * goal_info_msg = (rcl_action_goal_info_t *)rclpy_convert_from_py(
    pygoal_info_msg, &destroy_ros_message);
  if (!goal_info_msg) {
    return NULL;
  }

  rcl_action_goal_handle_t * goal_handle = rcl_action_accept_new_goal(
    action_server, goal_info_msg);
  destroy_ros_message(goal_info_msg);
  if (!goal_handle) {
    PyErr_Format(PyExc_RuntimeError, "Failed to accept new goal: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  return PyCapsule_New(goal_handle, "rcl_action_goal_handle_t", NULL);
}

static PyObject *
rclpy_action_notify_goal_done(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyaction_server;

  if (!PyArg_ParseTuple(args, "O", &pyaction_server)) {
    return NULL;
  }

  rcl_action_server_t * action_server = (rcl_action_server_t *)PyCapsule_GetPointer(
    pyaction_server, "rcl_action_server_t");
  if (!action_server) {
    return NULL;
  }

  rcl_ret_t ret = rcl_action_notify_goal_done(action_server);
  if (RCL_RET_OK != ret) {
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to notfiy action server of goal done: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

#define MULTI_DECREF(Arr, Size) \
  for (size_t i = 0; i < Size; ++i) { \
    Py_DECREF(Arr[i]); \
  }

/// Convert from a Python GoalEvent code to an rcl goal event code.
/**
 *  Note, this this function makes the assumption that no event code has the value -1.
 *  \param[in] pyevent The Python GoalEvent code.
 *  \return The rcl equivalent of the Python GoalEvent code, or
 *  \return -1 on failure.
 */
static int
convert_from_py_goal_event(const int64_t pyevent)
{
  // Holds references to PyObjects that should have references decremented
  PyObject * to_decref[11];
  // The number of objects in the decref list
  size_t num_to_decref = 0;

  PyObject * pyaction_server_module = PyImport_ImportModule("rclpy.action.server");
  if (!pyaction_server_module) {
    return -1;
  }

  PyObject * pygoal_event_class = PyObject_GetAttrString(pyaction_server_module, "GoalEvent");
  Py_DECREF(pyaction_server_module);
  if (!pygoal_event_class) {
    return -1;
  }
  to_decref[num_to_decref++] = pygoal_event_class;

  PyObject * pyexecute = PyObject_GetAttrString(pygoal_event_class, "EXECUTE");
  if (!pyexecute) {
    MULTI_DECREF(to_decref, num_to_decref)
    return -1;
  }
  to_decref[num_to_decref++] = pyexecute;

  PyObject * pycancel = PyObject_GetAttrString(pygoal_event_class, "CANCEL");
  if (!pycancel) {
    MULTI_DECREF(to_decref, num_to_decref)
    return -1;
  }
  to_decref[num_to_decref++] = pycancel;

  PyObject * pyset_succeeded = PyObject_GetAttrString(pygoal_event_class, "SET_SUCCEEDED");
  if (!pyset_succeeded) {
    MULTI_DECREF(to_decref, num_to_decref);
    return -1;
  }
  to_decref[num_to_decref++] = pyset_succeeded;

  PyObject * pyset_aborted = PyObject_GetAttrString(pygoal_event_class, "SET_ABORTED");
  if (!pyset_aborted) {
    MULTI_DECREF(to_decref, num_to_decref)
    return -1;
  }
  to_decref[num_to_decref++] = pyset_aborted;

  PyObject * pyset_canceled = PyObject_GetAttrString(pygoal_event_class, "SET_CANCELED");
  if (!pyset_canceled) {
    MULTI_DECREF(to_decref, num_to_decref)
    return -1;
  }
  to_decref[num_to_decref++] = pyset_canceled;

  PyObject * pyexecute_val = PyObject_GetAttrString(pyexecute, "value");
  if (!pyexecute_val) {
    MULTI_DECREF(to_decref, num_to_decref);
    return -1;
  }
  to_decref[num_to_decref++] = pyexecute_val;

  PyObject * pycancel_val = PyObject_GetAttrString(pycancel, "value");
  if (!pycancel_val) {
    MULTI_DECREF(to_decref, num_to_decref);
    return -1;
  }
  to_decref[num_to_decref++] = pycancel_val;

  PyObject * pyset_succeeded_val = PyObject_GetAttrString(pyset_succeeded, "value");
  if (!pyset_succeeded_val) {
    MULTI_DECREF(to_decref, num_to_decref);
    return -1;
  }
  to_decref[num_to_decref++] = pyset_succeeded_val;

  PyObject * pyset_aborted_val = PyObject_GetAttrString(pyset_aborted, "value");
  if (!pyset_aborted_val) {
    MULTI_DECREF(to_decref, num_to_decref);
    return -1;
  }
  to_decref[num_to_decref++] = pyset_aborted_val;

  PyObject * pyset_canceled_val = PyObject_GetAttrString(pyset_canceled, "value");
  if (!pyset_canceled_val) {
    MULTI_DECREF(to_decref, num_to_decref);
    return -1;
  }
  to_decref[num_to_decref++] = pyset_canceled_val;

  const int64_t execute = PyLong_AsLong(pyexecute_val);
  const int64_t cancel = PyLong_AsLong(pycancel_val);
  const int64_t set_succeeded = PyLong_AsLong(pyset_succeeded_val);
  const int64_t set_aborted = PyLong_AsLong(pyset_aborted_val);
  const int64_t set_canceled = PyLong_AsLong(pyset_canceled_val);
  MULTI_DECREF(to_decref, num_to_decref)

  if (execute == pyevent) {
    return GOAL_EVENT_EXECUTE;
  }
  if (cancel == pyevent) {
    return GOAL_EVENT_CANCEL;
  }
  if (set_succeeded == pyevent) {
    return GOAL_EVENT_SET_SUCCEEDED;
  }
  if (set_aborted == pyevent) {
    return GOAL_EVENT_SET_ABORTED;
  }
  if (set_canceled == pyevent) {
    return GOAL_EVENT_SET_CANCELED;
  }

  PyErr_Format(
    PyExc_RuntimeError, "Error converting goal event type: unknown goal event '%d'", pyevent);
  return -1;
}

static PyObject *
rclpy_action_update_goal_state(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pygoal_handle;
  int64_t pyevent;

  if (!PyArg_ParseTuple(args, "OL", &pygoal_handle, &pyevent)) {
    return NULL;
  }

  rcl_action_goal_handle_t * goal_handle = (rcl_action_goal_handle_t *)PyCapsule_GetPointer(
    pygoal_handle, "rcl_action_goal_handle_t");
  if (!goal_handle) {
    return NULL;
  }

  int event = convert_from_py_goal_event(pyevent);
  if (event < 0) {
    return NULL;
  }

  rcl_ret_t ret = rcl_action_update_goal_state(goal_handle, event);
  if (RCL_RET_OK != ret) {
    PyErr_Format(
      PyExc_RuntimeError, "Failed to update goal state: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

static PyObject *
rclpy_action_goal_handle_is_active(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pygoal_handle;

  if (!PyArg_ParseTuple(args, "O", &pygoal_handle)) {
    return NULL;
  }

  rcl_action_goal_handle_t * goal_handle = (rcl_action_goal_handle_t *)PyCapsule_GetPointer(
    pygoal_handle, "rcl_action_goal_handle_t");
  if (!goal_handle) {
    return NULL;
  }

  bool is_active = rcl_action_goal_handle_is_active(goal_handle);
  if (is_active) {
    Py_RETURN_TRUE;
  }
  Py_RETURN_FALSE;
}

static PyObject *
rclpy_action_server_goal_exists(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyaction_server;
  PyObject * pygoal_info;

  if (!PyArg_ParseTuple(args, "OO", &pyaction_server, &pygoal_info)) {
    return NULL;
  }

  rcl_action_server_t * action_server = (rcl_action_server_t *)PyCapsule_GetPointer(
    pyaction_server, "rcl_action_server_t");
  if (!action_server) {
    return NULL;
  }

  destroy_ros_message_signature * destroy_ros_message = NULL;
  rcl_action_goal_info_t * goal_info = rclpy_convert_from_py(pygoal_info, &destroy_ros_message);
  if (!goal_info) {
    return NULL;
  }

  bool exists = rcl_action_server_goal_exists(action_server, goal_info);
  destroy_ros_message(goal_info);

  if (exists) {
    Py_RETURN_TRUE;
  }
  Py_RETURN_FALSE;
}

static PyObject *
rclpy_action_goal_handle_get_status(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pygoal_handle;

  if (!PyArg_ParseTuple(args, "O", &pygoal_handle)) {
    return NULL;
  }

  rcl_action_goal_handle_t * goal_handle = (rcl_action_goal_handle_t *)PyCapsule_GetPointer(
    pygoal_handle, "rcl_action_goal_handle_t");
  if (!goal_handle) {
    return NULL;
  }

  rcl_action_goal_state_t status;
  rcl_ret_t ret = rcl_action_goal_handle_get_status(goal_handle, &status);
  if (RCL_RET_OK != ret) {
    PyErr_Format(
      PyExc_RuntimeError, "Failed to get goal status: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  return PyLong_FromLong(status);
}

static PyObject *
rclpy_action_process_cancel_request(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyaction_server;
  PyObject * pycancel_request;
  PyObject * pycancel_response_type;

  if (!PyArg_ParseTuple(
      args,
      "OOO",
      &pyaction_server,
      &pycancel_request,
      &pycancel_response_type))
  {
    return NULL;
  }

  rcl_action_server_t * action_server = (rcl_action_server_t *)PyCapsule_GetPointer(
    pyaction_server, "rcl_action_server_t");
  if (!action_server) {
    return NULL;
  }

  destroy_ros_message_signature * destroy_cancel_request = NULL;
  rcl_action_cancel_request_t * cancel_request =
    (rcl_action_cancel_request_t *)rclpy_convert_from_py(pycancel_request, &destroy_cancel_request);
  if (!cancel_request) {
    return NULL;
  }

  rcl_action_cancel_response_t cancel_response = rcl_action_get_zero_initialized_cancel_response();
  rcl_ret_t ret = rcl_action_process_cancel_request(
    action_server, cancel_request, &cancel_response);
  destroy_cancel_request(cancel_request);
  if (RCL_RET_OK != ret) {
    ret = rcl_action_cancel_response_fini(&cancel_response);
    PyErr_Format(PyExc_RuntimeError,
      "Failed to process cancel request: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pycancel_response = rclpy_convert_to_py(&cancel_response.msg, pycancel_response_type);
  ret = rcl_action_cancel_response_fini(&cancel_response);
  if (!pycancel_response) {
    return NULL;
  }
  if (RCL_RET_OK != ret) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to finalize cancel response: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  return pycancel_response;
}

static PyObject *
rclpy_action_expire_goals(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyaction_server;
  int64_t max_num_goals;

  if (!PyArg_ParseTuple(args, "OL", &pyaction_server, &max_num_goals)) {
    return NULL;
  }

  rcl_action_server_t * action_server = (rcl_action_server_t *)PyCapsule_GetPointer(
    pyaction_server, "rcl_action_server_t");
  if (!action_server) {
    return NULL;
  }

  rcl_action_goal_info_t * expired_goals =
    (rcl_action_goal_info_t *)malloc(sizeof(rcl_action_goal_info_t) * max_num_goals);
  if (!expired_goals) {
    return PyErr_NoMemory();
  }
  size_t num_expired;
  rcl_ret_t ret = rcl_action_expire_goals(
    action_server, expired_goals, max_num_goals, &num_expired);
  if (RCL_RET_OK != ret) {
    PyErr_Format(PyExc_RuntimeError, "Failed to expire goals: %s", rcl_get_error_string().str);
    rcl_reset_error();
    free(expired_goals);
    return NULL;
  }

  // Get Python GoalInfo type
  PyObject * pyaction_msgs_module = PyImport_ImportModule("action_msgs.msg");
  if (!pyaction_msgs_module) {
    free(expired_goals);
    return NULL;
  }
  PyObject * pygoal_info_class = PyObject_GetAttrString(pyaction_msgs_module, "GoalInfo");
  Py_DECREF(pyaction_msgs_module);
  if (!pygoal_info_class) {
    free(expired_goals);
    return NULL;
  }
  PyObject * pygoal_info_type = PyObject_CallObject(pygoal_info_class, NULL);
  Py_DECREF(pygoal_info_class);
  if (!pygoal_info_type) {
    free(expired_goals);
    return NULL;
  }

  // Create a tuple of GoalInfo instances to return
  PyObject * result_tuple = PyTuple_New(num_expired);
  if (!result_tuple) {
    free(expired_goals);
    Py_DECREF(pygoal_info_type);
    return NULL;
  }
  // PyTuple_SetItem() returns 0 on success
  int set_result = 0;
  for (size_t i = 0; i < num_expired; ++i) {
    PyObject * pygoal_info = rclpy_convert_to_py(&(expired_goals[i]), pygoal_info_type);
    set_result += PyTuple_SetItem(result_tuple, i, pygoal_info);
  }

  free(expired_goals);
  Py_DECREF(pygoal_info_type);
  if (0 != set_result) {
    Py_DECREF(result_tuple);
    return NULL;
  }
  return result_tuple;
}


static PyObject *
rclpy_action_get_client_names_and_types_by_node(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  char * remote_node_name;
  char * remote_node_namespace;

  if (!PyArg_ParseTuple(args, "Oss", &pynode, &remote_node_name, &remote_node_namespace)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
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
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to get action client names and type: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pynames_and_types = rclpy_convert_to_py_names_and_types(&names_and_types);
  if (!rclpy_names_and_types_fini(&names_and_types)) {
    Py_XDECREF(pynames_and_types);
    return NULL;
  }
  return pynames_and_types;
}

static PyObject *
rclpy_action_get_server_names_and_types_by_node(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  char * remote_node_name;
  char * remote_node_namespace;

  if (!PyArg_ParseTuple(args, "Oss", &pynode, &remote_node_name, &remote_node_namespace)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
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
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to get action server names and type: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pynames_and_types = rclpy_convert_to_py_names_and_types(&names_and_types);
  if (!rclpy_names_and_types_fini(&names_and_types)) {
    Py_XDECREF(pynames_and_types);
    return NULL;
  }
  return pynames_and_types;
}

static PyObject *
rclpy_action_get_names_and_types(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "O", &pynode)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_names_and_types(node, &allocator, &names_and_types);
  if (RCL_RET_OK != ret) {
    PyErr_Format(
      PyExc_RuntimeError,
      "Failed to get action names and type: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pynames_and_types = rclpy_convert_to_py_names_and_types(&names_and_types);
  if (!rclpy_names_and_types_fini(&names_and_types)) {
    Py_XDECREF(pynames_and_types);
    return NULL;
  }
  return pynames_and_types;
}

/// Define the public methods of this module
static PyMethodDef rclpy_action_methods[] = {
  {
    "rclpy_action_destroy_entity", rclpy_action_destroy_entity, METH_VARARGS,
    "Destroy a rclpy_action entity."
  },
  {
    "rclpy_action_destroy_server_goal_handle",
    rclpy_action_destroy_server_goal_handle,
    METH_VARARGS,
    "Destroy a ServerGoalHandle."
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
    "rclpy_action_create_server", rclpy_action_create_server, METH_VARARGS,
    "Create an action server."
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
    "rclpy_action_take_goal_request", rclpy_action_take_goal_request, METH_VARARGS,
    "Take a goal request."
  },
  {
    "rclpy_action_send_goal_response", rclpy_action_send_goal_response, METH_VARARGS,
    "Send a goal response."
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
    "rclpy_action_take_result_request", rclpy_action_take_result_request, METH_VARARGS,
    "Take a result request."
  },
  {
    "rclpy_action_send_result_response", rclpy_action_send_result_response, METH_VARARGS,
    "Send a result response."
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
    "rclpy_action_take_cancel_request", rclpy_action_take_cancel_request, METH_VARARGS,
    "Take a cancel request."
  },
  {
    "rclpy_action_send_cancel_response", rclpy_action_send_cancel_response, METH_VARARGS,
    "Send a cancel response."
  },
  {
    "rclpy_action_take_cancel_response", rclpy_action_take_cancel_response, METH_VARARGS,
    "Take a cancel response."
  },
  {
    "rclpy_action_publish_feedback", rclpy_action_publish_feedback, METH_VARARGS,
    "Publish a feedback message."
  },
  {
    "rclpy_action_take_feedback", rclpy_action_take_feedback, METH_VARARGS,
    "Take a feedback message."
  },
  {
    "rclpy_action_publish_status", rclpy_action_publish_status, METH_VARARGS,
    "Publish a status message."
  },
  {
    "rclpy_action_take_status", rclpy_action_take_status, METH_VARARGS,
    "Take a status message."
  },
  {
    "rclpy_action_accept_new_goal", rclpy_action_accept_new_goal, METH_VARARGS,
    "Accept a new goal using an action server."
  },
  {
    "rclpy_action_notify_goal_done", rclpy_action_notify_goal_done, METH_VARARGS,
    "Notify and action server that a goal has reached a terminal state."
  },
  {
    "rclpy_action_update_goal_state", rclpy_action_update_goal_state, METH_VARARGS,
    "Update a goal state."
  },
  {
    "rclpy_action_goal_handle_is_active", rclpy_action_goal_handle_is_active, METH_VARARGS,
    "Check if a goal is active."
  },
  {
    "rclpy_action_server_goal_exists", rclpy_action_server_goal_exists, METH_VARARGS,
    "Check if a goal being tracked by an action server."
  },
  {
    "rclpy_action_goal_handle_get_status", rclpy_action_goal_handle_get_status, METH_VARARGS,
    "Get the status of a goal."
  },
  {
    "rclpy_action_process_cancel_request", rclpy_action_process_cancel_request, METH_VARARGS,
    "Process a cancel request to determine what goals should be canceled."
  },
  {
    "rclpy_action_expire_goals", rclpy_action_expire_goals, METH_VARARGS,
    "Expire goals associated with an action server."
  },
  {
    "rclpy_action_get_client_names_and_types_by_node",
    rclpy_action_get_client_names_and_types_by_node,
    METH_VARARGS,
    "Get action client names and types by node."
  },
  {
    "rclpy_action_get_server_names_and_types_by_node",
    rclpy_action_get_server_names_and_types_by_node,
    METH_VARARGS,
    "Get action server names and types by node."
  },
  {
    "rclpy_action_get_names_and_types",
    rclpy_action_get_names_and_types,
    METH_VARARGS,
    "Get action names and types."
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
