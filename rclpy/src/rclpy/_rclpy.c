// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <rcl/graph.h>
#include <rcl/node.h>
#include <rcl/rcl.h>
#include <rmw/rmw.h>
#include <rosidl_generator_c/message_type_support_struct.h>

#ifndef RMW_IMPLEMENTATION_SUFFIX
#error "RMW_IMPLEMENTATION_SUFFIX is required to be set for _rclpy.c"
#endif

/// Initialize rcl with default options, ignoring parameters
static PyObject *
rclpy_init(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  // TODO(esteve): parse args
  rcl_ret_t ret = rcl_init(0, NULL, rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to init: %s", rcl_get_error_string_safe());
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Create a node
/*
 * \param[in] node_name string name of the node to be created
 * \return NULL on failure
 *         Capsule pointing to the pointer of the created rcl_node_t * structure otherwise
 */
static PyObject *
rclpy_create_node(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * node_name;

  if (!PyArg_ParseTuple(args, "s", &node_name)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyMem_Malloc(sizeof(rcl_node_t));
  *node = rcl_get_zero_initialized_node();
  rcl_node_options_t default_options = rcl_node_get_default_options();
  rcl_ret_t ret = rcl_node_init(node, node_name, &default_options);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create node: %s", rcl_get_error_string_safe());
    return NULL;
  }
  PyObject * pynode = PyCapsule_New(node, NULL, NULL);
  return pynode;
}

/// Create a publisher
/*
 * This function will create a publisher and attach it to the provided topic name
 * This publisher will use the typesupport defined in the message module
 * provided as pymsg_type to send messages over the wire.
 *
 * \param[in] pynode Capsule pointing to the node to add the publisher to
 * \param[in] pymsg_type Message type associated with the publisher
 * \param[in] pytopic Python object containing the name of the topic
 * to attach the publisher to
 * \param[in] pyqos_profile QoSProfile object with the profile of this publisher
 * \return NULL on failure
 *         Capsule pointing to the pointer of the created rcl_publisher_t * structure otherwise
 */
static PyObject *
rclpy_create_publisher(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pymsg_type;
  PyObject * pytopic;
  PyObject * pyqos_profile;

  if (!PyArg_ParseTuple(args, "OOOO", &pynode, &pymsg_type, &pytopic, &pyqos_profile)) {
    return NULL;
  }

  assert(PyUnicode_Check(pytopic));

  char * topic = (char *)PyUnicode_1BYTE_DATA(pytopic);

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, NULL);

  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)PyCapsule_GetPointer(pyqos_profile, NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__");

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");

  rosidl_message_type_support_t * ts =
    (rosidl_message_type_support_t *)PyCapsule_GetPointer(pyts, NULL);

  rcl_publisher_t * publisher = (rcl_publisher_t *)PyMem_Malloc(sizeof(rcl_publisher_t));
  *publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  if (pyqos_profile) {
    publisher_ops.qos = *qos_profile;
  }

  rcl_ret_t ret = rcl_publisher_init(publisher, node, ts, topic, &publisher_ops);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create publisher: %s", rcl_get_error_string_safe());
    return NULL;
  }
  PyObject * pypublisher = PyCapsule_New(publisher, NULL, NULL);
  return pypublisher;
}

/// Publish a message
/*
 * \param[in] pypublisher Capsule pointing to the publisher
 * \param[in] pymsg message to send
 * \return NULL
 */
static PyObject *
rclpy_publish(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pypublisher;
  PyObject * pymsg;

  if (!PyArg_ParseTuple(args, "OO", &pypublisher, &pymsg)) {
    return NULL;
  }

  rcl_publisher_t * publisher = (rcl_publisher_t *)PyCapsule_GetPointer(pypublisher, NULL);

  PyObject * pymsg_type = PyObject_GetAttrString(pymsg, "__class__");

  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__");

  PyObject * pyconvert_from_py = PyObject_GetAttrString(pymetaclass, "_CONVERT_FROM_PY");

  typedef void * (* convert_from_py_signature)(PyObject *);
  convert_from_py_signature convert_from_py =
    (convert_from_py_signature)PyCapsule_GetPointer(pyconvert_from_py, NULL);

  void * raw_ros_message = convert_from_py(pymsg);

  rcl_ret_t ret = rcl_publish(publisher, raw_ros_message);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to publish: %s", rcl_get_error_string_safe());
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Create a subscription
/*
 * This function will create a subscription and attach it to the provided topic name
 * This subscription will use the typesupport defined in the message module
 * provided as pymsg_type to send messages over the wire.
 *
 * \param[in] pynode Capsule pointing to the node to add the subscriber to
 * \param[in] pymsg_type Message module associated with the subscriber
 * \param[in] pytopic Python object containing the name of the topic to attach the subscription to
 * \param[in] pyqos_profile QoSProfile Python object with the profile of this subscriber
 * \return NULL on failure
 *         List with 2 elements:
 *            first element: a Capsule pointing to the pointer of the created rcl_subscription_t * structure
 *            second element: an integer representing the memory address of the created rcl_subscription_t
 */
static PyObject *
rclpy_create_subscription(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pymsg_type;
  PyObject * pytopic;
  PyObject * pyqos_profile;

  if (!PyArg_ParseTuple(args, "OOOO", &pynode, &pymsg_type, &pytopic, &pyqos_profile)) {
    return NULL;
  }

  assert(PyUnicode_Check(pytopic));

  char * topic = (char *)PyUnicode_1BYTE_DATA(pytopic);

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, NULL);

  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)PyCapsule_GetPointer(pyqos_profile, NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__");

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");

  rosidl_message_type_support_t * ts =
    (rosidl_message_type_support_t *)PyCapsule_GetPointer(pyts, NULL);

  rcl_subscription_t * subscription =
    (rcl_subscription_t *)PyMem_Malloc(sizeof(rcl_subscription_t));
  *subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();

  if (pyqos_profile) {
    subscription_ops.qos = *qos_profile;
  }

  rcl_ret_t ret = rcl_subscription_init(subscription, node, ts, topic, &subscription_ops);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create subscriptions: %s", rcl_get_error_string_safe());
    return NULL;
  }
  PyObject * pysubscription = PyCapsule_New(subscription, NULL, NULL);
  PyObject * pylist = PyList_New(0);
  PyList_Append(pylist, pysubscription);
  PyList_Append(pylist, PyLong_FromUnsignedLongLong((uint64_t)&subscription->impl));

  return pylist;
}

/// Return the identifier of the current rmw_implementation
/*
 * \return string containing the identifier of the current rmw_implementation
 */
static PyObject *
rclpy_create_client(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pysrv_type;
  PyObject * pyservice_name;
  PyObject * pyqos_profile;

  if (!PyArg_ParseTuple(args, "OOOO", &pynode, &pysrv_type, &pyservice_name, &pyqos_profile)) {
    return NULL;
  }

  assert(PyUnicode_Check(pyservice_name));

  char * service_name = (char *)PyUnicode_1BYTE_DATA(pyservice_name);

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, NULL);

  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)PyCapsule_GetPointer(pyqos_profile, NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pysrv_type, "__class__");

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");

  rosidl_service_type_support_t * ts =
    (rosidl_service_type_support_t *)PyCapsule_GetPointer(pyts, NULL);

  rcl_client_t * client =
    (rcl_client_t *)PyMem_Malloc(sizeof(rcl_client_t));
  *client = rcl_get_zero_initialized_client();
  rcl_client_options_t client_ops = rcl_client_get_default_options();

  if (qos_profile) {
    client_ops.qos = *qos_profile;
  }

  rcl_ret_t ret = rcl_client_init(client, node, ts, service_name, &client_ops);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create client: %s", rcl_get_error_string_safe());
    return NULL;
  }
  PyObject * pyclient = PyCapsule_New(client, NULL, NULL);
  PyObject * pylist = PyList_New(0);
  PyList_Append(pylist, pyclient);
  PyList_Append(pylist, PyLong_FromUnsignedLongLong((uint64_t)&client->impl));

  return pylist;
}

static PyObject *
rclpy_send_request(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclient;
  PyObject * pyrequest;

  if (!PyArg_ParseTuple(args, "OO", &pyclient, &pyrequest)) {
    return NULL;
  }
  rcl_client_t * client = (rcl_client_t *)PyCapsule_GetPointer(pyclient, NULL);
  assert(client != NULL);

  PyObject * pyrequest_type = PyObject_GetAttrString(pyrequest, "__class__");
  assert(pyrequest_type != NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pyrequest_type, "__class__");
  assert(pymetaclass != NULL);

  PyObject * pyconvert_from_py = PyObject_GetAttrString(pymetaclass, "_CONVERT_FROM_PY");

  assert(pyconvert_from_py != NULL);
  typedef void * (* convert_from_py_signature)(PyObject *);
  convert_from_py_signature convert_from_py =
    (convert_from_py_signature)PyCapsule_GetPointer(pyconvert_from_py, NULL);

  assert(convert_from_py != NULL);
  void * raw_ros_request = convert_from_py(pyrequest);
  int64_t sequence_number;
  rcl_ret_t ret = rcl_send_request(client, raw_ros_request, &sequence_number);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to send request: %s", rcl_get_error_string_safe());
    return NULL;
  }

  return PyLong_FromLongLong(sequence_number);
}

static PyObject *
rclpy_create_service(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pysrv_type;
  PyObject * pyservice_name;
  PyObject * pyqos_profile;

  if (!PyArg_ParseTuple(args, "OOOO", &pynode, &pysrv_type, &pyservice_name, &pyqos_profile)) {
    return NULL;
  }

  assert(PyUnicode_Check(pyservice_name));

  char * service_name = (char *)PyUnicode_1BYTE_DATA(pyservice_name);

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, NULL);

  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)PyCapsule_GetPointer(pyqos_profile, NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pysrv_type, "__class__");

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");

  rosidl_service_type_support_t * ts =
    (rosidl_service_type_support_t *)PyCapsule_GetPointer(pyts, NULL);

  rcl_service_t * service =
    (rcl_service_t *)PyMem_Malloc(sizeof(rcl_service_t));
  *service = rcl_get_zero_initialized_service();
  rcl_service_options_t service_ops = rcl_service_get_default_options();

  if (qos_profile) {
    service_ops.qos = *qos_profile;
  }

  rcl_ret_t ret = rcl_service_init(service, node, ts, service_name, &service_ops);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create service: %s", rcl_get_error_string_safe());
    return NULL;
  }
  PyObject * pyservice = PyCapsule_New(service, NULL, NULL);
  PyObject * pylist = PyList_New(0);
  PyList_Append(pylist, pyservice);
  PyList_Append(pylist, PyLong_FromUnsignedLongLong((uint64_t)&service->impl));

  return pylist;
}

static PyObject *
rclpy_send_response(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyservice;
  PyObject * pyresponse;
  PyObject * pyheader;

  if (!PyArg_ParseTuple(args, "OOO", &pyservice, &pyresponse, &pyheader)) {
    return NULL;
  }
  rcl_service_t * service = (rcl_service_t *)PyCapsule_GetPointer(pyservice, NULL);
  assert(service != NULL);

  rmw_request_id_t * header = (rmw_request_id_t *)PyCapsule_GetPointer(pyheader, NULL);
  assert(service != NULL);
  PyObject * pyresponse_type = PyObject_GetAttrString(pyresponse, "__class__");
  assert(pyresponse_type != NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pyresponse_type, "__class__");
  assert(pymetaclass != NULL);

  PyObject * pyconvert_from_py = PyObject_GetAttrString(pymetaclass, "_CONVERT_FROM_PY");

  assert(pyconvert_from_py != NULL);
  typedef void * (* convert_from_py_signature)(PyObject *);
  convert_from_py_signature convert_from_py =
    (convert_from_py_signature)PyCapsule_GetPointer(pyconvert_from_py, NULL);

  assert(convert_from_py != NULL);
  void * raw_ros_response = convert_from_py(pyresponse);

  rcl_ret_t ret = rcl_send_response(service, header, raw_ros_response);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to send request: %s", rcl_get_error_string_safe());
    return NULL;
  }
  Py_RETURN_NONE;
}

static PyObject *
rclpy_destroy_node_entity(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * entity_type;
  PyObject * pyentity;
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "zOO", &entity_type, &pyentity, &pynode)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, NULL);
  rcl_ret_t ret;
  if (0 == strcmp(entity_type, "subscription")) {
    rcl_subscription_t * subscription = (rcl_subscription_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_subscription_fini(subscription, node);
  }
  else if (0 == strcmp(entity_type, "publisher")) {
    rcl_publisher_t * publisher = (rcl_publisher_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_publisher_fini(publisher, node);
  }
  else if (0 == strcmp(entity_type, "client")) {
    rcl_client_t * client = (rcl_client_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_client_fini(client, node);
  }
  else if (0 == strcmp(entity_type, "service")) {
    rcl_service_t * service = (rcl_service_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_service_fini(service, node);
  }
  else {
    PyErr_Format(PyExc_RuntimeError,
      "%s is not a known entity", entity_type);
    Py_RETURN_FALSE;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to fini %s: %s", entity_type, rcl_get_error_string_safe());
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

static PyObject *
rclpy_destroy_entity(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * entity_type;
  PyObject * pyentity;

  if (!PyArg_ParseTuple(args, "zO", &entity_type, &pyentity)) {
    return NULL;
  }

  rcl_ret_t ret;
  if (0 == strcmp(entity_type, "guard_condition")) {
    rcl_guard_condition_t * gc = (rcl_guard_condition_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_guard_condition_fini(gc);
  }
  else if (0 == strcmp(entity_type, "node")) {
    rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_node_fini(node);
  }
  else {
    PyErr_Format(PyExc_RuntimeError,
      "%s is not a known entity", entity_type);
    Py_RETURN_FALSE;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to fini %s: %s", entity_type, rcl_get_error_string_safe());
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

static PyObject *
rclpy_get_rmw_implementation_identifier(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  const char * rmw_implementation_identifier = rmw_get_implementation_identifier();

  PyObject * pyrmw_implementation_identifier = Py_BuildValue(
    "s", rmw_implementation_identifier);

  return pyrmw_implementation_identifier;
}

/// Return a Capsule pointing to a zero initialized rcl_wait_set_t structure
static PyObject *
rclpy_get_zero_initialized_wait_set(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyMem_Malloc(sizeof(rcl_wait_set_t));
  *wait_set = rcl_get_zero_initialized_wait_set();
  PyObject * pywait_set = PyCapsule_New(wait_set, NULL, NULL);

  return pywait_set;
}

/// Initialize a waitset
/*
 * \param[in] pywait_set Capsule pointing to the waitset structure
 * \param[in] node_name string name of the node to be created
 * \param[in] number_of_subscriptions int
 * \param[in] number_of_guard_conditions int
 * \param[in] number_of_guard_timers int
 * \param[in] UNUSED FOR NOW number_of_clients int
 * \param[in] UNUSED FOR NOW number_of_services int
 * \return NULL
 */
static PyObject *
rclpy_wait_set_init(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;
  unsigned PY_LONG_LONG number_of_subscriptions;
  unsigned PY_LONG_LONG number_of_guard_conditions;
  unsigned PY_LONG_LONG number_of_timers;
  unsigned PY_LONG_LONG number_of_clients;
  unsigned PY_LONG_LONG number_of_services;

  if (!PyArg_ParseTuple(
      args, "OKKKKK", &pywait_set, &number_of_subscriptions,
      &number_of_guard_conditions, &number_of_timers,
      &number_of_clients, &number_of_services))
  {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, NULL);

  // TODO(jacquelinekay) Services.
  rcl_ret_t ret = rcl_wait_set_init(
    wait_set, number_of_subscriptions, number_of_guard_conditions, number_of_timers,
    number_of_clients, number_of_services, rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to initialize wait set: %s", rcl_get_error_string_safe());
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Clear all the subscription pointers in the waitset
/*
 * \param[in] pywait_set Capsule pointing to the waitset structure
 * \return NULL
 */
static PyObject *
rclpy_wait_set_clear_entities(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * entity_type;
  PyObject * pywait_set;

  if (!PyArg_ParseTuple(args, "zO", &entity_type, &pywait_set)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, NULL);
  rcl_ret_t ret;
  if (0 == strcmp(entity_type, "subscription")) {
    ret = rcl_wait_set_clear_subscriptions(wait_set);
  }
  else if (0 == strcmp(entity_type, "client")) {
    ret = rcl_wait_set_clear_clients(wait_set);
  }
  else if (0 == strcmp(entity_type, "service")) {
    ret = rcl_wait_set_clear_services(wait_set);
  }
  else if (0 == strcmp(entity_type, "guard_condition")) {
    ret = rcl_wait_set_clear_guard_conditions(wait_set);
  }
  else {
    PyErr_Format(PyExc_RuntimeError,
      "%s is not a known entity", entity_type);
    Py_RETURN_FALSE;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to %s from waitset: %s", entity_type, rcl_get_error_string_safe());
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

static PyObject *
rclpy_wait_set_add_entity(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * entity_type;
  PyObject * pywait_set;
  PyObject * pyentity;

  if (!PyArg_ParseTuple(args, "zOO", &entity_type, &pywait_set, &pyentity)) {
    return NULL;
  }
  rcl_ret_t ret;
  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, NULL);
  if (0 == strcmp(entity_type, "subscription")) {
    rcl_subscription_t * subscription =
      (rcl_subscription_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_wait_set_add_subscription(wait_set, subscription);
  } else if (0 == strcmp(entity_type, "client")) {
    rcl_client_t * client =
      (rcl_client_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_wait_set_add_client(wait_set, client);
  } else if (0 == strcmp(entity_type, "service")) {
    rcl_service_t * service =
      (rcl_service_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_wait_set_add_service(wait_set, service);
  } else if (0 == strcmp(entity_type, "guard_condition")) {
    rcl_guard_condition_t * guard_condition =
      (rcl_guard_condition_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_wait_set_add_guard_condition(wait_set, guard_condition);
  } else {
    PyErr_Format(PyExc_RuntimeError,
      "%s is not a known entity", entity_type);
    Py_RETURN_FALSE;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to add %s to wait set: %s", entity_type, rcl_get_error_string_safe());
    return NULL;
  }
  Py_RETURN_NONE;
}

static PyObject *
rclpy_get_ready_subscriptions(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;
  if (!PyArg_ParseTuple(args, "O", &pywait_set)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, NULL);
  if (!wait_set) {
    PyErr_Format(PyExc_RuntimeError, "waiset is null");
    return NULL;
  }

  size_t sub_idx;
  PyObject * sub_ready_list = PyList_New(0);
  for (sub_idx = 0; sub_idx < wait_set->size_of_subscriptions; sub_idx++) {
    if (wait_set->subscriptions[sub_idx]) {
      PyList_Append(
        sub_ready_list,
        PyLong_FromUnsignedLongLong((uint64_t)&wait_set->subscriptions[sub_idx]->impl));
    }
  }

  return sub_ready_list;
}

/// Wait until timeout is reached or event happened
/*
 * This function will wait for an event to happen or for the timeout to expire.
 * A negative timeout means wait forever, a timeout of 0 means no wait
 * \param[in] pywait_set Capsule pointing to the waitset structure
 * \param[in] timeout optional time to wait before waking up (in nanoseconds)
 * \return NULL
 */
static PyObject *
rclpy_get_ready_clients(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;
  if (!PyArg_ParseTuple(args, "O", &pywait_set)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, NULL);
  if (!wait_set) {
    PyErr_Format(PyExc_RuntimeError, "waiset is null");
    return NULL;
  }

  size_t client_idx;
  PyObject * client_ready_list = PyList_New(0);
  for (client_idx = 0; client_idx < wait_set->size_of_clients; client_idx++) {
    if (wait_set->clients[client_idx]) {
      PyList_Append(
        client_ready_list,
        PyLong_FromUnsignedLongLong((uint64_t)&wait_set->clients[client_idx]->impl));
    }
  }

  return client_ready_list;
}

static PyObject *
rclpy_get_ready_services(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;
  if (!PyArg_ParseTuple(args, "O", &pywait_set)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, NULL);
  if (!wait_set) {
    PyErr_Format(PyExc_RuntimeError, "waiset is null\n");
    return NULL;
  }

  size_t service_idx;
  PyObject * service_ready_list = PyList_New(0);
  for (service_idx = 0; service_idx < wait_set->size_of_services; service_idx++) {
    if (wait_set->services[service_idx]) {
      PyList_Append(
        service_ready_list,
        PyLong_FromUnsignedLongLong((uint64_t)&wait_set->services[service_idx]->impl));
    }
  }

  return service_ready_list;
}

static PyObject *
rclpy_wait(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;
  PY_LONG_LONG timeout = -1;

  if (!PyArg_ParseTuple(args, "O|K", &pywait_set, &timeout)) {
    return NULL;
  }
  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, NULL);
  rcl_ret_t ret = rcl_wait(wait_set, timeout);
  if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to wait on wait set: %s", rcl_get_error_string_safe());
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Take a message from a given subscription
/*
 * \param[in] pysubscription Capsule pointing to the subscription to add
 * \param[in] pymsg_type Instance of the message type to take
 * \return Python message with all fields populated with received message
 */
static PyObject *
rclpy_take(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pysubscription;
  PyObject * pymsg_type;

  if (!PyArg_ParseTuple(args, "OO", &pysubscription, &pymsg_type)) {
    return NULL;
  }

  rcl_subscription_t * subscription =
    (rcl_subscription_t *)PyCapsule_GetPointer(pysubscription, NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__");

  PyObject * pyconvert_from_py = PyObject_GetAttrString(pymetaclass, "_CONVERT_FROM_PY");

  typedef void *(* convert_from_py_signature)(PyObject *);
  convert_from_py_signature convert_from_py =
    (convert_from_py_signature)PyCapsule_GetPointer(pyconvert_from_py, NULL);

  PyObject * pymsg = PyObject_CallObject(pymsg_type, NULL);

  void * taken_msg = convert_from_py(pymsg);

  rcl_ret_t ret = rcl_take(subscription, taken_msg, NULL);

  if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to take from a subscription: %s", rcl_get_error_string_safe());
    return NULL;
  }

  if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    PyObject * pyconvert_to_py = PyObject_GetAttrString(pymsg_type, "_CONVERT_TO_PY");

    typedef PyObject *(* convert_to_py_signature)(void *);
    convert_to_py_signature convert_to_py =
      (convert_to_py_signature)PyCapsule_GetPointer(pyconvert_to_py, NULL);

    PyObject * pytaken_msg = convert_to_py(taken_msg);

    Py_INCREF(pytaken_msg);

    return pytaken_msg;
  }
  // if take failed, just do nothing
  Py_RETURN_NONE;
}

/// Status of the the client library
/*
 * \return True if rcl is running properly, False otherwise
 */
static PyObject *
rclpy_take_response(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclient;
  PyObject * pyresponse_type;
  PY_LONG_LONG sequence_number;

  if (!PyArg_ParseTuple(args, "OOK", &pyclient, &pyresponse_type, &sequence_number)) {
    return NULL;
  }

  rcl_client_t * client =
    (rcl_client_t *)PyCapsule_GetPointer(pyclient, NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pyresponse_type, "__class__");

  PyObject * pyconvert_from_py = PyObject_GetAttrString(pymetaclass, "_CONVERT_FROM_PY");

  typedef void *(* convert_from_py_signature)(PyObject *);
  convert_from_py_signature convert_from_py =
    (convert_from_py_signature)PyCapsule_GetPointer(pyconvert_from_py, NULL);

  PyObject * pysrv = PyObject_CallObject(pyresponse_type, NULL);

  assert(client != NULL);
  assert(convert_from_py != NULL);
  assert(pysrv != NULL);
  void * taken_response = convert_from_py(pysrv);
  rmw_request_id_t * header = (rmw_request_id_t *)PyMem_Malloc(sizeof(rmw_request_id_t));
  header->sequence_number = sequence_number;
  rcl_ret_t ret = rcl_take_response(client, header, taken_response);

  if (ret != RCL_RET_OK && ret != RCL_RET_SERVICE_TAKE_FAILED) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to take from a service: %s", rcl_get_error_string_safe());
    return NULL;
  }

  if (ret != RCL_RET_SERVICE_TAKE_FAILED) {
    PyObject * pyconvert_to_py = PyObject_GetAttrString(pyresponse_type, "_CONVERT_TO_PY");

    typedef PyObject *(* convert_to_py_signature)(void *);
    convert_to_py_signature convert_to_py =
      (convert_to_py_signature)PyCapsule_GetPointer(pyconvert_to_py, NULL);

    PyObject * pytaken_response = convert_to_py(taken_response);

    Py_INCREF(pytaken_response);

    return pytaken_response;
  }
  // if take_response failed, just do nothing
  Py_RETURN_NONE;
}

static PyObject *
rclpy_take_request(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyservice;
  PyObject * pyrequest_type;

  if (!PyArg_ParseTuple(args, "OO", &pyservice, &pyrequest_type)) {
    return NULL;
  }

  rcl_service_t * service =
    (rcl_service_t *)PyCapsule_GetPointer(pyservice, NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pyrequest_type, "__class__");

  PyObject * pyconvert_from_py = PyObject_GetAttrString(pymetaclass, "_CONVERT_FROM_PY");

  typedef void *(* convert_from_py_signature)(PyObject *);
  convert_from_py_signature convert_from_py =
    (convert_from_py_signature)PyCapsule_GetPointer(pyconvert_from_py, NULL);

  PyObject * pysrv = PyObject_CallObject(pyrequest_type, NULL);

  void * taken_request = convert_from_py(pysrv);
  rmw_request_id_t * header = (rmw_request_id_t *)PyMem_Malloc(sizeof(rmw_request_id_t));
  rcl_ret_t ret = rcl_take_request(service, header, taken_request);

  if (ret != RCL_RET_OK && ret != RCL_RET_SERVICE_TAKE_FAILED) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to take from a service: %s", rcl_get_error_string_safe());
    return NULL;
  }

  if (ret != RCL_RET_SERVICE_TAKE_FAILED) {
    PyObject * pyconvert_to_py = PyObject_GetAttrString(pyrequest_type, "_CONVERT_TO_PY");

    typedef PyObject *(* convert_to_py_signature)(void *);
    convert_to_py_signature convert_to_py =
      (convert_to_py_signature)PyCapsule_GetPointer(pyconvert_to_py, NULL);

    PyObject * pytaken_request = convert_to_py(taken_request);

    Py_INCREF(pytaken_request);

    PyObject * pylist = PyList_New(0);
    PyList_Append(pylist, pytaken_request);
    PyList_Append(pylist, PyCapsule_New(header, NULL, NULL));

    return pylist;
  }
  // if take_request failed, just do nothing
  Py_RETURN_NONE;
}

static PyObject *
rclpy_ok(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  bool ok = rcl_ok();
  if (ok) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

/// Request shutdown of the client library
/*
 * \return NULL
 */
static PyObject *
rclpy_shutdown(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcl_ret_t ret = rcl_shutdown();
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to shutdown: %s", rcl_get_error_string_safe());
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Get the list of topics discovered by the provided node
/*
 * \param[in] pynode Capsule pointing to the node
 * \return TopicNamesAndTypes object
 */
static PyObject *
rclpy_get_topic_names_and_types(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "O", &pynode)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, NULL);
  assert(pynode != NULL);
  assert(node != NULL);
  rcl_topic_names_and_types_t topic_names_and_types =
    rcl_get_zero_initialized_topic_names_and_types();
  rcl_ret_t ret = rcl_get_topic_names_and_types(node, &topic_names_and_types);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get_topic_names_and_types: %s", rcl_get_error_string_safe());
    return NULL;
  }
  PyObject * pynames_types_module = PyImport_ImportModule("rclpy.names_and_types");
  PyObject * pytopic_names_types_class = PyObject_GetAttrString(
    pynames_types_module, "TopicNamesAndTypes");
  PyObject * pytopic_names_types = NULL;
  pytopic_names_types = PyObject_CallObject(pytopic_names_types_class, NULL);

  PyObject * pytopic_names = PyList_New(topic_names_and_types.topic_count);
  PyObject * pytype_names = PyList_New(topic_names_and_types.topic_count);
  size_t idx;
  for (idx = 0; idx < topic_names_and_types.topic_count; ++idx) {
    PyList_SetItem(
      pytopic_names, idx, PyUnicode_FromString(topic_names_and_types.topic_names[idx]));
    PyList_SetItem(
      pytype_names, idx, PyUnicode_FromString(topic_names_and_types.type_names[idx]));
  }
  assert(PySequence_Check(pytopic_names));
  assert(pytopic_names != NULL);
  PyObject_SetAttrString(pytopic_names_types, "topic_names", pytopic_names);
  assert(PySequence_Check(pytype_names));
  assert(pytype_names != NULL);
  PyObject_SetAttrString(pytopic_names_types, "type_names", pytype_names);
  PyObject * pytopic_count = NULL;
  pytopic_count = PyLong_FromSize_t(topic_names_and_types.topic_count);
  assert(pytopic_count != NULL);
  PyObject_SetAttrString(
    pytopic_names_types,
    "topic_count",
    pytopic_count);

  ret = rcl_destroy_topic_names_and_types(&topic_names_and_types);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to destroy topic_names_and_types: %s", rcl_get_error_string_safe());
    return NULL;
  }

  assert(pytopic_names_types != NULL);
  return pytopic_names_types;
}

/// Define the public methods of this module
static PyMethodDef rclpy_methods[] = {
  {"rclpy_init", rclpy_init, METH_VARARGS,
   "Initialize RCL."},
  {"rclpy_create_node", rclpy_create_node, METH_VARARGS,
   "Create a Node."},
  {"rclpy_create_publisher", rclpy_create_publisher, METH_VARARGS,
   "Create a Publisher."},
  {"rclpy_create_subscription", rclpy_create_subscription, METH_VARARGS,
   "Create a Subscription."},
  {"rclpy_create_service", rclpy_create_service, METH_VARARGS,
   "Create a Service."},
  {"rclpy_create_client", rclpy_create_client, METH_VARARGS,
   "Create a Client."},

  {"rclpy_destroy_node_entity", rclpy_destroy_node_entity, METH_VARARGS,
   "Destroy a Node entity."},
  {"rclpy_destroy_entity", rclpy_destroy_entity, METH_VARARGS,
   "Destroy a Node."},

  {"rclpy_publish", rclpy_publish, METH_VARARGS,
   "Publish a message."},
  {"rclpy_send_request", rclpy_send_request, METH_VARARGS,
   "Send a request."},
  {"rclpy_send_response", rclpy_send_response, METH_VARARGS,
   "Send a response."},

  {"rclpy_get_zero_initialized_wait_set", rclpy_get_zero_initialized_wait_set, METH_NOARGS,
   "rclpy_get_zero_initialized_wait_set."},

  {"rclpy_wait_set_init", rclpy_wait_set_init, METH_VARARGS,
   "rclpy_wait_set_init."},

  {"rclpy_wait_set_clear_entities", rclpy_wait_set_clear_entities, METH_VARARGS,
   "rclpy_wait_set_clear_entities."},

  {"rclpy_wait_set_add_entity", rclpy_wait_set_add_entity, METH_VARARGS,
   "rclpy_wait_set_add_entity."},

  {"rclpy_get_ready_subscriptions", rclpy_get_ready_subscriptions, METH_VARARGS,
   "List non null subscriptions in waitset."},

  {"rclpy_get_ready_clients", rclpy_get_ready_clients, METH_VARARGS,
   "List non null clients in waitset."},

  {"rclpy_get_ready_services", rclpy_get_ready_services, METH_VARARGS,
   "List non null services in waitset."},

  {"rclpy_wait", rclpy_wait, METH_VARARGS,
   "rclpy_wait."},

  {"rclpy_take", rclpy_take, METH_VARARGS,
   "rclpy_take."},

  {"rclpy_take_request", rclpy_take_request, METH_VARARGS,
   "rclpy_take_request."},

  {"rclpy_take_response", rclpy_take_response, METH_VARARGS,
   "rclpy_take_response."},

  {"rclpy_ok", rclpy_ok, METH_NOARGS,
   "rclpy_ok."},

  {"rclpy_shutdown", rclpy_shutdown, METH_NOARGS,
   "rclpy_shutdown."},

  {"rclpy_get_topic_names_and_types", rclpy_get_topic_names_and_types, METH_VARARGS,
   "Get topic list from graph API."},

  {"rclpy_get_rmw_implementation_identifier", rclpy_get_rmw_implementation_identifier,
   METH_NOARGS, "Retrieve the identifier for the active RMW implementation."},
  {NULL, NULL, 0, NULL}  /* sentinel */
};

/// Define the Python module
static struct PyModuleDef _rclpymodule = {
  PyModuleDef_HEAD_INIT,
  "_rclpy",
  "_rclpy_doc",
  -1,   /* -1 means that the module keeps state in global variables */
  rclpy_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

#define MAKE_FN_NAME(x) PyInit__rclpy ## x
#define FUNCTION_NAME(suffix) MAKE_FN_NAME(suffix)

/// Init function of this module
PyMODINIT_FUNC FUNCTION_NAME(RMW_IMPLEMENTATION_SUFFIX)(void) {
  return PyModule_Create(&_rclpymodule);
}
