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
  node->impl = NULL;
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
  publisher->impl = NULL;
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
  subscription->impl = NULL;
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
  wait_set->subscriptions = NULL;
  wait_set->size_of_subscriptions = 0;
  wait_set->guard_conditions = NULL;
  wait_set->size_of_guard_conditions = 0;
  wait_set->timers = NULL;
  wait_set->size_of_timers = 0;
  wait_set->clients = NULL;
  wait_set->size_of_clients = 0;
  wait_set->services = NULL;
  wait_set->size_of_services = 0;
  wait_set->impl = NULL;
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
  const unsigned PY_LONG_LONG number_of_clients = 0;
  const unsigned PY_LONG_LONG number_of_services = 0;

  if (!PyArg_ParseTuple(
      args, "OKKK", &pywait_set, &number_of_subscriptions,
      &number_of_guard_conditions, &number_of_timers))
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
rclpy_wait_set_clear_subscriptions(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;

  if (!PyArg_ParseTuple(args, "O", &pywait_set)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, NULL);
  rcl_ret_t ret = rcl_wait_set_clear_subscriptions(wait_set);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to clear subscriptions from wait set: %s", rcl_get_error_string_safe());
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Add a subscription to the waitset
/*
 * \param[in] pywait_set Capsule pointing to the waitset structure
 * \param[in] pysubscription Capsule pointing to the subscription to add
 * \return NULL
 */
static PyObject *
rclpy_wait_set_add_subscription(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;
  PyObject * pysubscription;

  if (!PyArg_ParseTuple(args, "OO", &pywait_set, &pysubscription)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, NULL);
  rcl_subscription_t * subscription =
    (rcl_subscription_t *)PyCapsule_GetPointer(pysubscription, NULL);
  rcl_ret_t ret = rcl_wait_set_add_subscription(wait_set, subscription);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to add subscription to wait set: %s", rcl_get_error_string_safe());
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Get list of non-null subscriptions in waitset
/*
 * \param[in] pywait_set Capsule pointing to the waitset structure
 * \return List of subscription pointers ready for take
 */
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
  {"rclpy_publish", rclpy_publish, METH_VARARGS,
   "Publish a message."},
  {"rclpy_create_subscription", rclpy_create_subscription, METH_VARARGS,
   "Create a Subscription."},

  {"rclpy_get_zero_initialized_wait_set", rclpy_get_zero_initialized_wait_set, METH_NOARGS,
   "rclpy_get_zero_initialized_wait_set."},

  {"rclpy_wait_set_init", rclpy_wait_set_init, METH_VARARGS,
   "rclpy_wait_set_init."},

  {"rclpy_wait_set_clear_subscriptions", rclpy_wait_set_clear_subscriptions, METH_VARARGS,
   "rclpy_wait_set_clear_subscriptions."},

  {"rclpy_wait_set_add_subscription", rclpy_wait_set_add_subscription, METH_VARARGS,
   "rclpy_wait_set_add_subscription."},

  {"rclpy_get_ready_subscriptions", rclpy_get_ready_subscriptions, METH_VARARGS,
   "List non null subscriptions in waitset."},

  {"rclpy_wait", rclpy_wait, METH_VARARGS,
   "rclpy_wait."},

  {"rclpy_take", rclpy_take, METH_VARARGS,
   "rclpy_take."},

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
