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

#include <rmw/rmw.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/node.h>
#include <rosidl_generator_c/message_type_support_struct.h>

#ifndef RMW_IMPLEMENTATION_SUFFIX
#error "RMW_IMPLEMENTATION_SUFFIX is required to be set for _rclpy.c"
#endif

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

static PyObject *
rclpy_create_qos_policy(PyObject * Py_UNUSED(self), PyObject * args)
{
  unsigned PY_LONG_LONG pyqos_history;
  unsigned PY_LONG_LONG pyqos_depth;
  unsigned PY_LONG_LONG pyqos_reliability;
  unsigned PY_LONG_LONG pyqos_durability;

  if (!PyArg_ParseTuple(args, "KKKK", &pyqos_history, &pyqos_depth, &pyqos_reliability, &pyqos_durability)) {
    return NULL;
  }
  
  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)PyMem_Malloc(sizeof(rmw_qos_profile_t));
  qos_profile->history     = pyqos_history    ;
  qos_profile->depth       = pyqos_depth      ;
  qos_profile->reliability = pyqos_reliability;
  qos_profile->durability  = pyqos_durability ;
  PyObject * pyqos_profile = PyCapsule_New(qos_profile, NULL, NULL);
  return pyqos_profile;
}

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

  if (pyqos_profile)
    publisher_ops.qos = *qos_profile;
  rcl_ret_t ret = rcl_publisher_init(publisher, node, ts, topic, &publisher_ops);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create publisher: %s", rcl_get_error_string_safe());
    return NULL;
  }
  PyObject * pypublisher = PyCapsule_New(publisher, NULL, NULL);
  return pypublisher;
}

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

  if (pyqos_profile)
    subscription_ops.qos = *qos_profile;

  rcl_ret_t ret = rcl_subscription_init(subscription, node, ts, topic, &subscription_ops);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create subscriptions: %s", rcl_get_error_string_safe());
    return NULL;
  }
  PyObject * pysubscription = PyCapsule_New(subscription, NULL, NULL);
  return pysubscription;
}

static PyObject *
rclpy_get_rmw_implementation_identifier(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  const char * rmw_implementation_identifier = rmw_get_implementation_identifier();

  PyObject * pyrmw_implementation_identifier = Py_BuildValue(
    "s", rmw_implementation_identifier);

  return pyrmw_implementation_identifier;
}

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

static PyObject *
rclpy_wait(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;

  if (!PyArg_ParseTuple(args, "O", &pywait_set)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, NULL);
  rcl_ret_t ret = rcl_wait(wait_set, RCL_S_TO_NS(1));
  if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to wait on wait set: %s", rcl_get_error_string_safe());
    return NULL;
  }
  Py_RETURN_NONE;
}

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

  {"rclpy_create_qos_policy", rclpy_create_qos_policy, METH_VARARGS,
   "Create a QoS Policy."},

  {"rclpy_get_zero_initialized_wait_set", rclpy_get_zero_initialized_wait_set, METH_NOARGS,
   "rclpy_get_zero_initialized_wait_set."},

  {"rclpy_wait_set_init", rclpy_wait_set_init, METH_VARARGS,
   "rclpy_wait_set_init."},

  {"rclpy_wait_set_clear_subscriptions", rclpy_wait_set_clear_subscriptions, METH_VARARGS,
   "rclpy_wait_set_clear_subscriptions."},

  {"rclpy_wait_set_add_subscription", rclpy_wait_set_add_subscription, METH_VARARGS,
   "rclpy_wait_set_add_subscription."},

  {"rclpy_wait", rclpy_wait, METH_VARARGS,
   "rclpy_wait."},

  {"rclpy_take", rclpy_take, METH_VARARGS,
   "rclpy_take."},

  {"rclpy_ok", rclpy_ok, METH_NOARGS,
   "rclpy_ok."},

  {"rclpy_shutdown", rclpy_shutdown, METH_NOARGS,
   "rclpy_shutdown."},

  {"rclpy_get_rmw_implementation_identifier", rclpy_get_rmw_implementation_identifier,
   METH_NOARGS, "Retrieve the identifier for the active RMW implementation."},
  {NULL, NULL, 0, NULL}  /* sentinel */
};

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

PyMODINIT_FUNC FUNCTION_NAME(RMW_IMPLEMENTATION_SUFFIX)(void) {
  return PyModule_Create(&_rclpymodule);
}
