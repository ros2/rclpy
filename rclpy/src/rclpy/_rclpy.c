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

#include <c_utilities/types.h>
#include <rcl/error_handling.h>
#include <rcl/graph.h>
#include <rcl/node.h>
#include <rcl/rcl.h>
#include <rmw/rmw.h>
#include <rosidl_generator_c/message_type_support_struct.h>

#include <signal.h>

static rcl_guard_condition_t * g_sigint_gc_handle;

/// Catch signals
static void catch_function(int signo)
{
  (void) signo;
  rcl_ret_t ret = rcl_trigger_guard_condition(g_sigint_gc_handle);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to trigger guard_condition: %s", rcl_get_error_string_safe());
  }
}

/// Create a sigint guard condition
/*
 * \return NULL on failure:
 *         List with 2 elements on success:
 *            first element: a Capsule pointing to the pointer of the created rcl_guard_condition_t * structure
 *            second element: an integer representing the memory address of the created rcl_guard_condition_t
 */
static PyObject *
rclpy_get_sigint_guard_condition(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcl_guard_condition_t * sigint_gc =
    (rcl_guard_condition_t *)PyMem_Malloc(sizeof(rcl_guard_condition_t));
  *sigint_gc = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t sigint_gc_options = rcl_guard_condition_get_default_options();

  rcl_ret_t ret = rcl_guard_condition_init(sigint_gc, sigint_gc_options);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create guard_condition: %s", rcl_get_error_string_safe());
    return NULL;
  }
  g_sigint_gc_handle = sigint_gc;
  PyObject * pylist = PyList_New(0);
  PyList_Append(pylist, PyCapsule_New(sigint_gc, NULL, NULL));
  PyList_Append(pylist, PyLong_FromUnsignedLongLong((uint64_t)&sigint_gc->impl));

  return pylist;
}

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
 * \param[in] namespace string namespace for the node
 * \return NULL on failure
 *         Capsule pointing to the pointer of the created rcl_node_t * structure otherwise
 */
static PyObject *
rclpy_create_node(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * node_name;
  const char * namespace_;

  if (!PyArg_ParseTuple(args, "ss", &node_name, &namespace_)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyMem_Malloc(sizeof(rcl_node_t));
  *node = rcl_get_zero_initialized_node();
  rcl_node_options_t default_options = rcl_node_get_default_options();
  rcl_ret_t ret = rcl_node_init(node, node_name, namespace_, &default_options);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create node: %s", rcl_get_error_string_safe());
    return NULL;
  }
  PyObject * pynode = PyCapsule_New(node, NULL, NULL);
  return pynode;
}

/// Get the name of a node.
/*
 * \param[in] pynode Capsule pointing to the node to get the name from
 * \return NULL on failure
 *         String containing the name of the node otherwise
 */
static PyObject *
rclpy_get_node_name(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "O", &pynode)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, NULL);

  const char * node_name = rcl_node_get_name(node);
  if (!node_name) {
    return NULL;
  }

  return PyUnicode_FromString(node_name);
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

  if (!PyUnicode_Check(pytopic)) {
    PyErr_Format(PyExc_TypeError, "Argument pytopic is not a PyUnicode object");
    return NULL;
  }

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

  assert(convert_from_py != NULL &&
    "unable to retrieve convert_from_py function, type_support mustn't have been imported");
  void * raw_ros_message = convert_from_py(pymsg);

  rcl_ret_t ret = rcl_publish(publisher, raw_ros_message);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to publish: %s", rcl_get_error_string_safe());
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Create a timer
/*
 * \param[in] period_nsec unsigned PyLong object storing the period of the timer in nanoseconds in a 64bits unsigned variable
 * \return NULL on failure:
 *            Raise RuntimeError on initialisation failure
 *            Raise TypeError if argument of invalid type
 *            Raise ValueError if argument cannot be converted to uint64_t
 *         List with 2 elements:
 *            first element: a Capsule pointing to the pointer of the created rcl_timer_t * structure
 *            second element: an integer representing the memory address of the created rcl_timer_t
 */
static PyObject *
rclpy_create_timer(PyObject * Py_UNUSED(self), PyObject * args)
{
  unsigned PY_LONG_LONG period_nsec;

  if (!PyArg_ParseTuple(args, "K", &period_nsec)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *) PyMem_Malloc(sizeof(rcl_timer_t));
  *timer = rcl_get_zero_initialized_timer();

  rcl_ret_t ret = rcl_timer_init(timer, period_nsec, NULL, rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create subscriptions: %s", rcl_get_error_string_safe());
    return NULL;
  }
  PyObject * pytimer = PyCapsule_New(timer, NULL, NULL);
  PyObject * pylist = PyList_New(0);
  PyList_Append(pylist, pytimer);
  PyList_Append(pylist, PyLong_FromUnsignedLongLong((uint64_t)&timer->impl));

  return pylist;
}

/// Returns the period of the timer in nanoseconds
/*
 * \param[in] pytimer Capsule pointing to the timer
 * \return NULL on failure:
 *            Raise RuntimeError on rcl error
 *         PyLong integer in nanoseconds on success
 */
static PyObject *
rclpy_get_timer_period(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, NULL);
  uint64_t timer_period;
  rcl_ret_t ret = rcl_timer_get_period(timer, &timer_period);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get timer period: %s", rcl_get_error_string_safe());
    return NULL;
  }
  return PyLong_FromUnsignedLongLong(timer_period);
}

/// Cancel the timer
/*
 * \param[in] pytimer Capsule pointing to the timer
 * \return NULL on failure:
 *            Raise RuntimeError on rcl error
 *         NULL on success
 */
static PyObject *
rclpy_cancel_timer(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, NULL);
  rcl_ret_t ret = rcl_timer_cancel(timer);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to reset timer: %s", rcl_get_error_string_safe());
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Checks if timer is cancelled
/*
 * \param[in] pytimer Capsule pointing to the timer
 * \return False on failure:
 *            Raise RuntimeError on rcl error
 *         True on success
 */
static PyObject *
rclpy_is_timer_canceled(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, NULL);
  bool is_canceled;
  rcl_ret_t ret = rcl_timer_is_canceled(timer, &is_canceled);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to check timer ready: %s", rcl_get_error_string_safe());
    return NULL;
  }
  if (is_canceled) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

/// Reset the timer
/*
 * \param[in] pytimer Capsule pointing to the timer
 * \return NULL on failure:
 *            Raise RuntimeError on rcl error
 *         NULL on success
 */
static PyObject *
rclpy_reset_timer(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, NULL);
  rcl_ret_t ret = rcl_timer_reset(timer);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to reset timer: %s", rcl_get_error_string_safe());
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Checks if timer reached its timeout
/*
 * \param[in] pytimer Capsule pointing to the timer
 * \return False on failure:
 *            Raise RuntimeError on rcl error
 *         True on success
 */
static PyObject *
rclpy_is_timer_ready(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, NULL);
  bool is_ready;
  rcl_ret_t ret = rcl_timer_is_ready(timer, &is_ready);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to check timer ready: %s", rcl_get_error_string_safe());
    return NULL;
  }
  if (is_ready) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

/// Set the last call time and start counting again
/*
 * \param[in] pytimer Capsule pointing to the timer
 * \return NULL on failure:
 *            Raise RuntimeError on rcl error
 *         NULL on success
 */
static PyObject *
rclpy_call_timer(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, NULL);
  rcl_ret_t ret = rcl_timer_call(timer);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to call timer: %s", rcl_get_error_string_safe());
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Update the timer period
/*
 * The change in period will take effect after the next timer call
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \param[in] period_nsec unsigned PyLongLong containing the new period in nanoseconds
 * \return NULL on failure:
 *            Raise RuntimeError on rcl error
 *         NULL on success
 */
static PyObject *
rclpy_change_timer_period(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  unsigned PY_LONG_LONG period_nsec;
  if (!PyArg_ParseTuple(args, "OK", &pytimer, &period_nsec)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, NULL);
  uint64_t old_period;
  rcl_ret_t ret = rcl_timer_exchange_period(timer, period_nsec, &old_period);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to exchange timer period: %s", rcl_get_error_string_safe());
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Get the time before the timer will be ready
/*
 * the returned time can be negative, this means that the timer is ready and hasn't been called yet
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return NULL on failure:
 *            Raise RuntimeError on rcl error
 *         PyLongLong containing the time until next call in nanoseconds
 */
static PyObject *
rclpy_time_until_next_call(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, NULL);
  int64_t remaining_time;
  rcl_ret_t ret = rcl_timer_get_time_until_next_call(timer, &remaining_time);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get time until next timer call: %s", rcl_get_error_string_safe());
    return NULL;
  }

  return PyLong_FromLongLong(remaining_time);
}

/// Get the time since the timer has been called
/*
 * \param[in] pytimer Capsule pointing to the timer
 * \return NULL on failure:
 *            Raise RuntimeError on rcl error
 *         unsigned PyLongLong containing the time since last call in nanoseconds
 */
static PyObject *
rclpy_time_since_last_call(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, NULL);
  uint64_t elapsed_time;
  rcl_ret_t ret = rcl_timer_get_time_since_last_call(timer, &elapsed_time);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get time since last timer call: %s", rcl_get_error_string_safe());
    return NULL;
  }

  return PyLong_FromUnsignedLongLong(elapsed_time);
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

  if (!PyUnicode_Check(pytopic)) {
    PyErr_Format(PyExc_TypeError, "Argument pytopic is not a PyUnicode object");
    return NULL;
  }

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

/// Create a client
/*
 * This function will create a client and attach it to the topics <pyservice_name>Request and <pyservice_name>Reply
 * This client will use the typesupport defined in the service module
 * provided as pysrv_type to send messages over the wire.
 *
 * \param[in] pynode Capsule pointing to the node to add the client to
 * \param[in] pysrv_type Service module associated with the client
 * \param[in] pyservice_name Python object containing the name used to generate the
 *   <pyservice_name>Request and <pyservice_name>Reply topics names
 * \param[in] pyqos_profile QoSProfile Python object with the profile of this client
 * \return NULL on failure
 *         List with 2 elements:
 *            first element: a Capsule pointing to the pointer of the created rcl_client_t * structure
 *            second element: an integer representing the memory address of the created rcl_client_t
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

  if (!PyUnicode_Check(pyservice_name)) {
    PyErr_Format(PyExc_TypeError, "Argument pyservice_name is not a PyUnicode object");
    return NULL;
  }

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

/// Publish a request message
/*
 * \param[in] pyclient Capsule pointing to the client
 * \param[in] pyrequest request message to send
 * \return sequence_number PyLong object representing the index of the sent request
 */
static PyObject *
rclpy_send_request(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclient;
  PyObject * pyrequest;

  if (!PyArg_ParseTuple(args, "OO", &pyclient, &pyrequest)) {
    return NULL;
  }
  if (!PyCapsule_CheckExact(pyclient)) {
    PyErr_Format(PyExc_TypeError, "Argument pyclient is not a valid PyCapsule");
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

  assert(convert_from_py != NULL &&
    "unable to retrieve convert_from_py function, type_support mustn't have been imported");

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

/// Create a service server
/*
 * This function will create a service server and attach it to the topics <pyservice_name>Request and <pyservice_name>Reply
 * This service will use the typesupport defined in the service module
 * provided as pysrv_type to send messages over the wire.
 *
 * \param[in] pynode Capsule pointing to the node to add the service to
 * \param[in] pysrv_type Service module associated with the service
 * \param[in] pyservice_name Python object containing the name used to generate the
 *   <pyservice_name>Request and <pyservice_name>Reply topics names
 * \param[in] pyqos_profile QoSProfile Python object with the profile of this service
 * \return NULL on failure
 *         List with 2 elements:
 *            first element: a Capsule pointing to the pointer of the created rcl_service_t * structure
 *            second element: an integer representing the memory address of the created rcl_service_t
 */
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

  if (!PyUnicode_Check(pyservice_name)) {
    PyErr_Format(PyExc_TypeError, "Argument pyservice_name is not a PyUnicode object");
    return NULL;
  }

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

/// Publish a response message
/*
 * \param[in] pyservice Capsule pointing to the client
 * \param[in] pyresponse reply message to send
 * \param[in] pyheader Capsule pointing to the rmw_request_id_t header of the request we respond to
 * \return NULL
 */
static PyObject *
rclpy_send_response(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyservice;
  PyObject * pyresponse;
  PyObject * pyheader;

  if (!PyArg_ParseTuple(args, "OOO", &pyservice, &pyresponse, &pyheader)) {
    return NULL;
  }
  if (!PyCapsule_CheckExact(pyservice)) {
    PyErr_Format(PyExc_TypeError, "Argument pyservice is not a valid PyCapsule");
    return NULL;
  }
  rcl_service_t * service = (rcl_service_t *)PyCapsule_GetPointer(pyservice, NULL);
  assert(service != NULL);

  if (!PyCapsule_CheckExact(pyheader)) {
    PyErr_Format(PyExc_TypeError, "Argument pyheader is not a valid PyCapsule");
    return NULL;
  }
  rmw_request_id_t * header = (rmw_request_id_t *)PyCapsule_GetPointer(pyheader, NULL);
  assert(header != NULL);
  PyObject * pyresponse_type = PyObject_GetAttrString(pyresponse, "__class__");
  assert(pyresponse_type != NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pyresponse_type, "__class__");
  assert(pymetaclass != NULL);

  PyObject * pyconvert_from_py = PyObject_GetAttrString(pymetaclass, "_CONVERT_FROM_PY");

  assert(pyconvert_from_py != NULL);
  typedef void * (* convert_from_py_signature)(PyObject *);
  convert_from_py_signature convert_from_py =
    (convert_from_py_signature)PyCapsule_GetPointer(pyconvert_from_py, NULL);

  assert(convert_from_py != NULL &&
    "unable to retrieve convert_from_py function, type_support mustn't have been imported");
  void * raw_ros_response = convert_from_py(pyresponse);

  rcl_ret_t ret = rcl_send_response(service, header, raw_ros_response);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to send request: %s", rcl_get_error_string_safe());
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Destroy an entity attached to a node
/*
 * \param[in] entity_type string defining the entity ["subscription", "publisher", "client", "service"]
 * \param[in] pyentity Capsule pointing to the entity to destroy
 * \param[in] pynode Capsule pointing to the node the entity belongs to
 * \return True on success, False on failure
 */
static PyObject *
rclpy_destroy_node_entity(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * entity_type;
  PyObject * pyentity;
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "zOO", &entity_type, &pyentity, &pynode)) {
    Py_RETURN_FALSE;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, NULL);
  rcl_ret_t ret;
  if (0 == strcmp(entity_type, "subscription")) {
    rcl_subscription_t * subscription = (rcl_subscription_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_subscription_fini(subscription, node);
  } else if (0 == strcmp(entity_type, "publisher")) {
    rcl_publisher_t * publisher = (rcl_publisher_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_publisher_fini(publisher, node);
  } else if (0 == strcmp(entity_type, "client")) {
    rcl_client_t * client = (rcl_client_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_client_fini(client, node);
  } else if (0 == strcmp(entity_type, "service")) {
    rcl_service_t * service = (rcl_service_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_service_fini(service, node);
  } else {
    ret = RCL_RET_ERROR;  // to avoid a linter warning
    PyErr_Format(PyExc_RuntimeError,
      "%s is not a known entity", entity_type);
    Py_RETURN_FALSE;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to fini '%s': %s", entity_type, rcl_get_error_string_safe());
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

/// Destroy an rcl entity
/*
 * \param[in] entity_type string defining the entity ["node"]
 * \param[in] pyentity Capsule pointing to the entity to destroy
 * \return True on success, False on failure
 */
static PyObject *
rclpy_destroy_entity(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * entity_type;
  PyObject * pyentity;

  if (!PyArg_ParseTuple(args, "zO", &entity_type, &pyentity)) {
    Py_RETURN_FALSE;
  }

  rcl_ret_t ret;
  if (0 == strcmp(entity_type, "node")) {
    rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_node_fini(node);
  } else if (0 == strcmp(entity_type, "timer")) {
    rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_timer_fini(timer);
  } else {
    ret = RCL_RET_ERROR;  // to avoid a linter warning
    PyErr_Format(PyExc_RuntimeError,
      "%s is not a known entity", entity_type);
    Py_RETURN_FALSE;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to fini '%s': %s", entity_type, rcl_get_error_string_safe());
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
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
 * \param[in] number_of_timers int
 * \param[in] number_of_clients int
 * \param[in] number_of_services int
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

/// Clear all the pointers of a given wait_set field
/*
 * \param[in] entity_type string defining the entity ["subscription, client, service"]
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
  } else if (0 == strcmp(entity_type, "client")) {
    ret = rcl_wait_set_clear_clients(wait_set);
  } else if (0 == strcmp(entity_type, "service")) {
    ret = rcl_wait_set_clear_services(wait_set);
  } else if (0 == strcmp(entity_type, "timer")) {
    ret = rcl_wait_set_clear_timers(wait_set);
  } else if (0 == strcmp(entity_type, "guard_condition")) {
    ret = rcl_wait_set_clear_guard_conditions(wait_set);
  } else {
    ret = RCL_RET_ERROR;  // to avoid a linter warning
    PyErr_Format(PyExc_RuntimeError,
      "%s is not a known entity", entity_type);
    Py_RETURN_FALSE;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to clear '%s' from wait set: %s", entity_type, rcl_get_error_string_safe());
    Py_RETURN_FALSE;
  }
  Py_RETURN_TRUE;
}

/// Add an entity to the waitset structure
/*
 * \param[in] entity_type string defining the entity ["subscription, client, service"]
 * \param[in] pywait_set Capsule pointing to the waitset structure
 * \param[in] pyentity Capsule pointing to the entity to add
 * \return NULL
 */
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
  } else if (0 == strcmp(entity_type, "timer")) {
    rcl_timer_t * timer =
      (rcl_timer_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_wait_set_add_timer(wait_set, timer);
  } else if (0 == strcmp(entity_type, "guard_condition")) {
    rcl_guard_condition_t * guard_condition =
      (rcl_guard_condition_t *)PyCapsule_GetPointer(pyentity, NULL);
    ret = rcl_wait_set_add_guard_condition(wait_set, guard_condition);
  } else {
    ret = RCL_RET_ERROR;  // to avoid a linter warning
    PyErr_Format(PyExc_RuntimeError,
      "%s is not a known entity", entity_type);
    Py_RETURN_FALSE;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to add '%s' to wait set: %s", entity_type, rcl_get_error_string_safe());
    return NULL;
  }
  Py_RETURN_NONE;
}

#define GET_LIST_READY_ENTITIES(ENTITY_TYPE) \
  size_t idx; \
  size_t idx_max; \
  idx_max = wait_set->size_of_ ## ENTITY_TYPE ## s; \
  const rcl_ ## ENTITY_TYPE ## _t ** struct_ptr = wait_set->ENTITY_TYPE ## s; \
  for (idx = 0; idx < idx_max; idx ++) { \
    if (struct_ptr[idx]) { \
      PyList_Append( \
        entity_ready_list, \
        PyLong_FromUnsignedLongLong((uint64_t) & struct_ptr[idx]->impl)); \
    } \
  } \
  return entity_ready_list;
/// Get list of non-null entities in waitset
/*
 * \param[in] entity_type string defining the entity ["subscription, client, service"]
 * \param[in] pywait_set Capsule pointing to the waitset structure
 * \return List of wait_set entities pointers ready for take
 */
static PyObject *
rclpy_get_ready_entities(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * entity_type;
  PyObject * pywait_set;
  if (!PyArg_ParseTuple(args, "zO", &entity_type, &pywait_set)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, NULL);
  if (!wait_set) {
    PyErr_Format(PyExc_RuntimeError, "waiset is null");
    return NULL;
  }

  PyObject * entity_ready_list = PyList_New(0);
  if (0 == strcmp(entity_type, "subscription")) {
    GET_LIST_READY_ENTITIES(subscription)
  } else if (0 == strcmp(entity_type, "client")) {
    GET_LIST_READY_ENTITIES(client)
  } else if (0 == strcmp(entity_type, "service")) {
    GET_LIST_READY_ENTITIES(service)
  } else if (0 == strcmp(entity_type, "timer")) {
    GET_LIST_READY_ENTITIES(timer)
  } else if (0 == strcmp(entity_type, "guard_condition")) {
    GET_LIST_READY_ENTITIES(guard_condition)
  } else {
    PyErr_Format(PyExc_RuntimeError,
      "%s is not a known entity", entity_type);
    return NULL;
  }

  return entity_ready_list;
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

  signal(SIGINT, catch_function);
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
 * \param[in] pysubscription Capsule pointing to the subscription to process the message
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
  if (!PyCapsule_CheckExact(pysubscription)) {
    PyErr_Format(PyExc_TypeError, "Argument pysubscription is not a valid PyCapsule");
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

  assert(convert_from_py != NULL);
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

/// Take a request from a given service
/*
 * \param[in] pyservice Capsule pointing to the service to process the request
 * \param[in] pyrequest_type Instance of the message type to take
 * \return List with 2 elements:
 *            first element: a Python request message with all fields populated with received request
 *            second element: a Capsule pointing to the header (rmw_request_id) of the processed request
 */
static PyObject *
rclpy_take_request(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyservice;
  PyObject * pyrequest_type;

  if (!PyArg_ParseTuple(args, "OO", &pyservice, &pyrequest_type)) {
    return NULL;
  }
  if (!PyCapsule_CheckExact(pyservice)) {
    PyErr_Format(PyExc_TypeError, "Argument pyservice is not a valid PyCapsule");
    return NULL;
  }

  rcl_service_t * service =
    (rcl_service_t *)PyCapsule_GetPointer(pyservice, NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pyrequest_type, "__class__");

  PyObject * pyconvert_from_py = PyObject_GetAttrString(pymetaclass, "_CONVERT_FROM_PY");

  typedef void *(* convert_from_py_signature)(PyObject *);
  convert_from_py_signature convert_from_py =
    (convert_from_py_signature)PyCapsule_GetPointer(pyconvert_from_py, NULL);
  assert(convert_from_py != NULL &&
    "unable to retrieve convert_from_py function, type_support mustn't have been imported");

  PyObject * pysrv = PyObject_CallObject(pyrequest_type, NULL);

  void * taken_request = convert_from_py(pysrv);
  rmw_request_id_t * header = (rmw_request_id_t *)PyMem_Malloc(sizeof(rmw_request_id_t));
  rcl_ret_t ret = rcl_take_request(service, header, taken_request);

  if (ret != RCL_RET_OK && ret != RCL_RET_SERVICE_TAKE_FAILED) {
    PyErr_Format(PyExc_RuntimeError,
      "Service failed to take request: %s", rcl_get_error_string_safe());
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

/// Take a response from a given client
/*
 * \param[in] pyclient Capsule pointing to the client to process the response
 * \param[in] pyresponse_type Instance of the message type to take
 * \return Python response message with all fields populated with received response
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
  if (!PyCapsule_CheckExact(pyclient)) {
    PyErr_Format(PyExc_TypeError, "Argument pyclient is not a valid PyCapsule");
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
      "Client failed to take response: %s", rcl_get_error_string_safe());
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

/// Get the list of nodes discovered by the provided node
/*
 * \param[in] pynode Capsule pointing to the node
 * \return Python list of strings
 */
static PyObject *
rclpy_get_node_names(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "O", &pynode)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, NULL);
  utilities_string_array_t node_names =
    utilities_get_zero_initialized_string_array();
  rcl_ret_t ret = rcl_get_node_names(node, &node_names);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get_node_names: %s", rcl_get_error_string_safe());
    return NULL;
  }

  PyObject * pynode_names = PyList_New(node_names.size);
  size_t idx;
  for (idx = 0; idx < node_names.size; ++idx) {
    PyList_SetItem(
      pynode_names, idx, PyUnicode_FromString(node_names.data[idx]));
  }

  ret = utilities_string_array_fini(&node_names);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to destroy node_names: %s", rcl_get_error_string_safe());
    return NULL;
  }

  return pynode_names;
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
  PyObject * pytopic_names_types = PyObject_CallObject(pytopic_names_types_class, NULL);
  if (!pytopic_names_types) {
    PyErr_Format(PyExc_RuntimeError,
      "Couldn't create instance of TopicNamesAndTypes");
    return NULL;
  }

  PyObject * pytopic_names = PyList_New(topic_names_and_types.topic_count);
  PyObject * pytype_names = PyList_New(topic_names_and_types.topic_count);
  size_t idx;
  for (idx = 0; idx < topic_names_and_types.topic_count; ++idx) {
    PyList_SetItem(
      pytopic_names, idx, PyUnicode_FromString(topic_names_and_types.topic_names[idx]));
    PyList_SetItem(
      pytype_names, idx, PyUnicode_FromString(topic_names_and_types.type_names[idx]));
  }
  PyObject_SetAttrString(pytopic_names_types, "topic_names", pytopic_names);
  PyObject_SetAttrString(pytopic_names_types, "type_names", pytype_names);
  PyObject * pytopic_count = NULL;
  pytopic_count = PyLong_FromSize_t(topic_names_and_types.topic_count);
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

  return pytopic_names_types;
}

/// Define the public methods of this module
static PyMethodDef rclpy_methods[] = {
  {"rclpy_init", rclpy_init, METH_VARARGS,
   "Initialize RCL."},
  {"rclpy_create_node", rclpy_create_node, METH_VARARGS,
   "Create a Node."},
  {"rclpy_get_node_name", rclpy_get_node_name, METH_VARARGS,
   "Get the name of a node."},
  {"rclpy_create_publisher", rclpy_create_publisher, METH_VARARGS,
   "Create a Publisher."},
  {"rclpy_create_subscription", rclpy_create_subscription, METH_VARARGS,
   "Create a Subscription."},
  {"rclpy_create_service", rclpy_create_service, METH_VARARGS,
   "Create a Service."},
  {"rclpy_create_client", rclpy_create_client, METH_VARARGS,
   "Create a Client."},
  {"rclpy_create_timer", rclpy_create_timer, METH_VARARGS,
   "Create a Timer."},

  {"rclpy_get_sigint_guard_condition", rclpy_get_sigint_guard_condition, METH_NOARGS,
   "Create a guard_condition triggered when sigint is received."},

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

  {"rclpy_get_ready_entities", rclpy_get_ready_entities, METH_VARARGS,
   "List non null subscriptions in waitset."},

  {"rclpy_reset_timer", rclpy_reset_timer, METH_VARARGS,
   "Reset a timer."},

  {"rclpy_call_timer", rclpy_call_timer, METH_VARARGS,
   "Call a timer and starts counting again."},

  {"rclpy_change_timer_period", rclpy_change_timer_period, METH_VARARGS,
   "Set the period of a timer."},

  {"rclpy_is_timer_ready", rclpy_is_timer_ready, METH_VARARGS,
   "Check if a timer as reached timeout."},

  {"rclpy_cancel_timer", rclpy_cancel_timer, METH_VARARGS,
   "Cancel a timer."},

  {"rclpy_is_timer_canceled", rclpy_is_timer_canceled, METH_VARARGS,
   "Check if a timer is canceled."},

  {"rclpy_time_until_next_call", rclpy_time_until_next_call, METH_VARARGS,
   "Get the remaining time before timer is ready."},

  {"rclpy_time_since_last_call", rclpy_time_since_last_call, METH_VARARGS,
   "Get the elapsed time since last timer call."},

  {"rclpy_get_timer_period", rclpy_get_timer_period, METH_VARARGS,
   "Get the period of a timer."},

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

  {"rclpy_get_node_names", rclpy_get_node_names, METH_VARARGS,
   "Get node names list from graph API."},
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

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy(void)
{
  return PyModule_Create(&_rclpymodule);
}
