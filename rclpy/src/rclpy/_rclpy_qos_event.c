// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include "rcl/event.h"
#include "rclpy_common/handle.h"
#include "rmw/incompatible_qos_events_statuses.h"

typedef union _qos_event_callback_data {
  // Subscription events
  rmw_requested_deadline_missed_status_t requested_deadline_missed;
  rmw_liveliness_changed_status_t liveliness_changed;
  rmw_message_lost_status_t message_lost;
  rmw_requested_qos_incompatible_event_status_t requested_incompatible_qos;
  // Publisher events
  rmw_offered_deadline_missed_status_t offered_deadline_missed;
  rmw_liveliness_lost_status_t liveliness_lost;
  rmw_offered_qos_incompatible_event_status_t offered_incompatible_qos;
} _qos_event_callback_data_t;

typedef PyObject * (* _qos_event_data_filler_function)(_qos_event_callback_data_t *);

static
bool
_check_rcl_return(rclpy_module_state_t * module_state, rcl_ret_t ret, const char * error_msg)
{
  if (!module_state) {
    PyErr_Format(PyExc_RuntimeError, "_check_rcl_return got NULL module state");
    return false;
  }
  if (RCL_RET_OK == ret) {
    return true;
  }

  PyObject * exception =
    (RCL_RET_UNSUPPORTED == ret) ? module_state->UnsupportedEventTypeError : module_state->RCLError;
  PyErr_Format(exception, "%s: %s", error_msg, rcl_get_error_string().str);
  rcl_reset_error();

  return false;
}

static
void
_destroy_event_capsule(void * p)
{
  rcl_event_t * event = p;
  if (!event) {
    PyErr_Clear();
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_destroy_event_capsule failed to get pointer");
  }
  rcl_ret_t ret = rcl_event_fini(event);
  if (RCL_RET_OK != ret) {
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini event: %s", rcl_get_error_string().str);
  }
  PyMem_Free(event);
}

static
bool
_is_pycapsule_rcl_subscription(PyObject * pycapsule)
{
  return PyCapsule_IsValid(pycapsule, "rclpy_subscription_t");
}

static
bool
_is_pycapsule_rcl_publisher(PyObject * pycapsule)
{
  return PyCapsule_IsValid(pycapsule, "rclpy_publisher_t");
}

static
rcl_event_t *
_pycapsule_to_rcl_event(PyObject * pycapsule)
{
  return rclpy_handle_get_pointer_from_capsule(pycapsule, "rcl_event_t");
}

static
rcl_event_t *
_new_zero_initialized_rcl_event()
{
  rcl_event_t * event = (rcl_event_t *)PyMem_Malloc(sizeof(rcl_event_t));
  if (!event) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for event");
    return NULL;
  }
  *event = rcl_get_zero_initialized_event();
  return event;
}

/// Create a Python object of the given type from the qos_event module.
/**
  * \param[in] class_name The name of the rclpy.qos_event class to construct.
  * \param[in] args Tuple-like arguments to the constructor of the type.
  *   NOTE this function steals a reference to `args` instead of just borrowing it.
  */
static
PyObject * _create_py_qos_event(const char * class_name, PyObject * args)
{
  PyObject * pyqos_event_module = NULL;
  PyObject * pyqos_event_class = NULL;
  PyObject * pyqos_event = NULL;

  pyqos_event_module = PyImport_ImportModule("rclpy.qos_event");
  if (!pyqos_event_module) {
    goto cleanup;
  }

  pyqos_event_class = PyObject_GetAttrString(pyqos_event_module, class_name);
  if (!pyqos_event_class) {
    goto cleanup;
  }

  pyqos_event = PyObject_CallObject(pyqos_event_class, args);

cleanup:
  Py_XDECREF(pyqos_event_module);
  Py_XDECREF(pyqos_event_class);
  Py_XDECREF(args);

  return pyqos_event;
}

static
PyObject *
_requested_deadline_missed_to_py_object(_qos_event_callback_data_t * data)
{
  rmw_requested_deadline_missed_status_t * actual_data = &data->requested_deadline_missed;
  PyObject * args = Py_BuildValue(
    "ii",
    actual_data->total_count,
    actual_data->total_count_change);
  if (!args) {
    return NULL;
  }
  return _create_py_qos_event("QoSRequestedDeadlineMissedInfo", args);
}

static
PyObject *
_liveliness_changed_to_py_object(_qos_event_callback_data_t * data)
{
  rmw_liveliness_changed_status_t * actual_data = &data->liveliness_changed;
  PyObject * args = Py_BuildValue(
    "iiii",
    actual_data->alive_count,
    actual_data->not_alive_count,
    actual_data->alive_count_change,
    actual_data->not_alive_count_change);
  if (!args) {
    return NULL;
  }
  return _create_py_qos_event("QoSLivelinessChangedInfo", args);
}

static
PyObject *
_message_lost_to_py_object(_qos_event_callback_data_t * data)
{
  rmw_message_lost_status_t * actual_data = &data->message_lost;
  PyObject * args = Py_BuildValue(
    "ii",
    actual_data->total_count,
    actual_data->total_count_change);
  if (!args) {
    return NULL;
  }
  return _create_py_qos_event("QoSMessageLostInfo", args);
}

static
PyObject *
_requested_incompatible_qos_to_py_object(_qos_event_callback_data_t * data)
{
  rmw_requested_qos_incompatible_event_status_t * actual_data = &data->requested_incompatible_qos;
  PyObject * args = Py_BuildValue(
    "iii",
    actual_data->total_count,
    actual_data->total_count_change,
    actual_data->last_policy_kind);
  if (!args) {
    return NULL;
  }
  return _create_py_qos_event("QoSRequestedIncompatibleQoSInfo", args);
}

static
PyObject *
_offered_deadline_missed_to_py_object(_qos_event_callback_data_t * data)
{
  rmw_offered_deadline_missed_status_t * actual_data = &data->offered_deadline_missed;
  PyObject * args = Py_BuildValue(
    "ii",
    actual_data->total_count,
    actual_data->total_count_change);
  if (!args) {
    return NULL;
  }
  return _create_py_qos_event("QoSOfferedDeadlineMissedInfo", args);
}

static
PyObject *
_liveliness_lost_to_py_object(_qos_event_callback_data_t * data)
{
  rmw_liveliness_lost_status_t * actual_data = &data->liveliness_lost;
  PyObject * args = Py_BuildValue(
    "ii",
    actual_data->total_count,
    actual_data->total_count_change);
  if (!args) {
    return NULL;
  }
  return _create_py_qos_event("QoSLivelinessLostInfo", args);
}

static
PyObject *
_offered_incompatible_qos_to_py_object(_qos_event_callback_data_t * data)
{
  rmw_offered_qos_incompatible_event_status_t * actual_data = &data->offered_incompatible_qos;
  PyObject * args = Py_BuildValue(
    "iii",
    actual_data->total_count,
    actual_data->total_count_change,
    actual_data->last_policy_kind);
  if (!args) {
    return NULL;
  }
  return _create_py_qos_event("QoSOfferedIncompatibleQoSInfo", args);
}

static
_qos_event_data_filler_function
_get_qos_event_data_filler_function_for(PyObject * pyparent, unsigned PY_LONG_LONG event_type)
{
  if (_is_pycapsule_rcl_subscription(pyparent)) {
    switch (event_type) {
      case RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED:
        return &_requested_deadline_missed_to_py_object;
      case RCL_SUBSCRIPTION_LIVELINESS_CHANGED:
        return &_liveliness_changed_to_py_object;
      case RCL_SUBSCRIPTION_MESSAGE_LOST:
        return &_message_lost_to_py_object;
      case RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS:
        return &_requested_incompatible_qos_to_py_object;
      default:
        PyErr_Format(
          PyExc_ValueError,
          "Event type %llu for Subscriptions not understood by rclpy.", event_type);
    }
  } else if (_is_pycapsule_rcl_publisher(pyparent)) {
    switch (event_type) {
      case RCL_PUBLISHER_OFFERED_DEADLINE_MISSED:
        return &_offered_deadline_missed_to_py_object;
      case RCL_PUBLISHER_LIVELINESS_LOST:
        return &_liveliness_lost_to_py_object;
      case RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS:
        return &_offered_incompatible_qos_to_py_object;
      default:
        PyErr_Format(
          PyExc_ValueError,
          "Event type %llu for Publishers not understood by rclpy.", event_type);
    }
  } else {
    PyErr_Format(
      PyExc_TypeError,
      "Parent handle was not a valid Publisher or Subscription.");
  }
  return NULL;
}

/// Create an event object for QoS event handling.
/**
  * This function will create an event handle for the given Subscription or Publisher parent.
  *
  * Raises MemoryError if the event can't be allocated.
  * Raises RuntimeError on initialization failure.
  * Raises TypeError if the capsules are not the correct types.
  *
  * \param[in] event_type Enum value of
  *   rcl_publisher_event_type_t or rcl_subscription_event_type_t, chosen by the type of pyparent.
  * \param[in] pyparent Capsule containing the parent Publisher or Subscription.
  * \return capsule containing rcl_event_t.
  * \return NULL on failure.
  */
static PyObject *
rclpy_create_event(PyObject * module, PyObject * args)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }
  unsigned PY_LONG_LONG event_type;
  PyObject * pyparent = NULL;

  rcl_ret_t ret;
  rcl_subscription_t * subscription = NULL;
  rcl_publisher_t * publisher = NULL;
  rcl_event_t * event = NULL;

  if (!PyArg_ParseTuple(args, "KO", &event_type, &pyparent)) {
    return NULL;
  }

  rclpy_handle_t * parent_handle = PyCapsule_GetPointer(pyparent, PyCapsule_GetName(pyparent));
  if (_is_pycapsule_rcl_subscription(pyparent)) {
    rclpy_subscription_t * py_subscription = _rclpy_handle_get_pointer(parent_handle);
    subscription = py_subscription ? &py_subscription->subscription : NULL;
  } else if (_is_pycapsule_rcl_publisher(pyparent)) {
    rclpy_publisher_t * py_publisher = _rclpy_handle_get_pointer(parent_handle);
    publisher = py_publisher ? &py_publisher->publisher : NULL;
  } else {
    PyErr_Format(PyExc_TypeError, "Event parent was not a valid Publisher or Subscription.");
    return NULL;
  }

  event = _new_zero_initialized_rcl_event();
  if (!event) {
    return NULL;
  }

  if (subscription) {
    ret = rcl_subscription_event_init(event, subscription, event_type);
  } else {
    ret = rcl_publisher_event_init(event, publisher, event_type);
  }
  if (!_check_rcl_return(module_state, ret, "Failed to initialize event")) {
    PyMem_Free(event);
    return NULL;
  }

  rclpy_handle_t * event_handle = _rclpy_create_handle(event, _destroy_event_capsule);
  if (!event_handle) {
    ret = rcl_event_fini(event);
    PyMem_Free(event);
    _check_rcl_return(module_state, ret, "Failed to fini 'rcl_event_t'");
    return NULL;
  }
  _rclpy_handle_add_dependency(event_handle, parent_handle);
  if (PyErr_Occurred()) {
    _rclpy_handle_dec_ref(event_handle);
    return NULL;
  }
  PyObject * event_capsule = _rclpy_create_handle_capsule(event_handle, "rcl_event_t");
  if (!event_capsule) {
    _rclpy_handle_dec_ref(event_handle);
    return NULL;
  }
  return event_capsule;
}

/// Get a pending QoS event's data.
/**
  * After having determined that a middleware event is ready, get the callback payload.
  *
  * Raises RuntimeError on failure to take the event from the middleware.
  * Raises TypeError if the capsules are not the correct types.
  * Raises ValueError on unknown event_type argument.
  *
  * \param[in] pyevent Event handle from rclpy_create_event.
  * \param[in] event_type Enum value of
  *   rcl_publisher_event_type_t or rcl_subscription_event_type_t, chosen by the type of pyparent.
  * \param[in] pyparent Capsule containing the parent Publisher or Subscription.
  * \return Python object from rclpy.qos_event containing callback data.
  * \return NULL on failure.
  */
static PyObject *
rclpy_take_event(PyObject * Py_UNUSED(self), PyObject * args)
{
  // Arguments
  PyObject * pyevent = NULL;
  PyObject * pyparent = NULL;
  unsigned PY_LONG_LONG event_type;

  // Type conversion
  rcl_ret_t ret;
  rcl_event_t * event = NULL;
  _qos_event_callback_data_t event_data;
  _qos_event_data_filler_function event_filler = NULL;

  if (!PyArg_ParseTuple(args, "OOK", &pyevent, &pyparent, &event_type)) {
    return NULL;
  }

  event = _pycapsule_to_rcl_event(pyevent);
  if (!event) {
    return NULL;
  }

  event_filler = _get_qos_event_data_filler_function_for(pyparent, event_type);
  if (!event_filler) {
    return NULL;
  }

  ret = rcl_take_event(event, &event_data);
  if (RCL_RET_UNSUPPORTED == ret) {
    PyErr_Format(
      PyExc_NotImplementedError,
      "Take event is not implemented in the current RMW implementation: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
  } else if (RCL_RET_OK != ret) {
    PyErr_Format(PyExc_RuntimeError, "Failed to take event: %s", rcl_get_error_string().str);
    rcl_reset_error();
  } else {
    return event_filler(&event_data);
  }
  return NULL;
}
