// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

// Include pybind11 before rclpy_common/handle.h includes Python.h
#include <pybind11/pybind11.h>

#include <rcl/error_handling.h>
#include <rcl/event.h>
#include <rcl/types.h>
#include <rcpputils/scope_exit.hpp>
#include <rmw/incompatible_qos_events_statuses.h>

#include <memory>
#include <utility>

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"
#include "rclpy_common/exceptions.hpp"

#include "qos_events.hpp"

namespace rclpy
{

namespace
{

#define PYMODULE_NAME "rclpy.qos_event"

typedef union qos_event_callback_data {
  // Subscription events
  rmw_requested_deadline_missed_status_t requested_deadline_missed;
  rmw_liveliness_changed_status_t liveliness_changed;
  rmw_message_lost_status_t message_lost;
  rmw_requested_qos_incompatible_event_status_t requested_incompatible_qos;
  // Publisher events
  rmw_offered_deadline_missed_status_t offered_deadline_missed;
  rmw_liveliness_lost_status_t liveliness_lost;
  rmw_offered_qos_incompatible_event_status_t offered_incompatible_qos;
} qos_event_callback_data_t;

typedef py::object qos_event_data_filler_function (const qos_event_callback_data_t *);

template<typename T>
using unique_cstruct_ptr = std::unique_ptr<T, void (*)(T *)>;

unique_cstruct_ptr<rcl_event_t>
create_zero_initialized_event()
{
  unique_cstruct_ptr<rcl_event_t> event(
    static_cast<rcl_event_t *>(PyMem_Malloc(sizeof(rcl_event_t))),
    [](rcl_event_t * event) {
      if (!event) {
        PyErr_Clear();
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level,
          "no rcl_event_t to delete");
      }
      rcl_ret_t ret = rcl_event_fini(event);
      if (RCL_RET_OK != ret) {
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level,
          "failed to fini event: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      PyMem_Free(event);
    });
  if (!event) {
    throw std::bad_alloc();
  }
  *event = rcl_get_zero_initialized_event();
  return event;
}

py::object
_requested_deadline_missed_to_py_object(const qos_event_callback_data_t * data)
{
  py::module qos_events = py::module::import(PYMODULE_NAME);
  py::object pyclass = qos_events.attr("QoSRequestedDeadlineMissedInfo");
  return pyclass(
    data->requested_deadline_missed.total_count,
    data->requested_deadline_missed.total_count_change);
}

py::object
_liveliness_changed_to_py_object(const qos_event_callback_data_t * data)
{
  py::module qos_events = py::module::import(PYMODULE_NAME);
  py::object pyclass = qos_events.attr("QoSLivelinessChangedInfo");
  return pyclass(
    data->liveliness_changed.alive_count,
    data->liveliness_changed.not_alive_count,
    data->liveliness_changed.alive_count_change,
    data->liveliness_changed.not_alive_count_change);
}

py::object
_message_lost_to_py_object(const qos_event_callback_data_t * data)
{
  py::module qos_events = py::module::import(PYMODULE_NAME);
  py::object pyclass = qos_events.attr("QoSMessageLostInfo");
  return pyclass(
    data->message_lost.total_count,
    data->message_lost.total_count_change);
}

py::object
_requested_incompatible_qos_to_py_object(const qos_event_callback_data_t * data)
{
  py::module qos_events = py::module::import(PYMODULE_NAME);
  py::object pyclass = qos_events.attr("QoSRequestedIncompatibleQoSInfo");
  return pyclass(
    data->requested_incompatible_qos.total_count,
    data->requested_incompatible_qos.total_count_change,
    // enum does not specify underlying data type, need intermediate cast
    static_cast<int>(data->requested_incompatible_qos.last_policy_kind));
}

py::object
_offered_deadline_missed_to_py_object(const qos_event_callback_data_t * data)
{
  py::module qos_events = py::module::import(PYMODULE_NAME);
  py::object pyclass = qos_events.attr("QoSOfferedDeadlineMissedInfo");
  return pyclass(
    data->offered_deadline_missed.total_count,
    data->offered_deadline_missed.total_count_change);
}

py::object
_liveliness_lost_to_py_object(const qos_event_callback_data_t * data)
{
  py::module qos_events = py::module::import(PYMODULE_NAME);
  py::object pyclass = qos_events.attr("QoSLivelinessLostInfo");
  return pyclass(
    data->liveliness_lost.total_count,
    data->liveliness_lost.total_count_change);
}

py::object
_offered_incompatible_qos_to_py_object(const qos_event_callback_data_t * data)
{
  py::module qos_events = py::module::import(PYMODULE_NAME);
  py::object pyclass = qos_events.attr("QoSOfferedIncompatibleQoSInfo");
  return pyclass(
    data->offered_incompatible_qos.total_count,
    data->offered_incompatible_qos.total_count_change,
    // enum does not specify underlying data type, need intermediate cast
    static_cast<int>(data->offered_incompatible_qos.last_policy_kind));
}

qos_event_data_filler_function *
qos_event_data_filler_function_for(py::capsule pyparent, py::object pyevent_type)
{
  if (strcmp(pyparent.name(), "rclpy_subscription_t") == 0) {
    switch (pyevent_type.cast<rcl_subscription_event_type_t>()) {
      case RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED:
        return &_requested_deadline_missed_to_py_object;
      case RCL_SUBSCRIPTION_LIVELINESS_CHANGED:
        return &_liveliness_changed_to_py_object;
      case RCL_SUBSCRIPTION_MESSAGE_LOST:
        return &_message_lost_to_py_object;
      case RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS:
        return &_requested_incompatible_qos_to_py_object;
      default:
        throw py::value_error("event type for subscriptions not understood");
        // although this suggests a misalignment between C and Python interfaces
    }
  }
  if (strcmp(pyparent.name(), "rclpy_publisher_t") == 0) {
    switch (pyevent_type.cast<rcl_publisher_event_type_t>()) {
      case RCL_PUBLISHER_OFFERED_DEADLINE_MISSED:
        return &_offered_deadline_missed_to_py_object;
      case RCL_PUBLISHER_LIVELINESS_LOST:
        return &_liveliness_lost_to_py_object;
      case RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS:
        return &_offered_incompatible_qos_to_py_object;
      default:
        throw py::value_error("event type for publishers not understood");
        // although this suggests a misalignment between C and Python interfaces
    }
  }
  throw py::type_error("event parent is neither a publisher nor a subscription");
}

py::capsule
event_wrap_in_capsule(unique_cstruct_ptr<rcl_event_t> event, py::capsule pyparent)
{
  rclpy_handle_destructor_t destructor =
    reinterpret_cast<rclpy_handle_destructor_t>(event.get_deleter());
  PyObject * pyevent_c =
    rclpy_create_handle_capsule(event.get(), "rcl_event_t", destructor);
  if (!pyevent_c) {
    throw py::error_already_set();
  }
  auto pyevent = py::reinterpret_steal<py::capsule>(pyevent_c);
  event.release();  // pyevent now owns rcl_event_t

  rclpy_handle_t * event_handle = static_cast<rclpy_handle_t *>(pyevent);
  rclpy_handle_t * parent_handle = static_cast<rclpy_handle_t *>(pyparent);
  _rclpy_handle_add_dependency(event_handle, parent_handle);
  if (PyErr_Occurred()) {
    throw py::error_already_set();
  }
  return pyevent;
}

py::object
publisher_event_create(rcl_publisher_event_type_t event_type, py::capsule pypublisher)
{
  auto wrapper = static_cast<rclpy_publisher_t *>(
    rclpy_handle_get_pointer_from_capsule(pypublisher.ptr(), "rclpy_publisher_t"));
  if (!wrapper) {
    throw py::error_already_set();
  }

  unique_cstruct_ptr<rcl_event_t> event = create_zero_initialized_event();

  rcl_ret_t ret = rcl_publisher_event_init(
    event.get(), &(wrapper->publisher), event_type);
  if (RCL_RET_OK != ret) {
    if (RCL_RET_BAD_ALLOC == ret) {
      rcl_reset_error();
      throw std::bad_alloc();
    }
    if (RCL_RET_UNSUPPORTED == ret) {
      throw UnsupportedEventTypeError("publisher event is unsupported");
    }
    throw RCLError("failed to create publisher event");
  }

  return event_wrap_in_capsule(std::move(event), pypublisher);
}

py::object
subscription_event_create(rcl_subscription_event_type_t event_type, py::capsule pysubscription)
{
  auto wrapper = static_cast<rclpy_subscription_t *>(
    rclpy_handle_get_pointer_from_capsule(pysubscription.ptr(), "rclpy_subscription_t"));
  if (!wrapper) {
    throw py::error_already_set();
  }

  unique_cstruct_ptr<rcl_event_t> event = create_zero_initialized_event();

  rcl_ret_t ret = rcl_subscription_event_init(
    event.get(), &(wrapper->subscription), event_type);
  if (RCL_RET_OK != ret) {
    if (RCL_RET_BAD_ALLOC == ret) {
      rcl_reset_error();
      throw std::bad_alloc();
    }
    if (RCL_RET_UNSUPPORTED == ret) {
      throw UnsupportedEventTypeError("subscription event is unsupported");
    }
    throw RCLError("failed to create subscription event");
  }

  return event_wrap_in_capsule(std::move(event), pysubscription);
}

}  // namespace

py::object
create_event(py::object pyevent_type, py::capsule pyparent)
{
  if (strcmp(pyparent.name(), "rclpy_subscription_t") == 0) {
    auto event_type = pyevent_type.cast<rcl_subscription_event_type_t>();
    return subscription_event_create(event_type, pyparent);
  }
  if (strcmp(pyparent.name(), "rclpy_publisher_t") == 0) {
    auto event_type = pyevent_type.cast<rcl_publisher_event_type_t>();
    return publisher_event_create(event_type, pyparent);
  }
  throw py::type_error("event parent is neither a publisher nor a subscription");
}

py::object
take_event(py::capsule pyevent, py::capsule pyparent, py::object pyevent_type)
{
  auto event = static_cast<rcl_event_t *>(
    rclpy_handle_get_pointer_from_capsule(pyevent.ptr(), "rcl_event_t"));
  if (!event) {
    throw py::error_already_set();
  }

  qos_event_data_filler_function * event_filler =
    qos_event_data_filler_function_for(pyparent, pyevent_type);

  qos_event_callback_data_t event_data;
  rcl_ret_t ret = rcl_take_event(event, &event_data);
  if (RCL_RET_OK != ret) {
    if (RCL_RET_BAD_ALLOC == ret) {
      rcl_reset_error();
      throw std::bad_alloc();
    }
    if (RCL_RET_EVENT_TAKE_FAILED == ret) {
      return py::none();
    }
    throw RCLError("failed to take event");
  }

  return event_filler(&event_data);
}

}  // namespace rclpy
