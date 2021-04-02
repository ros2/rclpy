// Copyright 2021 Open Source Robotics Foundation, Inc.
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
#include <rcl/rcl.h>
#include <rcl/types.h>

#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"
#include "wait_set.hpp"

namespace rclpy
{
/// Destructor for a wait set
void
_rclpy_destroy_wait_set(PyObject * pycapsule)
{
  auto wait_set = static_cast<rcl_wait_set_t *>(PyCapsule_GetPointer(pycapsule, "rcl_wait_set_t"));

  rcl_ret_t ret = rcl_wait_set_fini(wait_set);
  if (ret != RCL_RET_OK) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level,
      "Failed to fini wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  PyMem_Free(wait_set);
}

py::capsule
get_zero_initialized_wait_set()
{
  auto deleter = [](rcl_wait_set_t * ptr) {PyMem_FREE(ptr);};
  auto wait_set = std::unique_ptr<rcl_wait_set_t, decltype(deleter)>(
    static_cast<rcl_wait_set_t *>(PyMem_Malloc(sizeof(rcl_wait_set_t))),
    deleter);
  if (!wait_set) {
    throw std::bad_alloc();
  }

  *wait_set = rcl_get_zero_initialized_wait_set();
  return py::capsule(wait_set.release(), "rcl_wait_set_t", _rclpy_destroy_wait_set);
}

void
wait_set_init(
  py::capsule pywait_set,
  size_t number_of_subscriptions,
  size_t number_of_guard_conditions,
  size_t number_of_timers,
  size_t number_of_clients,
  size_t number_of_services,
  size_t number_of_events,
  py::capsule pycontext)
{
  if (0 != std::strcmp("rcl_wait_set_t", pywait_set.name())) {
    throw py::value_error("capsule is not an rcl_wait_set_t");
  }
  auto wait_set = static_cast<rcl_wait_set_t *>(pywait_set);

  auto context = static_cast<rcl_context_t *>(
    rclpy_handle_get_pointer_from_capsule(pycontext.ptr(), "rcl_context_t"));
  if (!context) {
    throw py::error_already_set();
  }

  rcl_ret_t ret = rcl_wait_set_init(
    wait_set,
    number_of_subscriptions,
    number_of_guard_conditions,
    number_of_timers,
    number_of_clients,
    number_of_services,
    number_of_events,
    context,
    rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to initialize wait set");
  }
}

void
wait_set_clear_entities(py::capsule pywait_set)
{
  if (0 != std::strcmp("rcl_wait_set_t", pywait_set.name())) {
    throw py::value_error("capsule is not an rcl_wait_set_t");
  }
  auto wait_set = static_cast<rcl_wait_set_t *>(pywait_set);

  rcl_ret_t ret = rcl_wait_set_clear(wait_set);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to clear wait set");
  }
}

size_t
wait_set_add_entity(const std::string & entity_type, py::capsule pywait_set, py::capsule pyentity)
{
  if (0 != std::strcmp("rcl_wait_set_t", pywait_set.name())) {
    throw py::value_error("capsule is not an rcl_wait_set_t");
  }
  auto wait_set = static_cast<rcl_wait_set_t *>(pywait_set);

  rcl_ret_t ret = RCL_RET_ERROR;
  size_t index;

  if ("subscription" == entity_type) {
    auto sub = static_cast<rclpy_subscription_t *>(
      rclpy_handle_get_pointer_from_capsule(pyentity.ptr(), "rclpy_subscription_t"));
    if (!sub) {
      throw py::error_already_set();
    }
    ret = rcl_wait_set_add_subscription(wait_set, &(sub->subscription), &index);
  } else if ("guard_condition" == entity_type) {
    auto guard_condition = static_cast<rcl_guard_condition_t *>(
      rclpy_handle_get_pointer_from_capsule(pyentity.ptr(), "rcl_guard_condition_t"));
    if (!guard_condition) {
      throw py::error_already_set();
    }
    ret = rcl_wait_set_add_guard_condition(wait_set, guard_condition, &index);
  } else {
    std::string error_text{"'"};
    error_text += entity_type;
    error_text += "' is not a known entity";
    throw std::runtime_error(error_text);
  }
  if (ret != RCL_RET_OK) {
    std::string error_text{"failed to add'"};
    error_text += entity_type;
    error_text += "' to waitset";
    throw RCLError(error_text);
  }
  return index;
}

size_t
wait_set_add_service(const py::capsule pywait_set, const Service & service)
{
  if (0 != std::strcmp("rcl_wait_set_t", pywait_set.name())) {
    throw py::value_error("capsule is not an rcl_wait_set_t");
  }
  auto wait_set = static_cast<rcl_wait_set_t *>(pywait_set);

  size_t index;
  rcl_ret_t ret = rcl_wait_set_add_service(wait_set, service.rcl_ptr(), &index);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to add service to wait set");
  }
  return index;
}

size_t
wait_set_add_timer(const py::capsule pywait_set, const Timer & timer)
{
  if (0 != std::strcmp("rcl_wait_set_t", pywait_set.name())) {
    throw py::value_error("capsule is not an rcl_wait_set_t");
  }
  auto wait_set = static_cast<rcl_wait_set_t *>(pywait_set);

  size_t index;
  rcl_ret_t ret = rcl_wait_set_add_timer(wait_set, timer.rcl_ptr(), &index);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to add client to wait set");
  }
  return index;
}

size_t
wait_set_add_client(const py::capsule pywait_set, const Client & client)
{
  if (0 != std::strcmp("rcl_wait_set_t", pywait_set.name())) {
    throw py::value_error("capsule is not an rcl_wait_set_t");
  }
  auto wait_set = static_cast<rcl_wait_set_t *>(pywait_set);

  size_t index;
  rcl_ret_t ret = rcl_wait_set_add_client(wait_set, client.rcl_ptr(), &index);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to add client to wait set");
  }
  return index;
}

size_t
wait_set_add_event(const py::capsule pywait_set, const QoSEvent & event)
{
  if (0 != std::strcmp("rcl_wait_set_t", pywait_set.name())) {
    throw py::value_error("capsule is not an rcl_wait_set_t");
  }
  auto wait_set = static_cast<rcl_wait_set_t *>(pywait_set);

  size_t index;
  rcl_ret_t ret = rcl_wait_set_add_event(wait_set, event.rcl_ptr(), &index);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to add event to wait set");
  }
  return index;
}

bool
wait_set_is_ready(const std::string & entity_type, py::capsule pywait_set, size_t index)
{
  if (0 != std::strcmp("rcl_wait_set_t", pywait_set.name())) {
    throw py::value_error("capsule is not an rcl_wait_set_t");
  }
  auto wait_set = static_cast<rcl_wait_set_t *>(pywait_set);

  const void ** entities = NULL;
  size_t num_entities = 0;
  if ("subscription" == entity_type) {
    entities = reinterpret_cast<const void **>(wait_set->subscriptions);
    num_entities = wait_set->size_of_subscriptions;
  } else if ("client" == entity_type) {
    entities = reinterpret_cast<const void **>(wait_set->clients);
    num_entities = wait_set->size_of_clients;
  } else if ("service" == entity_type) {
    entities = reinterpret_cast<const void **>(wait_set->services);
    num_entities = wait_set->size_of_services;
  } else if ("timer" == entity_type) {
    entities = reinterpret_cast<const void **>(wait_set->timers);
    num_entities = wait_set->size_of_timers;
  } else if ("guard_condition" == entity_type) {
    entities = reinterpret_cast<const void **>(wait_set->guard_conditions);
    num_entities = wait_set->size_of_guard_conditions;
  } else if ("event" == entity_type) {
    entities = reinterpret_cast<const void **>(wait_set->events);
    num_entities = wait_set->size_of_events;
  } else {
    std::string error_text{"'"};
    error_text += entity_type;
    error_text += "' is not a known entity";
    throw std::runtime_error(error_text);
  }

  if (!entities) {
    std::string error_text{"wait set '"};
    error_text += entity_type;
    error_text += "' isn't allocated";
    throw std::runtime_error(error_text);
  }
  if (index >= num_entities) {
    throw std::out_of_range("wait set index too big");
  }
  return nullptr != entities[index];
}

template<typename EntityArray>
py::list
_get_ready_entities(const EntityArray ** entities, const size_t num_entities)
{
  py::list entity_list;
  for (size_t i = 0; i < num_entities; ++i) {
    auto address = reinterpret_cast<size_t>(entities[i]);
    if (address) {
      entity_list.append(address);
    }
  }
  return entity_list;
}

py::list
get_ready_entities(const std::string & entity_type, py::capsule pywait_set)
{
  if (0 != std::strcmp("rcl_wait_set_t", pywait_set.name())) {
    throw py::value_error("capsule is not an rcl_wait_set_t");
  }
  auto wait_set = static_cast<rcl_wait_set_t *>(pywait_set);

  if ("subscription" == entity_type) {
    return _get_ready_entities(wait_set->subscriptions, wait_set->size_of_subscriptions);
  } else if ("client" == entity_type) {
    return _get_ready_entities(wait_set->clients, wait_set->size_of_clients);
  } else if ("service" == entity_type) {
    return _get_ready_entities(wait_set->services, wait_set->size_of_services);
  } else if ("timer" == entity_type) {
    return _get_ready_entities(wait_set->timers, wait_set->size_of_timers);
  } else if ("guard_condition" == entity_type) {
    return _get_ready_entities(wait_set->guard_conditions, wait_set->size_of_guard_conditions);
  }

  std::string error_text{"'"};
  error_text += entity_type;
  error_text += "' is not a known entity";
  throw std::runtime_error(error_text);
}

void
wait(py::capsule pywait_set, int64_t timeout)
{
  if (0 != std::strcmp("rcl_wait_set_t", pywait_set.name())) {
    throw py::value_error("capsule is not an rcl_wait_set_t");
  }
  auto wait_set = static_cast<rcl_wait_set_t *>(pywait_set);

  rcl_ret_t ret;

  // Could be a long wait, release the GIL
  {
    py::gil_scoped_release gil_release;
    ret = rcl_wait(wait_set, timeout);
  }

  if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
    throw RCLError("failed to wait on wait set");
  }
}
}  // namespace rclpy
