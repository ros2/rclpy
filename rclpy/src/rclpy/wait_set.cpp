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
WaitSet::WaitSet()
{
  // Create a client
  rcl_wait_set_ = std::shared_ptr<rcl_wait_set_t>(
    new rcl_wait_set_t,
    [](rcl_wait_set_t * waitset)
    {
      rcl_ret_t ret = rcl_wait_set_fini(waitset);
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini wait set: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete waitset;
    });
  *rcl_wait_set_ = rcl_get_zero_initialized_wait_set();
}

WaitSet::WaitSet(
  size_t number_of_subscriptions,
  size_t number_of_guard_conditions,
  size_t number_of_timers,
  size_t number_of_clients,
  size_t number_of_services,
  size_t number_of_events,
  py::capsule pycontext)
{
  // Create a client
  rcl_wait_set_ = std::shared_ptr<rcl_wait_set_t>(
    new rcl_wait_set_t,
    [](rcl_wait_set_t * waitset)
    {
      rcl_ret_t ret = rcl_wait_set_fini(waitset);
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini wait set: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete waitset;
    });
  *rcl_wait_set_ = rcl_get_zero_initialized_wait_set();

  auto context = static_cast<rcl_context_t *>(
    rclpy_handle_get_pointer_from_capsule(pycontext.ptr(), "rcl_context_t"));
  if (!context) {
    throw py::error_already_set();
  }

  rcl_ret_t ret = rcl_wait_set_init(
    rcl_wait_set_.get(),
    number_of_subscriptions,
    number_of_guard_conditions,
    number_of_timers,
    number_of_clients,
    number_of_services,
    number_of_events,
    context,
    rcl_get_default_allocator());
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to initialize wait set");
  }
}

void
WaitSet::destroy()
{
  rcl_wait_set_.reset();
}

void
WaitSet::wait_set_clear_entities()
{
  rcl_ret_t ret = rcl_wait_set_clear(rcl_wait_set_.get());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to clear wait set");
  }
}

size_t
WaitSet::wait_set_add_entity(const std::string & entity_type, py::capsule pyentity)
{
  rcl_ret_t ret = RCL_RET_ERROR;
  size_t index;

  if ("guard_condition" == entity_type) {
    auto guard_condition = static_cast<rcl_guard_condition_t *>(
      rclpy_handle_get_pointer_from_capsule(pyentity.ptr(), "rcl_guard_condition_t"));
    if (!guard_condition) {
      throw py::error_already_set();
    }
    ret = rcl_wait_set_add_guard_condition(rcl_wait_set_.get(), guard_condition, &index);
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
WaitSet::wait_set_add_service(const Service & service)
{
  size_t index;
  rcl_ret_t ret = rcl_wait_set_add_service(rcl_wait_set_.get(), service.rcl_ptr(), &index);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to add service to wait set");
  }
  return index;
}

size_t
WaitSet::wait_set_add_subscription(const Subscription & subscription)
{
  size_t index;
  rcl_ret_t ret = rcl_wait_set_add_subscription(
    rcl_wait_set_.get(), subscription.rcl_ptr(), &index);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to add subscription to wait set");
  }
  return index;
}

size_t
WaitSet::wait_set_add_timer(const Timer & timer)
{
  size_t index;
  rcl_ret_t ret = rcl_wait_set_add_timer(rcl_wait_set_.get(), timer.rcl_ptr(), &index);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to add client to wait set");
  }
  return index;
}

size_t
WaitSet::wait_set_add_client(const Client & client)
{
  size_t index;
  rcl_ret_t ret = rcl_wait_set_add_client(rcl_wait_set_.get(), client.rcl_ptr(), &index);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to add client to wait set");
  }
  return index;
}

size_t
WaitSet::wait_set_add_event(const QoSEvent & event)
{
  size_t index;
  rcl_ret_t ret = rcl_wait_set_add_event(rcl_wait_set_.get(), event.rcl_ptr(), &index);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to add event to wait set");
  }
  return index;
}

bool
WaitSet::wait_set_is_ready(const std::string & entity_type, size_t index)
{
  const void ** entities = NULL;
  size_t num_entities = 0;
  if ("subscription" == entity_type) {
    entities = reinterpret_cast<const void **>(rcl_wait_set_.get()->subscriptions);
    num_entities = rcl_wait_set_.get()->size_of_subscriptions;
  } else if ("client" == entity_type) {
    entities = reinterpret_cast<const void **>(rcl_wait_set_.get()->clients);
    num_entities = rcl_wait_set_.get()->size_of_clients;
  } else if ("service" == entity_type) {
    entities = reinterpret_cast<const void **>(rcl_wait_set_.get()->services);
    num_entities = rcl_wait_set_.get()->size_of_services;
  } else if ("timer" == entity_type) {
    entities = reinterpret_cast<const void **>(rcl_wait_set_.get()->timers);
    num_entities = rcl_wait_set_.get()->size_of_timers;
  } else if ("guard_condition" == entity_type) {
    entities = reinterpret_cast<const void **>(rcl_wait_set_.get()->guard_conditions);
    num_entities = rcl_wait_set_.get()->size_of_guard_conditions;
  } else if ("event" == entity_type) {
    entities = reinterpret_cast<const void **>(rcl_wait_set_.get()->events);
    num_entities = rcl_wait_set_.get()->size_of_events;
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
WaitSet::get_ready_entities(const std::string & entity_type)
{
  if ("subscription" == entity_type) {
    return _get_ready_entities(
      rcl_wait_set_.get()->subscriptions, rcl_wait_set_.get()->size_of_subscriptions);
  } else if ("client" == entity_type) {
    return _get_ready_entities(
      rcl_wait_set_.get()->clients, rcl_wait_set_.get()->size_of_clients);
  } else if ("service" == entity_type) {
    return _get_ready_entities(
      rcl_wait_set_.get()->services, rcl_wait_set_.get()->size_of_services);
  } else if ("timer" == entity_type) {
    return _get_ready_entities(
      rcl_wait_set_.get()->timers, rcl_wait_set_.get()->size_of_timers);
  } else if ("guard_condition" == entity_type) {
    return _get_ready_entities(
      rcl_wait_set_.get()->guard_conditions, rcl_wait_set_.get()->size_of_guard_conditions);
  }

  std::string error_text{"'"};
  error_text += entity_type;
  error_text += "' is not a known entity";
  throw std::runtime_error(error_text);
}

void
WaitSet::wait(int64_t timeout)
{
  rcl_ret_t ret;

  // Could be a long wait, release the GIL
  {
    py::gil_scoped_release gil_release;
    ret = rcl_wait(rcl_wait_set_.get(), timeout);
  }

  if (RCL_RET_OK != ret && RCL_RET_TIMEOUT != ret) {
    throw RCLError("failed to wait on wait set");
  }
}
void define_waitset(py::object module)
{
  py::class_<WaitSet, Destroyable, std::shared_ptr<WaitSet>>(module, "WaitSet")
  .def(py::init<size_t, size_t, size_t, size_t, size_t, size_t, py::capsule>())
  .def(py::init<>())
  .def_property_readonly(
    "pointer", [](const WaitSet & waitset) {
      return reinterpret_cast<size_t>(waitset.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "wait_set_clear_entities", &WaitSet::wait_set_clear_entities,
    "Clear all the pointers in the wait set")
  .def(
    "wait_set_add_entity", &WaitSet::wait_set_add_entity,
    "Add an entity to the wait set structure")
  .def(
    "wait_set_add_service", &WaitSet::wait_set_add_service,
    "Add a service to the wait set structure")
  .def(
    "wait_set_add_subscription", &WaitSet::wait_set_add_subscription,
    "Add a subcription to the wait set structure")
  .def(
    "wait_set_add_client", &WaitSet::wait_set_add_client,
    "Add a client to the wait set structure")
  .def(
    "wait_set_add_timer", &WaitSet::wait_set_add_timer,
    "Add a timer to the wait set structure")
  .def(
    "wait_set_clear_entities", &WaitSet::wait_set_clear_entities,
    "Add a client to the wait set structure")
  .def(
    "wait_set_add_event", &WaitSet::wait_set_add_event,
    "Add an event to the wait set structure")
  .def(
    "wait_set_is_ready", &WaitSet::wait_set_is_ready,
    "Check if an entity in the wait set is ready by its index")
  .def(
    "get_ready_entities", &WaitSet::get_ready_entities,
    "Get list of non-null entities in wait set")
  .def(
    "wait", &WaitSet::wait,
    "Wait until timeout is reached or event happened");
}
}  // namespace rclpy
