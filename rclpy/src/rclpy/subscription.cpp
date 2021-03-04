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

#include <memory>
#include <string>

extern "C"
{
#include "rclpy_common/common.h"
}
#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "subscription.hpp"

namespace rclpy
{
static void
_rclpy_destroy_subscription(void * p)
{
  auto sub = static_cast<rclpy_subscription_t *>(p);
  if (!sub) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_subscription got NULL pointer");
    return;
  }

  rcl_ret_t ret = rcl_subscription_fini(&(sub->subscription), sub->node);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini subscription: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(sub);
}

py::capsule
subscription_create(
  py::capsule pynode, py::object pymsg_type, std::string topic,
  py::capsule pyqos_profile)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  auto msg_type = static_cast<rosidl_message_type_support_t *>(
    rclpy_common_get_type_support(pymsg_type.ptr()));
  if (!msg_type) {
    throw py::error_already_set();
  }

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();

  if (!pyqos_profile.is(py::none())) {
    if (0 != strcmp("rmw_qos_profile_t", pyqos_profile.name())) {
      throw py::value_error("capsule is not an rmw_qos_profile_t");
    }
    auto qos_profile = static_cast<rmw_qos_profile_t *>(pyqos_profile);
    subscription_ops.qos = *qos_profile;
  }

  // Use smart pointer to make sure memory is free'd on error
  auto deleter = [](rclpy_subscription_t * ptr) {_rclpy_destroy_subscription(ptr);};
  auto sub = std::unique_ptr<rclpy_subscription_t, decltype(deleter)>(
    static_cast<rclpy_subscription_t *>(PyMem_Malloc(sizeof(rclpy_subscription_t))),
    deleter);
  if (!sub) {
    throw std::bad_alloc();
  }
  sub->subscription = rcl_get_zero_initialized_subscription();
  sub->node = node;

  rcl_ret_t ret = rcl_subscription_init(
    &(sub->subscription), node, msg_type,
    topic.c_str(), &subscription_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      std::string error_text{"Failed to create subscription due to invalid topic name '"};
      error_text += topic;
      error_text += "'";
      throw RCLError(error_text);
    }
    throw RCLError("Failed to create subscription");
  }

  PyObject * pysub_c =
    rclpy_create_handle_capsule(sub.get(), "rclpy_subscription_t", _rclpy_destroy_subscription);
  if (!pysub_c) {
    throw py::error_already_set();
  }
  auto pysub = py::reinterpret_steal<py::capsule>(pysub_c);
  // pysub now owns the rclpy_subscription_t
  sub.release();

  auto sub_handle = static_cast<rclpy_handle_t *>(pysub);
  auto node_handle = static_cast<rclpy_handle_t *>(pynode);
  _rclpy_handle_add_dependency(sub_handle, node_handle);
  if (PyErr_Occurred()) {
    _rclpy_handle_dec_ref(node_handle);
    throw py::error_already_set();
  }

  return pysub;
}

py::object
subscription_get_logger_name(py::capsule pysubscription)
{
  auto sub = static_cast<rclpy_subscription_t *>(
    rclpy_handle_get_pointer_from_capsule(pysubscription.ptr(), "rclpy_subscription_t"));
  if (!sub) {
    throw py::error_already_set();
  }

  const char * node_logger_name = rcl_node_get_logger_name(sub->node);
  if (NULL == node_logger_name) {
    return py::none();
  }

  return py::str(node_logger_name);
}

std::string
subscription_get_topic_name(py::capsule pysubscription)
{
  auto sub = static_cast<rclpy_subscription_t *>(
    rclpy_handle_get_pointer_from_capsule(pysubscription.ptr(), "rclpy_subscription_t"));
  if (!sub) {
    throw py::error_already_set();
  }

  const char * subscription_name = rcl_subscription_get_topic_name(&(sub->subscription));
  if (NULL == subscription_name) {
    throw RCLError("failed to get subscription topic name");
  }

  return std::string(subscription_name);
}
}  // namespace rclpy
