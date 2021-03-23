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

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "publisher.hpp"

namespace rclpy
{
static void
_rclpy_destroy_publisher(void * p)
{
  auto pub = static_cast<rclpy_publisher_t *>(p);
  if (!pub) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_publisher got NULL pointer");
    return;
  }

  rcl_ret_t ret = rcl_publisher_fini(&(pub->publisher), pub->node);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini publisher: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(pub);
}

py::capsule
publisher_create(
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

  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  if (!pyqos_profile.is_none()) {
    if (0 != strcmp("rmw_qos_profile_t", pyqos_profile.name())) {
      throw py::value_error("capsule is not an rmw_qos_profile_t");
    }
    auto qos_profile = static_cast<rmw_qos_profile_t *>(pyqos_profile);
    publisher_ops.qos = *qos_profile;
  }

  // Use smart pointer to make sure memory is free'd on error
  auto deleter = [](rclpy_publisher_t * ptr) {_rclpy_destroy_publisher(ptr);};
  auto pub = std::unique_ptr<rclpy_publisher_t, decltype(deleter)>(
    static_cast<rclpy_publisher_t *>(PyMem_Malloc(sizeof(rclpy_publisher_t))),
    deleter);
  if (!pub) {
    throw std::bad_alloc();
  }
  pub->publisher = rcl_get_zero_initialized_publisher();
  pub->node = node;

  rcl_ret_t ret = rcl_publisher_init(
    &(pub->publisher), node, msg_type,
    topic.c_str(), &publisher_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      std::string error_text{"Failed to create publisher due to invalid topic name '"};
      error_text += topic;
      error_text += "'";
      throw py::value_error(error_text);
    }
    throw RCLError("Failed to create publisher");
  }

  PyObject * pypub_c =
    rclpy_create_handle_capsule(pub.get(), "rclpy_publisher_t", _rclpy_destroy_publisher);
  if (!pypub_c) {
    throw py::error_already_set();
  }
  auto pypub = py::reinterpret_steal<py::capsule>(pypub_c);
  // pypub now owns the rclpy_publisher_t
  pub.release();

  auto pub_handle = static_cast<rclpy_handle_t *>(pypub);
  auto node_handle = static_cast<rclpy_handle_t *>(pynode);
  _rclpy_handle_add_dependency(pub_handle, node_handle);
  if (PyErr_Occurred()) {
    _rclpy_handle_dec_ref(pub_handle);
    throw py::error_already_set();
  }

  return pypub;
}

const char *
publisher_get_logger_name(py::capsule pypublisher)
{
  auto pub = static_cast<rclpy_publisher_t *>(
    rclpy_handle_get_pointer_from_capsule(pypublisher.ptr(), "rclpy_publisher_t"));
  if (!pub) {
    throw py::error_already_set();
  }

  const char * node_logger_name = rcl_node_get_logger_name(&pub->node);
  if (!node_logger_name) {
    throw RCLError("Node logger name not set");
  }

  return node_logger_name;
}

size_t
publisher_get_subscription_count(py::capsule pypublisher)
{
  auto pub = static_cast<rclpy_publisher_t *>(
    rclpy_handle_get_pointer_from_capsule(pypublisher.ptr(), "rclpy_publisher_t"));
  if (!pub) {
    throw py::error_already_set();
  }

  size_t count = 0;
  rcl_ret_t ret = rcl_publisher_get_subscription_count(&pub->publisher, &count);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get subscription count");
  }

  return count;
}

std::string
publisher_get_topic_name(py::capsule pypublisher)
{
  auto pub = static_cast<rclpy_publisher_t *>(
    rclpy_handle_get_pointer_from_capsule(pypublisher.ptr(), "rclpy_publisher_t"));
  if (!pub) {
    throw py::error_already_set();
  }

  const char * topic_name = rcl_publisher_get_topic_name(&pub->publisher);
  if (!topic_name) {
    throw RCLError("failed to get topic name");
  }

  return std::string(topic_name);
}

void
publisher_publish_message(py::capsule pypublisher, py::object pymsg)
{
  auto pub = static_cast<rclpy_publisher_t *>(
    rclpy_handle_get_pointer_from_capsule(pypublisher.ptr(), "rclpy_publisher_t"));
  if (!pub) {
    throw py::error_already_set();
  }

  destroy_ros_message_signature * destroy_ros_message = NULL;
  void * raw_ros_message = rclpy_convert_from_py(pymsg.ptr(), &destroy_ros_message);
  if (!raw_ros_message) {
    throw py::error_already_set();
  }

  rcl_ret_t ret = rcl_publish(&(pub->publisher), raw_ros_message, NULL);
  destroy_ros_message(raw_ros_message);
  if (ret != RCL_RET_OK) {
    throw RCLError("Failed to publish");
  }
}

void
publisher_publish_raw(py::capsule pypublisher, std::string msg)
{
  auto pub = static_cast<rclpy_publisher_t *>(
    rclpy_handle_get_pointer_from_capsule(pypublisher.ptr(), "rclpy_publisher_t"));
  if (!pub) {
    throw py::error_already_set();
  }

  rcl_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
  serialized_msg.buffer_capacity = msg.size();
  serialized_msg.buffer_length = msg.size();
  serialized_msg.buffer = reinterpret_cast<uint8_t *>(const_cast<char *>(msg.c_str()));

  rcl_ret_t ret = rcl_publish_serialized_message(&(pub->publisher), &serialized_msg, NULL);
  if (ret != RCL_RET_OK) {
    throw RCLError("Failed to publish");
  }
}
}  // namespace rclpy
