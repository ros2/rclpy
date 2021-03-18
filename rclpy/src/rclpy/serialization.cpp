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
#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>

#include <memory>
#include <stdexcept>

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "serialization.hpp"

namespace rclpy
{
struct SerializedMessage
{
  explicit SerializedMessage(rcutils_allocator_t allocator)
  {
    rcl_msg = rmw_get_zero_initialized_serialized_message();
    rcutils_ret_t rcutils_ret = rmw_serialized_message_init(&rcl_msg, 0u, &allocator);
    if (RCUTILS_RET_OK != rcutils_ret) {
      throw RCUtilsError("failed to initialize serialized message");
    }
  }

  ~SerializedMessage()
  {
    rcutils_ret_t ret = rmw_serialized_message_fini(&rcl_msg);
    if (RCUTILS_RET_OK != ret) {
      RCUTILS_SAFE_FWRITE_TO_STDERR(
        "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
        "failed to fini rcl_serialized_msg_t in destructor:");
      RCUTILS_SAFE_FWRITE_TO_STDERR(rcutils_get_error_string().str);
      RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
      rcutils_reset_error();
    }
  }

  rcl_serialized_message_t rcl_msg;
};

py::bytes
serialize(py::object pymsg, py::object pymsg_type)
{
  // Get type support
  auto ts = static_cast<rosidl_message_type_support_t *>(
    rclpy_common_get_type_support(pymsg_type.ptr()));
  if (!ts) {
    throw py::error_already_set();
  }

  destroy_ros_message_signature * destroy_ros_message = nullptr;
  auto ros_msg = std::unique_ptr<void, decltype(destroy_ros_message)>(
    rclpy_convert_from_py(pymsg.ptr(), &destroy_ros_message), destroy_ros_message);
  if (!ros_msg) {
    throw py::error_already_set();
  }

  // Create a serialized message object
  SerializedMessage serialized_msg(rcutils_get_default_allocator());

  // Serialize
  rmw_ret_t rmw_ret = rmw_serialize(ros_msg.get(), ts, &serialized_msg.rcl_msg);
  if (RMW_RET_OK != rmw_ret) {
    throw RMWError("Failed to serialize ROS message");
  }

  // Bundle serialized message in a bytes object
  return py::bytes(
    reinterpret_cast<const char *>(serialized_msg.rcl_msg.buffer),
    serialized_msg.rcl_msg.buffer_length);
}

py::object
deserialize(py::bytes pybuffer, py::object pymsg_type)
{
  // Get type support
  auto ts = static_cast<rosidl_message_type_support_t *>(
    rclpy_common_get_type_support(pymsg_type.ptr()));
  if (!ts) {
    throw py::error_already_set();
  }

  // Create a serialized message object
  rcl_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
  // Just copy pointer to avoid extra allocation and copy
  char * serialized_buffer;
  Py_ssize_t length;
  if (PYBIND11_BYTES_AS_STRING_AND_SIZE(pybuffer.ptr(), &serialized_buffer, &length)) {
    throw py::error_already_set();
  }
  if (length < 0) {
    throw py::error_already_set();
  }
  serialized_msg.buffer_capacity = length;
  serialized_msg.buffer_length = length;
  serialized_msg.buffer = reinterpret_cast<uint8_t *>(serialized_buffer);

  destroy_ros_message_signature * destroy_ros_message = nullptr;
  void * deserialized_ros_msg_c = rclpy_create_from_py(pymsg_type.ptr(), &destroy_ros_message);
  if (!deserialized_ros_msg_c) {
    throw py::error_already_set();
  }
  auto deserialized_ros_msg = std::unique_ptr<void, decltype(destroy_ros_message)>(
    deserialized_ros_msg_c, destroy_ros_message);

  // Deserialize
  rmw_ret_t rmw_ret = rmw_deserialize(&serialized_msg, ts, deserialized_ros_msg.get());

  if (RMW_RET_OK != rmw_ret) {
    throw RMWError("failed to deserialize ROS message");
  }

  PyObject * pydeserialized_ros_msg_c =
    rclpy_convert_to_py(deserialized_ros_msg.get(), pymsg_type.ptr());
  return py::reinterpret_steal<py::object>(pydeserialized_ros_msg_c);
}
}  // namespace rclpy
