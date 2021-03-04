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

#include <string>

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "publisher.hpp"

namespace rclpy
{
size_t
publisher_get_subscription_count(py::capsule pypublisher)
{
  auto publisher = static_cast<rclpy_publisher_t *>(
    rclpy_handle_get_pointer_from_capsule(pypublisher.ptr(), "rclpy_publisher_t"));
  if (!publisher) {
    throw py::error_already_set();
  }

  size_t count = 0;
  rcl_ret_t ret = rcl_publisher_get_subscription_count(&publisher->publisher, &count);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get subscription count");
  }

  return count;
}

std::string
publisher_get_topic_name(py::capsule pypublisher)
{
  auto publisher = static_cast<rclpy_publisher_t *>(
    rclpy_handle_get_pointer_from_capsule(pypublisher.ptr(), "rclpy_publisher_t"));
  if (!publisher) {
    throw py::error_already_set();
  }

  const char * topic_name = rcl_publisher_get_topic_name(&publisher->publisher);
  if (!topic_name) {
    throw RCLError("failed to get topic name");
  }

  return std::string(topic_name);
}
}  // namespace rclpy
