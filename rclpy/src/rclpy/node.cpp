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
#include <rcl/graph.h>

#include <string>

#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "node.hpp"


namespace rclpy
{
const char *
get_node_name(py::capsule pynode)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  const char * node_name = rcl_node_get_name(node);
  if (!node_name) {
    throw RCLError("Node name not set");
  }

  return node_name;
}

const char *
get_node_namespace(py::capsule pynode)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  const char * node_namespace = rcl_node_get_namespace(node);
  if (!node_namespace) {
    throw RCLError("Node namespace not set");
  }

  return node_namespace;
}

unsigned int
get_count_publishers(py::capsule pynode, const char * topic_name)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  size_t count = 0;
  rcl_ret_t ret = rcl_count_publishers(node, topic_name, &count);
  if (ret != RCL_RET_OK) {
    throw RCLError(std::string("Error in rcl_count_publishers: ") +
      rcl_get_error_string().str);
  }

  return count;
}

unsigned int
get_count_subscribers(py::capsule pynode, const char * topic_name)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  size_t count = 0;
  rcl_ret_t ret = rcl_count_subscribers(node, topic_name, &count);
  if (ret != RCL_RET_OK) {
    throw RCLError(std::string("Error in rcl_count_subscribers: ") +
      rcl_get_error_string().str);
  }

  return count;
}

}  // namespace rclpy
