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
#include <rcl/graph.h>
#include <rcutils/error_handling.h>
#include <rcpputils/scope_exit.hpp>

#include <memory>
#include <string>

#include "rclpy_common/common.hpp"
#include "rclpy_common/exceptions.hpp"
#include "rclpy_common/handle.h"

#include "graph.hpp"


namespace rclpy
{

py::list
graph_get_publisher_names_and_types_by_node(
  py::capsule pynode, bool no_demangle,
  std::string node_name, std::string node_namespace)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_names_and_types_t publisher_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_publisher_names_and_types_by_node(
    node, &allocator, no_demangle, node_name.c_str(),
    node_namespace.c_str(), &publisher_names_and_types);
  if (RCL_RET_OK != ret) {
    if (RCL_RET_NODE_NAME_NON_EXISTENT == ret) {
      throw NodeNameNonExistentError(
              "cannot get publisher names and types for nonexistent node");
    }
    throw RCLError("failed to get publisher names and types");
  }
  RCPPUTILS_SCOPE_EXIT(
    {
      ret = rcl_names_and_types_fini(&publisher_names_and_types);
      if (RCL_RET_OK != ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "failed to fini publisher names and types during error handling: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
    });

  return convert_to_py_names_and_types(&publisher_names_and_types);
}

py::list
graph_get_subscriber_names_and_types_by_node(
  py::capsule pynode, bool no_demangle,
  std::string node_name, std::string node_namespace)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_names_and_types_t subscriber_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_subscriber_names_and_types_by_node(
    node, &allocator, no_demangle, node_name.c_str(),
    node_namespace.c_str(), &subscriber_names_and_types);
  if (RCL_RET_OK != ret) {
    if (RCL_RET_NODE_NAME_NON_EXISTENT == ret) {
      throw NodeNameNonExistentError(
              "cannot get subscriber names and types for nonexistent node");
    }
    throw RCLError("failed to get subscriber names and types");
  }
  RCPPUTILS_SCOPE_EXIT(
    {
      ret = rcl_names_and_types_fini(&subscriber_names_and_types);
      if (RCL_RET_OK != ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "failed to fini subscriber names and types during error handling: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
    });

  return convert_to_py_names_and_types(&subscriber_names_and_types);
}

py::list
graph_get_service_names_and_types_by_node(
  py::capsule pynode, std::string node_name, std::string node_namespace)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_names_and_types_t service_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_service_names_and_types_by_node(
    node, &allocator, node_name.c_str(), node_namespace.c_str(), &service_names_and_types);
  if (RCL_RET_OK != ret) {
    if (RCL_RET_NODE_NAME_NON_EXISTENT == ret) {
      throw NodeNameNonExistentError(
              "cannot get service names and types for nonexistent node");
    }
    throw RCLError("failed to get service names and types");
  }
  RCPPUTILS_SCOPE_EXIT(
    {
      ret = rcl_names_and_types_fini(&service_names_and_types);
      if (RCL_RET_OK != ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "failed to fini service names and types during error handling: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
    });

  return convert_to_py_names_and_types(&service_names_and_types);
}

py::list
graph_get_client_names_and_types_by_node(
  py::capsule pynode, std::string node_name, std::string node_namespace)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_names_and_types_t client_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_client_names_and_types_by_node(
    node, &allocator, node_name.c_str(), node_namespace.c_str(), &client_names_and_types);
  if (RCL_RET_OK != ret) {
    if (RCL_RET_NODE_NAME_NON_EXISTENT == ret) {
      throw NodeNameNonExistentError(
              "cannot get client names and types for nonexistent node");
    }
    throw RCLError("failed to get client names and types");
  }
  RCPPUTILS_SCOPE_EXIT(
    {
      ret = rcl_names_and_types_fini(&client_names_and_types);
      if (RCL_RET_OK != ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "failed to fini client names and types during error handling: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
    });

  return convert_to_py_names_and_types(&client_names_and_types);
}

py::list
graph_get_topic_names_and_types(py::capsule pynode, bool no_demangle)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_names_and_types_t topic_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret =
    rcl_get_topic_names_and_types(node, &allocator, no_demangle, &topic_names_and_types);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to get topic names and types");
  }

  RCPPUTILS_SCOPE_EXIT(
    {
      ret = rcl_names_and_types_fini(&topic_names_and_types);
      if (RCL_RET_OK != ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "failed to fini topic names and types during error handling: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
    });

  return convert_to_py_names_and_types(&topic_names_and_types);
}

py::list
graph_get_service_names_and_types(py::capsule pynode)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_names_and_types_t service_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_service_names_and_types(node, &allocator, &service_names_and_types);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to get service names and types");
  }
  RCPPUTILS_SCOPE_EXIT(
    {
      ret = rcl_names_and_types_fini(&service_names_and_types);
      if (RCL_RET_OK != ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "failed to fini service names and types during error handling: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
    });

  return convert_to_py_names_and_types(&service_names_and_types);
}

}  // namespace rclpy
