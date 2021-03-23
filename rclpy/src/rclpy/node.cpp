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
#include <rcl/rcl.h>
#include <rcpputils/scope_exit.hpp>


#include <string>

#include "rclpy_common/exceptions.hpp"
#include "rclpy_common/handle.h"

#include "node.hpp"
#include "utils.hpp"


namespace rclpy
{
const char *
get_node_fully_qualified_name(py::capsule pynode)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  const char * fully_qualified_node_name = rcl_node_get_fully_qualified_name(node);
  if (!fully_qualified_node_name) {
    throw RCLError("Fully qualified name not set");
  }

  return fully_qualified_node_name;
}

const char *
get_node_logger_name(py::capsule pynode)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  const char * node_logger_name = rcl_node_get_logger_name(node);
  if (!node_logger_name) {
    throw RCLError("Logger name not set");
  }

  return node_logger_name;
}

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

py::list
get_node_names_impl(py::capsule pynode, bool get_enclaves)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcutils_string_array_t node_names = rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t node_namespaces = rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t enclaves = rcutils_get_zero_initialized_string_array();

  rcl_ret_t ret = RCL_RET_OK;
  if (get_enclaves) {
    ret = rcl_get_node_names_with_enclaves(
      node, allocator, &node_names, &node_namespaces, &enclaves);
  } else {
    ret = rcl_get_node_names(
      node, allocator, &node_names, &node_namespaces);
  }
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to get node names");
  }

  py::list pynode_names_and_namespaces(node_names.size);

  RCPPUTILS_SCOPE_EXIT(
    {
      for (size_t idx = 0; idx < node_names.size; ++idx) {
        if (get_enclaves) {
          pynode_names_and_namespaces[idx] = py::make_tuple(
            py::str(node_names.data[idx]),
            py::str(node_namespaces.data[idx]),
            py::str(enclaves.data[idx]));
        } else {
          pynode_names_and_namespaces[idx] = py::make_tuple(
            py::str(node_names.data[idx]),
            py::str(node_namespaces.data[idx]));
        }
      }

      rcutils_ret_t fini_ret = rcutils_string_array_fini(&node_names);
      if (RCUTILS_RET_OK != fini_ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "failed to fini node names during error handling: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
      fini_ret = rcutils_string_array_fini(&node_namespaces);
      if (RCUTILS_RET_OK != fini_ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "failed to fini node namespaces during error handling: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
      fini_ret = rcutils_string_array_fini(&enclaves);
      if (RCUTILS_RET_OK != fini_ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "failed to fini enclaves string array during error handling: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
    });

  return pynode_names_and_namespaces;
}

py::list
get_node_names_and_namespaces(py::capsule pynode)
{
  return get_node_names_impl(pynode, false);
}

py::list
get_node_names_and_namespaces_with_enclaves(py::capsule pynode)
{
  return get_node_names_impl(pynode, true);
}
}  // namespace rclpy
