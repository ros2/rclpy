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

#include <pybind11/pybind11.h>

#include <rcl/allocator.h>
#include <rcl/error_handling.h>
#include <rcl/graph.h>
#include <rcutils/error_handling.h>

#include <string>

#include <rcpputils/scope_exit.hpp>

#include "exceptions.hpp"
#include "graph.hpp"
#include "node.hpp"
#include "utils.hpp"

namespace rclpy
{

py::list
graph_get_publisher_names_and_types_by_node(
  Node & node, bool no_demangle,
  std::string node_name, std::string node_namespace)
{
  rcl_names_and_types_t publisher_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_publisher_names_and_types_by_node(
    node.rcl_ptr(), &allocator, no_demangle, node_name.c_str(),
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
  Node & node, bool no_demangle,
  std::string node_name, std::string node_namespace)
{
  rcl_names_and_types_t subscriber_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_subscriber_names_and_types_by_node(
    node.rcl_ptr(), &allocator, no_demangle, node_name.c_str(),
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
  Node & node, std::string node_name, std::string node_namespace)
{
  rcl_names_and_types_t service_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_service_names_and_types_by_node(
    node.rcl_ptr(), &allocator, node_name.c_str(), node_namespace.c_str(),
    &service_names_and_types);
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
  Node & node, std::string node_name, std::string node_namespace)
{
  rcl_names_and_types_t client_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_client_names_and_types_by_node(
    node.rcl_ptr(), &allocator, node_name.c_str(), node_namespace.c_str(), &client_names_and_types);
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
graph_get_topic_names_and_types(Node & node, bool no_demangle)
{
  rcl_names_and_types_t topic_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret =
    rcl_get_topic_names_and_types(node.rcl_ptr(), &allocator, no_demangle, &topic_names_and_types);
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
graph_get_service_names_and_types(Node & node)
{
  rcl_names_and_types_t service_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_service_names_and_types(
    node.rcl_ptr(), &allocator, &service_names_and_types);
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

typedef rcl_ret_t (* rcl_get_info_by_topic_func_t)(
  const rcl_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rcl_topic_endpoint_info_array_t * info_array);

py::list
_get_info_by_topic(
  Node & node,
  const char * topic_name,
  bool no_mangle,
  const char * type,
  rcl_get_info_by_topic_func_t rcl_get_info_by_topic)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_topic_endpoint_info_array_t info_array = rcl_get_zero_initialized_topic_endpoint_info_array();

  RCPPUTILS_SCOPE_EXIT(
    {
      rcl_ret_t fini_ret = rcl_topic_endpoint_info_array_fini(&info_array, &allocator);
      if (RCL_RET_OK != fini_ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "rcl_topic_endpoint_info_array_fini failed: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
    });

  rcl_ret_t ret = rcl_get_info_by_topic(
    node.rcl_ptr(), &allocator, topic_name, no_mangle, &info_array);
  if (RCL_RET_OK != ret) {
    if (RCL_RET_UNSUPPORTED == ret) {
      throw NotImplementedError(
              std::string("Failed to get information by topic for ") +
              type + ": function not supported by RMW_IMPLEMENTATION");
    }
    throw RCLError(
            std::string("Failed to get information by topic for ") + type);
  }

  return convert_to_py_topic_endpoint_info_list(&info_array);
}

py::list
graph_get_publishers_info_by_topic(
  Node & node, const char * topic_name, bool no_mangle)
{
  return _get_info_by_topic(
    node, topic_name, no_mangle, "publishers",
    rcl_get_publishers_info_by_topic);
}

py::list
graph_get_subscriptions_info_by_topic(
  Node & node, const char * topic_name, bool no_mangle)
{
  return _get_info_by_topic(
    node, topic_name, no_mangle, "subscriptions",
    rcl_get_subscriptions_info_by_topic);
}

}  // namespace rclpy
