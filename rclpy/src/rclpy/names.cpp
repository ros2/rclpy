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

#include <rcl/context.h>
#include <rcl/error_handling.h>
#include <rcl/expand_topic_name.h>
#include <rcl/rcl.h>
#include <rcl/remap.h>
#include <rcl/types.h>
#include <rcl/validate_topic_name.h>
#include <rcutils/error_handling.h>
#include <rmw/error_handling.h>
#include <rmw/validate_full_topic_name.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>

#include <memory>
#include <stdexcept>
#include <string>

#include "exceptions.hpp"
#include "names.hpp"

namespace rclpy
{
py::object
get_validation_error_for_topic_name(const char * topic_name)
{
  int validation_result;
  size_t invalid_index;
  rcl_ret_t ret = rcl_validate_topic_name(topic_name, &validation_result, &invalid_index);
  if (RCL_RET_BAD_ALLOC == ret) {
    rcl_reset_error();
    throw std::bad_alloc();
  } else if (RCL_RET_OK != ret) {
    throw RCLError("failed to validate name");
  }

  if (RCL_TOPIC_NAME_VALID == validation_result) {
    return py::none();
  }

  const char * validation_message = rcl_topic_name_validation_result_string(validation_result);

  return py::make_tuple(validation_message, invalid_index);
}

py::object
get_validation_error_for_full_topic_name(const char * topic_name)
{
  int validation_result;
  size_t invalid_index;
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, &invalid_index);
  if (RMW_RET_BAD_ALLOC == ret) {
    rmw_reset_error();
    throw std::bad_alloc();
  } else if (RMW_RET_OK != ret) {
    throw RMWError("failed to validate name");
  }

  if (validation_result == RMW_NAMESPACE_VALID) {
    return py::none();
  }

  const char * validation_message = rmw_full_topic_name_validation_result_string(validation_result);

  return py::make_tuple(validation_message, invalid_index);
}

py::object
get_validation_error_for_namespace(const char * namespace_)
{
  int validation_result;
  size_t invalid_index;
  rmw_ret_t ret = rmw_validate_namespace(namespace_, &validation_result, &invalid_index);
  if (RMW_RET_BAD_ALLOC == ret) {
    rmw_reset_error();
    throw std::bad_alloc();
  } else if (RMW_RET_OK != ret) {
    throw RMWError("failed to validate namespace");
  }

  if (RMW_NAMESPACE_VALID == validation_result) {
    return py::none();
  }

  const char * validation_message = rmw_namespace_validation_result_string(validation_result);

  return py::make_tuple(validation_message, invalid_index);
}

py::object
get_validation_error_for_node_name(const char * node_name)
{
  int validation_result;
  size_t invalid_index;
  rmw_ret_t ret = rmw_validate_node_name(node_name, &validation_result, &invalid_index);
  if (RMW_RET_BAD_ALLOC == ret) {
    rmw_reset_error();
    throw std::bad_alloc();
  } else if (RMW_RET_OK != ret) {
    throw RMWError("failed to validate node name");
  }

  if (RMW_NODE_NAME_VALID == validation_result) {
    return py::none();
  }

  const char * validation_message = rmw_node_name_validation_result_string(validation_result);

  return py::make_tuple(validation_message, invalid_index);
}

std::string
expand_topic_name(const char * topic, const char * node_name, const char * node_namespace)
{
  rcutils_allocator_t rcutils_allocator = rcutils_get_default_allocator();
  rcutils_string_map_t substitutions_map = rcutils_get_zero_initialized_string_map();

  rcutils_ret_t rcutils_ret = rcutils_string_map_init(&substitutions_map, 0, rcutils_allocator);
  if (RCUTILS_RET_BAD_ALLOC == rcutils_ret) {
    rcl_reset_error();
    throw std::bad_alloc();
  } else if (RCUTILS_RET_OK != rcutils_ret) {
    throw RCUtilsError("failed to initialize string map");
  }

  rcl_ret_t ret = rcl_get_default_topic_name_substitutions(&substitutions_map);
  if (RCL_RET_BAD_ALLOC == ret) {
    rcl_reset_error();
    throw std::bad_alloc();
  } else if (RCL_RET_OK != ret) {
    auto exception = RCLError("failed to get default substitutions");
    // finalize the string map before returning
    rcutils_ret = rcutils_string_map_fini(&substitutions_map);
    if (RCUTILS_RET_OK != rcutils_ret) {
      RCUTILS_SAFE_FWRITE_TO_STDERR(
        "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
        "failed to fini string_map during error handling\n");
      RCUTILS_SAFE_FWRITE_TO_STDERR(rcutils_get_error_string().str);
      rcutils_reset_error();
    }
    throw exception;
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();

  char * output_cstr = nullptr;
  ret = rcl_expand_topic_name(
    topic,
    node_name,
    node_namespace,
    &substitutions_map,
    allocator,
    &output_cstr);

  auto topic_deleter = [&](char * topic) {allocator.deallocate(topic, allocator.state);};
  auto expanded_topic = std::unique_ptr<char, decltype(topic_deleter)>(
    output_cstr, topic_deleter);

  rcutils_ret = rcutils_string_map_fini(&substitutions_map);
  if (RCUTILS_RET_OK != rcutils_ret) {
    throw RCUtilsError("failed to fini string map");
  }

  if (RCL_RET_BAD_ALLOC == ret) {
    rcl_reset_error();
    throw std::bad_alloc();
  } else if (  // NOLINT
    ret == RCL_RET_TOPIC_NAME_INVALID ||
    ret == RCL_RET_UNKNOWN_SUBSTITUTION)
  {
    std::string error_text{"topic name '"};
    error_text += topic;
    error_text += "' is invalid";
    throw py::value_error(append_rcl_error(error_text));
  } else if (RCL_RET_NODE_INVALID_NAME == ret) {
    std::string error_text{"node name '"};
    error_text += node_name;
    error_text += "' is invalid";
    throw py::value_error(append_rcl_error(error_text));
  } else if (RCL_RET_NODE_INVALID_NAMESPACE == ret) {
    std::string error_text{"node namespace '"};
    error_text += node_namespace;
    error_text += "' is invalid";
    throw py::value_error(append_rcl_error(error_text));
  } else if (RCL_RET_OK != ret) {
    throw RCLError("failed to do name expansion");
  }

  return std::string{expanded_topic.get()};
}

std::string
remap_topic_name(Node & node, const char * topic_name)
{
  // Get the node options
  const rcl_node_options_t * node_options = rcl_node_get_options(node.rcl_ptr());
  if (nullptr == node_options) {
    throw RCLError("failed to get node options");
  }

  const rcl_arguments_t * global_args = nullptr;
  if (node_options->use_global_arguments) {
    global_args = &(node.rcl_ptr()->context->global_arguments);
  }

  char * output_cstr = nullptr;
  rcl_ret_t ret = rcl_remap_topic_name(
    &(node_options->arguments),
    global_args,
    topic_name,
    rcl_node_get_name(node.rcl_ptr()),
    rcl_node_get_namespace(node.rcl_ptr()),
    node_options->allocator,
    &output_cstr);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to remap topic name");
  }

  if (nullptr == output_cstr) {
    // no remapping found
    return std::string{topic_name};
  }

  auto name_deleter = [&](char * name) {
      node_options->allocator.deallocate(name, node_options->allocator.state);
    };
  auto remapped_name = std::unique_ptr<char, decltype(name_deleter)>(
    output_cstr, name_deleter);

  return std::string{remapped_name.get()};
}

std::string
resolve_name(Node & node, const char * topic_name, bool only_expand, bool is_service)
{
  const rcl_node_options_t * node_options = rcl_node_get_options(node.rcl_ptr());
  if (nullptr == node_options) {
    throw RCLError("failed to get node options");
  }

  char * output_cstr = nullptr;
  rcl_ret_t ret = rcl_node_resolve_name(
    node.rcl_ptr(),
    topic_name,
    node_options->allocator,
    is_service,
    only_expand,
    &output_cstr);

  auto name_deleter = [&](char * name) {
      node_options->allocator.deallocate(name, node_options->allocator.state);
    };
  auto resolved_name = std::unique_ptr<char, decltype(name_deleter)>(
    output_cstr, name_deleter);

  if (RCL_RET_OK != ret) {
    throw RCLError("failed to resolve name");
  }

  return std::string{resolved_name.get()};
}
}  // namespace rclpy
