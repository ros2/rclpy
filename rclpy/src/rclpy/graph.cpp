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

#include "rclpy_common/exceptions.hpp"
#include "rclpy_common/handle.h"

#include "graph.hpp"
#include "utils.hpp"


namespace rclpy
{

typedef rcl_ret_t (* rcl_get_info_by_topic_func_t)(
  const rcl_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rcl_topic_endpoint_info_array_t * info_array);

static PyObject *
_get_info_by_topic(
  PyObject * module,
  PyObject * args,
  const char * type,
  rcl_get_info_by_topic_func_t rcl_get_info_by_topic)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }
  PyObject * pynode;
  const char * topic_name;
  int no_mangle;

  if (!PyArg_ParseTuple(args, "Osp", &pynode, &topic_name, &no_mangle)) {
    return NULL;
  }

  rcl_node_t * node = rclpy_handle_get_pointer_from_capsule(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_topic_endpoint_info_array_t info_array = rcl_get_zero_initialized_topic_endpoint_info_array();
  rcl_ret_t ret = rcl_get_info_by_topic(node, &allocator, topic_name, no_mangle, &info_array);
  rcl_ret_t fini_ret;
  if (RCL_RET_OK != ret) {
    if (RCL_RET_BAD_ALLOC == ret) {
      PyErr_Format(
        PyExc_MemoryError, "Failed to get information by topic for %s: %s",
        type, rcl_get_error_string().str);
    } else if (RCL_RET_UNSUPPORTED == ret) {
      PyErr_Format(
        PyExc_NotImplementedError, "Failed to get information by topic for %s: "
        "function not supported by RMW_IMPLEMENTATION", type);
    } else {
      PyErr_Format(
        module_state->RCLError, "Failed to get information by topic for %s: %s",
        type, rcl_get_error_string().str);
    }
    rcl_reset_error();
    fini_ret = rcl_topic_endpoint_info_array_fini(&info_array, &allocator);
    if (fini_ret != RCL_RET_OK) {
      PyErr_Format(
        module_state->RCLError, "rcl_topic_endpoint_info_array_fini failed: %s",
        rcl_get_error_string().str);
      rcl_reset_error();
    }
    return NULL;
  }
  PyObject * py_info_array = rclpy_convert_to_py_topic_endpoint_info_list(&info_array);
  fini_ret = rcl_topic_endpoint_info_array_fini(&info_array, &allocator);
  if (RCL_RET_OK != fini_ret) {
    PyErr_Format(module_state->RCLError, "rcl_topic_endpoint_info_array_fini failed.");
    rcl_reset_error();
    return NULL;
  }
  return py_info_array;
}

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

/// Return a list of publishers on a given topic.
/**
 * The returned publisher information includes node name, node namespace, topic type, gid,
 * and qos profile
 *
 * \param[in] pynode Capsule pointing to the node to get the namespace from.
 * \param[in] topic_name the topic name to get the publishers for.
 * \param[in] no_mangle if `true`, `topic_name` needs to be a valid middleware topic name,
 *     otherwise it should be a valid ROS topic name.
 * \return list of publishers
 */
static PyObject *
graph_get_publishers_info_by_topic(PyObject * module, PyObject * args)
{
  return _get_info_by_topic(module, args, "publishers", rcl_get_publishers_info_by_topic);
}

/// Return a list of subscriptions on a given topic.
/**
 * The returned subscription information includes node name, node namespace, topic type, gid,
 * and qos profile
 *
 * \param[in] pynode Capsule pointing to the node to get the namespace from.
 * \param[in] topic_name the topic name to get the subscriptions for.
 * \param[in] no_mangle if `true`, `topic_name` needs to be a valid middleware topic name,
 *     otherwise it should be a valid ROS topic name.
 * \return list of subscriptions.
 */
static PyObject *
graph_get_subscriptions_info_by_topic(PyObject * module, PyObject * args)
{
  return _get_info_by_topic(module, args, "subscriptions", rcl_get_subscriptions_info_by_topic);
}

}  // namespace rclpy
