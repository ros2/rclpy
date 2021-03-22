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

#include <string>

#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "node.hpp"


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

/// Get the list of nodes discovered by the provided node
/**
 *  Raises ValueError if pynode is not a node capsule
 *  Raises RuntimeError  if there is an rcl error
 *
 * \param[in] module the Python module this function is part of
 * \param[in] args arguments tuple, composed by only one argument:
 *  - node: Capsule pointing to the node
 * \param[in] get_enclaves specifies if the output includes the enclaves names or not
 * \return Python list of tuples, containing:
 *  node name, node namespace, and
 *  enclave if `get_enclaves` is true.
 */
static PyObject *
rclpy_get_node_names_impl(PyObject * module, PyObject * args, bool get_enclaves)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "O", &pynode)) {
    return NULL;
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_node_t * node = rclpy_handle_get_pointer_from_capsule(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }
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
  if (ret != RCL_RET_OK) {
    PyErr_Format(
      module_state->RCLError, "Failed to get node names: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  rcutils_ret_t fini_names_ret;
  rcutils_ret_t fini_namespaces_ret;
  rcutils_ret_t fini_enclaves_ret;
  PyObject * pynode_names_and_namespaces = PyList_New(node_names.size);
  if (!pynode_names_and_namespaces) {
    goto cleanup;
  }

  size_t idx;
  for (idx = 0; idx < node_names.size; ++idx) {
    PyObject * pytuple = PyTuple_New(get_enclaves ? 3 : 2);
    if (!pytuple) {
      goto cleanup;
    }
    PyObject * pynode_name = PyUnicode_FromString(node_names.data[idx]);
    if (!pynode_name) {
      Py_DECREF(pytuple);
      goto cleanup;
    }
    // Steals the reference
    PyTuple_SET_ITEM(pytuple, 0, pynode_name);
    PyObject * pynode_namespace = PyUnicode_FromString(node_namespaces.data[idx]);
    if (!pynode_namespace) {
      Py_DECREF(pytuple);
      goto cleanup;
    }
    // Steals the reference
    PyTuple_SET_ITEM(pytuple, 1, pynode_namespace);
    if (get_enclaves) {
      PyObject * pynode_enclaves = PyUnicode_FromString(enclaves.data[idx]);
      if (!pynode_enclaves) {
        Py_DECREF(pytuple);
        goto cleanup;
      }
      // Steals the reference
      PyTuple_SET_ITEM(pytuple, 2, pynode_enclaves);
    }
    // Steals the reference
    PyList_SET_ITEM(pynode_names_and_namespaces, idx, pytuple);
  }

cleanup:
  fini_names_ret = rcutils_string_array_fini(&node_names);
  fini_namespaces_ret = rcutils_string_array_fini(&node_namespaces);
  fini_enclaves_ret = rcutils_string_array_fini(&enclaves);
  if (PyErr_Occurred()) {
    Py_XDECREF(pynode_names_and_namespaces);
    return NULL;
  }
  if (fini_names_ret != RCUTILS_RET_OK) {
    PyErr_Format(
      module_state->RCLError,
      "Failed to destroy node_names: %s", rcl_get_error_string().str);
    Py_XDECREF(pynode_names_and_namespaces);
    rcl_reset_error();
    return NULL;
  }
  if (fini_namespaces_ret != RCUTILS_RET_OK) {
    PyErr_Format(
      module_state->RCLError,
      "Failed to destroy node_namespaces: %s", rcl_get_error_string().str);
    Py_XDECREF(pynode_names_and_namespaces);
    rcl_reset_error();
    return NULL;
  }
  if (fini_enclaves_ret != RCUTILS_RET_OK) {
    PyErr_Format(
      module_state->RCLError,
      "Failed to destroy enclaves string array: %s", rcl_get_error_string().str);
    Py_XDECREF(pynode_names_and_namespaces);
    rcl_reset_error();
    return NULL;
  }

  return pynode_names_and_namespaces;
}

/// Get the list of nodes discovered by the provided node
/**
 *  Raises ValueError if pynode is not a node capsule
 *  Raises RuntimeError  if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node
 * \return Python list of tuples where each tuple contains the two strings:
 *   the node name and node namespace
 */
static PyObject *
rclpy_get_node_names_and_namespaces(PyObject * module, PyObject * args)
{
  return rclpy_get_node_names_impl(module, args, false);
}

/// Get the list of nodes discovered by the provided node, with their respective enclaves.
/**
 *  Raises ValueError if pynode is not a node capsule
 *  Raises RuntimeError  if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node
 * \return Python list of tuples where each tuple contains three strings:
 *   node name, node namespace, and enclave.
 */
static PyObject *
rclpy_get_node_names_and_namespaces_with_enclaves(
  PyObject * module, PyObject * args)
{
  return rclpy_get_node_names_impl(module, args, true);
}
}  // namespace rclpy
