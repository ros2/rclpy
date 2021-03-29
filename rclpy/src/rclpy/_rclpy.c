// Copyright 2016-2020 Open Source Robotics Foundation, Inc.
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

// Must be before Python.h; makes #-formats use ssize_t instead of int
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <rcl/error_handling.h>
#include <rcl/expand_topic_name.h>
#include <rcl/graph.h>
#include <rcl/logging.h>
#include <rcl/node.h>
#include <rcl/publisher.h>
#include <rcl/rcl.h>
#include <rcl/remap.h>
#include <rcl/time.h>
#include <rcl/validate_topic_name.h>
#include <rcl/init_options.h>
#include <rcl/context.h>
#include <rcl_interfaces/msg/parameter_type.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcutils/allocator.h>
#include <rcutils/format_string.h>
#include <rcutils/macros.h>
#include <rcutils/strdup.h>
#include <rcutils/types.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rmw/topic_endpoint_info_array.h>
#include <rmw/types.h>
#include <rmw/validate_full_topic_name.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"

typedef struct
{
  PyObject * NodeNameNonExistentError;
  PyObject * RCLError;
  PyObject * RCLInvalidROSArgsError;
  PyObject * UnknownROSArgsError;
  PyObject * UnsupportedEventTypeError;
} rclpy_module_state_t;

/// Define the public methods of this module
static PyMethodDef rclpy_methods[] = {
  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rclpy__doc__, "ROS 2 Python client library.");

// Initialization function called after __spec__ and __package__ are set by loader
// Purpose: relative imports work in this function
static int rclpy_exec(PyObject * m)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(m);
  if (!module_state) {
    // exception already raised
    return -1;
  }

  PyObject * globals = PyModule_GetDict(m);
  if (NULL == globals) {
    return -1;
  }

  PyObject * exceptions_module = PyImport_ImportModuleLevel(
    "_rclpy_pybind11", globals, NULL, NULL, 1);
  if (NULL == exceptions_module) {
    return -1;
  }

  module_state->RCLError =
    PyObject_GetAttrString(exceptions_module, "RCLError");
  if (NULL == module_state->RCLError) {
    Py_DECREF(exceptions_module);
    return -1;
  }

  module_state->RCLInvalidROSArgsError =
    PyObject_GetAttrString(exceptions_module, "RCLInvalidROSArgsError");
  if (NULL == module_state->RCLInvalidROSArgsError) {
    Py_DECREF(module_state->RCLError);
    Py_DECREF(exceptions_module);
    return -1;
  }

  module_state->UnknownROSArgsError =
    PyObject_GetAttrString(exceptions_module, "UnknownROSArgsError");
  if (NULL == module_state->UnknownROSArgsError) {
    Py_DECREF(module_state->RCLInvalidROSArgsError);
    Py_DECREF(module_state->RCLError);
    Py_DECREF(exceptions_module);
    return -1;
  }

  module_state->NodeNameNonExistentError =
    PyObject_GetAttrString(exceptions_module, "NodeNameNonExistentError");
  if (NULL == module_state->NodeNameNonExistentError) {
    Py_DECREF(module_state->UnknownROSArgsError);
    Py_DECREF(module_state->RCLInvalidROSArgsError);
    Py_DECREF(module_state->RCLError);
    Py_DECREF(exceptions_module);
    return -1;
  }

  module_state->UnsupportedEventTypeError =
    PyObject_GetAttrString(exceptions_module, "UnsupportedEventTypeError");
  if (NULL == module_state->UnsupportedEventTypeError) {
    Py_DECREF(module_state->NodeNameNonExistentError);
    Py_DECREF(module_state->UnknownROSArgsError);
    Py_DECREF(module_state->RCLInvalidROSArgsError);
    Py_DECREF(module_state->RCLError);
    Py_DECREF(exceptions_module);
    return -1;
  }

  Py_DECREF(exceptions_module);

  // 0 is success
  return 0;
}

static int rclpy_traverse(PyObject * m, visitproc visit, void * arg)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(m);
  if (!module_state) {
    return -1;
  }
  Py_VISIT(module_state->NodeNameNonExistentError);
  Py_VISIT(module_state->UnknownROSArgsError);
  Py_VISIT(module_state->UnsupportedEventTypeError);
  Py_VISIT(module_state->RCLInvalidROSArgsError);
  Py_VISIT(module_state->RCLError);
  return 0;
}

static int rclpy_clear(PyObject * m)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(m);
  if (!module_state) {
    return -1;
  }
  Py_CLEAR(module_state->NodeNameNonExistentError);
  Py_CLEAR(module_state->UnknownROSArgsError);
  Py_CLEAR(module_state->UnsupportedEventTypeError);
  Py_CLEAR(module_state->RCLInvalidROSArgsError);
  Py_CLEAR(module_state->RCLError);
  return 0;
}

static PyModuleDef_Slot rclpy_slots[] = {
  {Py_mod_exec, rclpy_exec},
  {0, NULL}
};

/// Define the Python module
static struct PyModuleDef _rclpymodule = {
  PyModuleDef_HEAD_INIT,
  "_rclpy",
  rclpy__doc__,
  sizeof(rclpy_module_state_t),
  rclpy_methods,
  rclpy_slots,
  rclpy_traverse,
  rclpy_clear,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy(void)
{
  return PyModuleDef_Init(&_rclpymodule);
}
