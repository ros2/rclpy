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

#include "./detail/thread_safe_logging_output_handler.h"
#include "./detail/execute_with_logging_mutex.h"
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


/// Deallocate a list of allocated strings.
void
_rclpy_arg_list_fini(int num_args, char ** argv)
{
  if (!argv) {
    return;
  }
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // Free each arg individually
  for (int i = 0; i < num_args; ++i) {
    char * arg = argv[i];
    if (!arg) {
      // NULL in list means array was partially initialized when an error occurred
      break;
    }
    allocator.deallocate(arg, allocator.state);
  }
  // Then free the whole array
  allocator.deallocate(argv, allocator.state);
}

/// Copy a sequence of strings into a c-style array
/* Raises OverflowError if there are too many arguments in pyargs
 * Raises MemoryError if an allocation fails
 * \param[in] pyargs a sequence of strings to be converted
 * \param[out] num_args the number of args in the output, equal to len(pyargs)
 * \param[out] arg_values a c-array of c-style strings
 */
rcl_ret_t
_rclpy_pyargs_to_list(PyObject * pyargs, int * num_args, char *** arg_values)
{
  // Convert to list() in case pyargs is a generator
  pyargs = PySequence_List(pyargs);
  if (!pyargs) {
    // Exception raised
    return RCL_RET_ERROR;
  }
  Py_ssize_t pysize_num_args = PyList_Size(pyargs);
  if (pysize_num_args > INT_MAX) {
    PyErr_Format(PyExc_OverflowError, "Too many arguments");
    Py_DECREF(pyargs);
    return RCL_RET_ERROR;
  }
  *num_args = (int)pysize_num_args;
  *arg_values = NULL;

  rcl_allocator_t allocator = rcl_get_default_allocator();
  if (*num_args > 0) {
    *arg_values = allocator.allocate(sizeof(char *) * (*num_args), allocator.state);
    if (!*arg_values) {
      PyErr_Format(PyExc_MemoryError, "Failed to allocate space for arguments");
      Py_DECREF(pyargs);
      return RCL_RET_BAD_ALLOC;
    }

    for (int i = 0; i < *num_args; ++i) {
      // Returns borrowed reference, do not decref
      PyObject * pyarg = PyList_GetItem(pyargs, i);
      if (!pyarg) {
        _rclpy_arg_list_fini(i, *arg_values);
        Py_DECREF(pyargs);
        // Exception raised
        return RCL_RET_ERROR;
      }
      const char * arg_str = PyUnicode_AsUTF8(pyarg);
      (*arg_values)[i] = rcutils_strdup(arg_str, allocator);
      if (!(*arg_values)[i]) {
        _rclpy_arg_list_fini(i, *arg_values);
        PyErr_Format(PyExc_MemoryError, "Failed to duplicate string");
        Py_DECREF(pyargs);
        return RCL_RET_BAD_ALLOC;
      }
    }
  }
  Py_DECREF(pyargs);
  return RCL_RET_OK;
}

/// Raise an UnknownROSArgsError exception
/* \param[in] module_state the module state for the _rclpy module
 * \param[in] pyargs a sequence of string args
 * \param[in] unknown_ros_args_count the number of unknown ROS args
 * \param[in] unknown_ros_args_indices the indices to unknown ROS args
 */
void _rclpy_raise_unknown_ros_args(
  rclpy_module_state_t * module_state,
  PyObject * pyargs,
  const int * unknown_ros_args_indices,
  int unknown_ros_args_count)
{
  PyObject * unknown_ros_pyargs = NULL;
  if (!module_state) {
    PyErr_Format(PyExc_RuntimeError, "_rclpy_raise_unknown_ros_args got NULL module state");
    goto cleanup;
  }

  pyargs = PySequence_List(pyargs);
  if (NULL == pyargs) {
    goto cleanup;
  }

  unknown_ros_pyargs = PyList_New(0);
  if (NULL == unknown_ros_pyargs) {
    goto cleanup;
  }

  for (int i = 0; i < unknown_ros_args_count; ++i) {
    PyObject * ros_pyarg = PyList_GetItem(
      pyargs, (Py_ssize_t)unknown_ros_args_indices[i]);
    if (NULL == ros_pyarg) {
      goto cleanup;
    }
    if (PyList_Append(unknown_ros_pyargs, ros_pyarg) != 0) {
      goto cleanup;
    }
  }

  PyErr_Format(
    module_state->UnknownROSArgsError,
    "Found unknown ROS arguments: %R",
    unknown_ros_pyargs);
cleanup:
  Py_XDECREF(unknown_ros_pyargs);
  Py_XDECREF(pyargs);
}

/// Parse a sequence of strings into rcl_arguments_t struct
/* Raises TypeError of pyargs is not a sequence
 * Raises OverflowError if len(pyargs) > INT_MAX
 * \param[in] module_state the module state for the _rclpy module
 * \param[in] pyargs a Python sequence of strings
 * \param[out] parsed_args a zero initialized pointer to rcl_arguments_t
 */
rcl_ret_t
_rclpy_parse_args(
  rclpy_module_state_t * module_state, PyObject * pyargs, rcl_arguments_t * parsed_args)
{
  if (!module_state) {
    PyErr_Format(PyExc_RuntimeError, "_rclpy_parse_args got NULL module state");
    return RCL_RET_ERROR;
  }
  rcl_ret_t ret;

  int num_args = 0;
  char ** arg_values = NULL;
  // Py_None is a singleton so comparing pointer value works
  if (Py_None != pyargs) {
    ret = _rclpy_pyargs_to_list(pyargs, &num_args, &arg_values);
    if (RCL_RET_OK != ret) {
      // Exception set
      return ret;
    }
  }
  // call rcl_parse_arguments() even if pyargs is None so rcl_arguments_t structure is always valid.
  // Otherwise the remapping functions will error if the user passes no arguments to a node and sets
  // use_global_arguments to False.

  rcl_allocator_t allocator = rcl_get_default_allocator();
  // Adding const via cast to eliminate warning about incompatible pointer type
  const char ** const_arg_values = (const char **)arg_values;
  ret = rcl_parse_arguments(num_args, const_arg_values, allocator, parsed_args);

  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_INVALID_ROS_ARGS) {
      PyErr_Format(
        module_state->RCLInvalidROSArgsError,
        "Failed to parse ROS arguments: %s",
        rcl_get_error_string().str);
    } else {
      PyErr_Format(
        module_state->RCLError,
        "Failed to init: %s",
        rcl_get_error_string().str);
    }
    rcl_reset_error();
    goto cleanup;
  }

  int unparsed_ros_args_count = rcl_arguments_get_count_unparsed_ros(parsed_args);
  if (unparsed_ros_args_count > 0) {
    int * unparsed_ros_args_indices = NULL;
    ret = rcl_arguments_get_unparsed_ros(
      parsed_args, allocator, &unparsed_ros_args_indices);
    if (RCL_RET_OK != ret) {
      PyErr_Format(
        module_state->RCLError,
        "Failed to get unparsed ROS arguments: %s",
        rcl_get_error_string().str);
      rcl_reset_error();
      goto cleanup;
    }
    _rclpy_raise_unknown_ros_args(
      module_state, pyargs, unparsed_ros_args_indices, unparsed_ros_args_count);
    allocator.deallocate(unparsed_ros_args_indices, allocator.state);
    ret = RCL_RET_ERROR;
  }
cleanup:
  _rclpy_arg_list_fini(num_args, arg_values);
  return ret;
}

int pyobj_to_long(PyObject * obj, void * i)
{
  PY_LONG_LONG tmp;
  PY_LONG_LONG * val = (PY_LONG_LONG *)i;

  if (obj == Py_None) {
    return 1;  // Return success if object is None
  }

  if (PyLong_Check(obj)) {
    tmp = PyLong_AsLongLong(obj);
    if (PyErr_Occurred()) {
      return 0;  // Conversion failed.
    } else {
      *val = tmp;
      return 1;  // Successful conversion.
    }
  }

  PyErr_SetString(PyExc_TypeError, "PyObject must be long or None.");
  return 0;  // Conversion failed
}

/// Initialize rcl logging
/**
 * Raises RuntimeError if rcl logging could not be initialized
 */
static PyObject *
rclpy_logging_configure_impl(PyObject * module, PyObject * args)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }
  // Expect one argument, a context.
  PyObject * pycontext;
  if (!PyArg_ParseTuple(args, "O", &pycontext)) {
    // Exception raised
    return NULL;
  }
  rcl_context_t * context = rclpy_handle_get_pointer_from_capsule(pycontext, "rcl_context_t");
  if (!context) {
    return NULL;
  }
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_logging_configure_with_output_handler(
    &context->global_arguments,
    &allocator,
    rclpy_detail_thread_safe_logging_output_handler);
  if (RCL_RET_OK != ret) {
    PyErr_Format(
      module_state->RCLError,
      "Failed to initialize logging: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// See rclpy_logging_configure_impl above.
static PyObject *
rclpy_logging_configure(PyObject * self, PyObject * args)
{
  return rclpy_detail_execute_with_logging_mutex(rclpy_logging_configure_impl, self, args);
}

/// Finalize rcl logging
/**
 * Produces a RuntimeWarning if rcl logging could not be finalized
 */
static PyObject *
rclpy_logging_fini_impl(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcl_ret_t ret = rcl_logging_fini();
  if (RCL_RET_OK != ret) {
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning,
      stack_level,
      "Failed to fini logging: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// See rclpy_logging_fini_impl above.
static PyObject *
rclpy_logging_fini(PyObject * self, PyObject * args)
{
  return rclpy_detail_execute_with_logging_mutex(rclpy_logging_fini_impl, self, args);
}

/// Handle destructor for node
static void
_rclpy_destroy_node_impl(void * p)
{
  rcl_node_t * node = p;
  if (!node) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_node got a NULL pointer");
    return;
  }

  rcl_ret_t ret = rcl_node_fini(node);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini node: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(node);
}

/// See _rclpy_destroy_node_impl above.
static void
_rclpy_destroy_node(void * p)
{
  rclpy_detail_execute_with_logging_mutex2(_rclpy_destroy_node_impl, p);
}

/// Create a node
/**
 * Raises ValueError if the node name or namespace is invalid
 * Raises RuntimeError if the node could not be initialized for an unexpected reason
 * Raises MemoryError if memory could not be allocated for the node
 *
 * \param[in] node_name string name of the node to be created
 * \param[in] namespace string namespace for the node
 * \return Capsule of the pointer to the created rcl_node_t * structure, or
 * \return NULL on failure
 */
static PyObject *
rclpy_create_node_impl(PyObject * module, PyObject * args)
{
  rcl_ret_t ret;
  const char * node_name;
  const char * namespace_;
  PyObject * pycontext;
  PyObject * py_cli_args;
  int use_global_arguments;
  int enable_rosout;

  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }

  if (!PyArg_ParseTuple(
      args, "ssOOpp",
      &node_name,
      &namespace_,
      &pycontext,
      &py_cli_args,
      &use_global_arguments,
      &enable_rosout))
  {
    return NULL;
  }

  rclpy_handle_t * context_handle = PyCapsule_GetPointer(pycontext, "rcl_context_t");
  if (!context_handle) {
    return NULL;
  }
  rcl_context_t * context = _rclpy_handle_get_pointer(context_handle);
  if (!context) {
    return NULL;
  }

  rcl_arguments_t arguments = rcl_get_zero_initialized_arguments();
  ret = _rclpy_parse_args(module_state, py_cli_args, &arguments);
  if (RCL_RET_OK != ret) {
    // exception set
    return NULL;
  }

  rcl_node_t * node = PyMem_Malloc(sizeof(rcl_node_t));
  if (!node) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for node");
    return NULL;
  }
  *node = rcl_get_zero_initialized_node();
  rcl_node_options_t options = rcl_node_get_default_options();
  options.use_global_arguments = use_global_arguments;
  options.arguments = arguments;
  options.enable_rosout = enable_rosout;
  ret = rcl_node_init(node, node_name, namespace_, context, &options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_BAD_ALLOC) {
      PyErr_Format(
        PyExc_MemoryError, "%s", rcl_get_error_string().str);
    } else if (ret == RCL_RET_NODE_INVALID_NAME) {
      PyErr_Format(
        PyExc_ValueError, "invalid node name: %s", rcl_get_error_string().str);
    } else if (ret == RCL_RET_NODE_INVALID_NAMESPACE) {
      PyErr_Format(
        PyExc_ValueError,
        "invalid node namespace: %s", rcl_get_error_string().str);
    } else {
      PyErr_Format(
        module_state->RCLError,
        "Unknown error creating node: %s", rcl_get_error_string().str);
    }
    rcl_reset_error();
    PyMem_Free(node);

    if (RCL_RET_OK != rcl_arguments_fini(&arguments)) {
      // Warn because an exception is already raised
      // Warning should use line number of the current stack frame
      int stack_level = 1;
      PyErr_WarnFormat(
        PyExc_RuntimeWarning, stack_level, "Failed to fini arguments during error handling: %s",
        rcl_get_error_string().str);
      rcl_reset_error();
    }
    return NULL;
  }
  if (RCL_RET_OK != rcl_arguments_fini(&arguments)) {
    // Warn because the node was successfully created
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini arguments: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
  }

  rclpy_handle_t * node_handle = _rclpy_create_handle(node, _rclpy_destroy_node);
  if (!node_handle) {
    _rclpy_destroy_node(node);
    return NULL;
  }
  _rclpy_handle_add_dependency(node_handle, context_handle);
  if (PyErr_Occurred()) {
    _rclpy_handle_dec_ref(node_handle);
    return NULL;
  }
  PyObject * node_capsule = _rclpy_create_handle_capsule(node_handle, "rcl_node_t");
  if (!node_capsule) {
    _rclpy_handle_dec_ref(node_handle);
    return NULL;
  }
  return node_capsule;
}

/// See rclpy_create_node_impl above.
static PyObject *
rclpy_create_node(PyObject * self, PyObject * args)
{
  return rclpy_detail_execute_with_logging_mutex(rclpy_create_node_impl, self, args);
}

/// Define the public methods of this module
static PyMethodDef rclpy_methods[] = {
  {
    "rclpy_logging_configure", rclpy_logging_configure, METH_VARARGS,
    "Initialize RCL logging."
  },
  {
    "rclpy_logging_fini", rclpy_logging_fini, METH_NOARGS,
    "Finalize RCL logging."
  },
  {
    "rclpy_create_node", rclpy_create_node, METH_VARARGS,
    "Create a Node."
  },

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
