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

#include "./_rclpy_qos_event.c"


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

static PyObject *
rclpy_remove_ros_args(PyObject * module, PyObject * args)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }

  // Expect one argument which is a list of strings
  PyObject * pyargs;
  if (!PyArg_ParseTuple(args, "O", &pyargs)) {
    // Exception raised
    return NULL;
  }

  rcl_ret_t ret;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  int num_args;
  char ** arg_values;
  ret = _rclpy_pyargs_to_list(pyargs, &num_args, &arg_values);
  if (RCL_RET_OK != ret) {
    // Exception set
    return NULL;
  }

  // Adding const via cast to eliminate warning about incompatible pointer type
  const char ** const_arg_values = (const char **)arg_values;
  rcl_arguments_t parsed_args = rcl_get_zero_initialized_arguments();

  ret = rcl_parse_arguments(num_args, const_arg_values, allocator, &parsed_args);
  if (ret != RCL_RET_OK) {
    PyErr_Format(module_state->RCLError, "Failed to init: %s", rcl_get_error_string().str);
    rcl_reset_error();
    _rclpy_arg_list_fini(num_args, arg_values);
    return NULL;
  }

  int nonros_argc = 0;
  const char ** nonros_argv = NULL;

  ret = rcl_remove_ros_arguments(
    const_arg_values,
    &parsed_args,
    allocator,
    &nonros_argc,
    &nonros_argv);
  if (RCL_RET_OK != ret) {
    PyErr_Format(module_state->RCLError, "Failed to init: %s", rcl_get_error_string().str);
    rcl_reset_error();
    _rclpy_arg_list_fini(num_args, arg_values);
    return NULL;
  }

  PyObject * pyresult_list = PyList_New(nonros_argc);
  if (!pyresult_list) {
    goto cleanup;
  }
  for (int ii = 0; ii < nonros_argc; ++ii) {
    PyObject * pynonros_argv_string = PyUnicode_FromString(nonros_argv[ii]);
    if (!pynonros_argv_string) {
      Py_DECREF(pyresult_list);
      goto cleanup;
    }
    // Steals the reference to the item, so it will be destroyed with the list
    PyList_SET_ITEM(pyresult_list, ii, pynonros_argv_string);
  }

cleanup:
/* it was determined that the following warning is likely a front-end parsing issue in MSVC.
 * See: https://github.com/ros2/rclpy/pull/180#issuecomment-375452757
 */
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4090)
#endif
  allocator.deallocate(nonros_argv, allocator.state);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

  _rclpy_arg_list_fini(num_args, arg_values);

  ret = rcl_arguments_fini(&parsed_args);
  if (PyErr_Occurred()) {
    return NULL;
  }
  if (RCL_RET_OK != ret) {
    PyErr_Format(module_state->RCLError, "Failed to init: %s", rcl_get_error_string().str);
    rcl_reset_error();
    Py_XDECREF(pyresult_list);
    return NULL;
  }

  return pyresult_list;
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

/// Initialize rcl with default options, ignoring parameters
/**
 * Raises RuntimeError if rcl could not be initialized
 */
static PyObject *
rclpy_init(PyObject * module, PyObject * args)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }
  // Expect two arguments, one is a list of strings and the other is a context.
  PyObject * pyargs;
  PyObject * pyseqlist;
  PyObject * pycontext;
  PY_LONG_LONG domain_id = (PY_LONG_LONG) RCL_DEFAULT_DOMAIN_ID;

  if (!PyArg_ParseTuple(args, "OO|O&", &pyargs, &pycontext, pyobj_to_long, (void *)&domain_id)) {
    // Exception raised
    return NULL;
  }

  if (domain_id != (PY_LONG_LONG)RCL_DEFAULT_DOMAIN_ID && domain_id < 0) {
    PyErr_Format(
      PyExc_RuntimeError,
      "Domain id (%ll) should not be lower than zero.", domain_id);
    return NULL;
  }

  pyseqlist = PySequence_List(pyargs);
  if (!pyseqlist) {
    // Exception raised
    return NULL;
  }
  Py_ssize_t pysize_num_args = PyList_Size(pyseqlist);
  if (pysize_num_args > INT_MAX) {
    PyErr_Format(PyExc_OverflowError, "Too many arguments");
    Py_DECREF(pyseqlist);
    return NULL;
  }
  int num_args = (int)pysize_num_args;

  rcl_context_t * context = rclpy_handle_get_pointer_from_capsule(pycontext, "rcl_context_t");
  if (!context) {
    Py_DECREF(pyseqlist);
    return NULL;
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();
  const char ** arg_values = NULL;
  bool have_args = true;
  if (num_args > 0) {
    arg_values = allocator.allocate(sizeof(char *) * num_args, allocator.state);
    if (!arg_values) {
      PyErr_Format(PyExc_MemoryError, "Failed to allocate space for arguments");
      Py_DECREF(pyseqlist);
      return NULL;
    }

    for (int i = 0; i < num_args; ++i) {
      // Returns borrowed reference, do not decref
      PyObject * pyarg = PyList_GetItem(pyseqlist, i);
      if (!pyarg) {
        have_args = false;
        break;
      }
      // Borrows a pointer, do not free arg_values[i]
      arg_values[i] = PyUnicode_AsUTF8(pyarg);
      if (!arg_values[i]) {
        have_args = false;
        break;
      }
    }
  }

  if (have_args) {
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
    if (RCL_RET_OK == ret) {
      // Set domain id
      ret = rcl_init_options_set_domain_id(&init_options, (size_t)domain_id);
      if (RCL_RET_OK != ret) {
        PyErr_Format(
          PyExc_RuntimeError,
          "Failed to set domain id to init_options: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      ret = rcl_init(num_args, arg_values, &init_options, context);
      if (RCL_RET_OK == ret) {
        int unparsed_ros_args_count =
          rcl_arguments_get_count_unparsed_ros(&context->global_arguments);
        if (unparsed_ros_args_count > 0) {
          int * unparsed_ros_args_indices = NULL;
          ret = rcl_arguments_get_unparsed_ros(
            &context->global_arguments, allocator, &unparsed_ros_args_indices);
          if (RCL_RET_OK == ret) {
            _rclpy_raise_unknown_ros_args(
              module_state,
              pyargs, unparsed_ros_args_indices, unparsed_ros_args_count);
            allocator.deallocate(unparsed_ros_args_indices, allocator.state);
          } else {
            PyErr_Format(
              PyExc_RuntimeError,
              "Failed to get unparsed ROS arguments: %s",
              rcl_get_error_string().str);
            rcl_reset_error();
          }
        }
      } else {
        PyErr_Format(PyExc_RuntimeError, "Failed to init: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
    } else {
      PyErr_Format(
        PyExc_RuntimeError, "Failed to initialize init_options: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
  }
  if (NULL != arg_values) {
/* it was determined that the following warning is likely a front-end parsing issue in MSVC.
 * See: https://github.com/ros2/rclpy/pull/180#issuecomment-375452757
 */
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4090)
#endif
    allocator.deallocate(arg_values, allocator.state);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
  }
  Py_DECREF(pyseqlist);

  if (PyErr_Occurred()) {
    return NULL;
  }
  Py_RETURN_NONE;
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

/// Get the name of the logger associated with the node of the publisher.
/**
 * Raises ValueError if pypublisher is not a publisher capsule
 *
 * \param[in] pypublisher Capsule pointing to the publisher to get the logger name of
 * \return logger_name, or
 * \return None on failure
 */
static PyObject *
rclpy_get_publisher_logger_name(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pypublisher;
  if (!PyArg_ParseTuple(args, "O", &pypublisher)) {
    return NULL;
  }

  rclpy_publisher_t * pub = rclpy_handle_get_pointer_from_capsule(pypublisher, "rclpy_publisher_t");
  if (NULL == pub) {
    return NULL;
  }

  const char * node_logger_name = rcl_node_get_logger_name(pub->node);
  if (NULL == node_logger_name) {
    Py_RETURN_NONE;
  }

  return PyUnicode_FromString(node_logger_name);
}

typedef rcl_ret_t (* count_func)(const rcl_node_t * node, const char * topic_name, size_t * count);

static PyObject *
_count_subscribers_publishers(
  PyObject * module, PyObject * args, const char * type, count_func count_function)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }
  PyObject * pynode;
  const char * topic_name;

  if (!PyArg_ParseTuple(args, "Os", &pynode, &topic_name)) {
    return NULL;
  }

  rcl_node_t * node = rclpy_handle_get_pointer_from_capsule(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  size_t count = 0;
  rcl_ret_t ret = count_function(node, topic_name, &count);
  if (ret != RCL_RET_OK) {
    PyErr_Format(
      module_state->RCLError, "Failed to count %s: %s", type, rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  return PyLong_FromSize_t(count);
}

/// Count publishers for a topic.
/**
 *
 * \param[in] pynode Capsule pointing to the node to get the namespace from
 * \param[in] topic_name string fully qualified topic name
 * \return count of publishers
 */
static PyObject *
rclpy_count_publishers(PyObject * module, PyObject * args)
{
  return _count_subscribers_publishers(module, args, "publishers", rcl_count_publishers);
}

/// Count subscribers for a topic.
/**
 *
 * \param[in] pynode Capsule pointing to the node to get the namespace from
 * \param[in] topic_name string fully qualified topic name
 * \return count of subscribers
 */
static PyObject *
rclpy_count_subscribers(PyObject * module, PyObject * args)
{
  return _count_subscribers_publishers(module, args, "subscribers", rcl_count_subscribers);
}

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
rclpy_get_publishers_info_by_topic(PyObject * module, PyObject * args)
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
rclpy_get_subscriptions_info_by_topic(PyObject * module, PyObject * args)
{
  return _get_info_by_topic(module, args, "subscriptions", rcl_get_subscriptions_info_by_topic);
}

/// Return the identifier of the current rmw_implementation
/**
 * \return string containing the identifier of the current rmw_implementation
 */
static PyObject *
rclpy_get_rmw_implementation_identifier(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  const char * rmw_implementation_identifier = rmw_get_implementation_identifier();

  PyObject * pyrmw_implementation_identifier = Py_BuildValue(
    "s", rmw_implementation_identifier);

  return pyrmw_implementation_identifier;
}
/// Take a raw message from a given subscription (internal- for rclpy_take with raw=True)
/**
 * \param[in] module_state the module state for the _rclpy module
 * \param[in] rcl subscription pointer pointing to the subscription to process the message
 * \param[in] message_info struct pointer, may be null. if non-null, will be filled with message info
 * \return Python byte array with the raw serialized message contents
 */
static PyObject *
rclpy_take_raw_with_info(
  rclpy_module_state_t * module_state,
  rcl_subscription_t * subscription, rmw_message_info_t * message_info)
{
  if (!module_state) {
    PyErr_Format(PyExc_RuntimeError, "rclpy_take_raw_with_info got NULL module state");
    return NULL;
  }
  // Create a serialized message object
  rcl_serialized_message_t msg = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_ret_t ret = rmw_serialized_message_init(&msg, 0u, &allocator);
  if (ret != RCL_RET_OK) {
    PyErr_Format(
      module_state->RCLError,
      "Failed to initialize message: %s", rcl_get_error_string().str);
    rcl_reset_error();
    rmw_ret_t r_fini = rmw_serialized_message_fini(&msg);
    if (r_fini != RMW_RET_OK) {
      PyErr_Format(module_state->RCLError, "Failed to deallocate message buffer: %d", r_fini);
    }
    return NULL;
  }

  ret = rcl_take_serialized_message(subscription, &msg, message_info, NULL);
  if (ret != RCL_RET_OK) {
    PyErr_Format(
      module_state->RCLError,
      "Failed to take_serialized from a subscription: %s", rcl_get_error_string().str);
    rcl_reset_error();
    rmw_ret_t r_fini = rmw_serialized_message_fini(&msg);
    if (r_fini != RMW_RET_OK) {
      PyErr_Format(module_state->RCLError, "Failed to deallocate message buffer: %d", r_fini);
    }
    return NULL;
  }
  PyObject * python_bytes = PyBytes_FromStringAndSize((char *)(msg.buffer), msg.buffer_length);
  rmw_ret_t r_fini = rmw_serialized_message_fini(&msg);
  if (r_fini != RMW_RET_OK) {
    PyErr_Format(module_state->RCLError, "Failed to deallocate message buffer: %d", r_fini);
    if (python_bytes) {
      Py_DECREF(python_bytes);
    }
    return NULL;
  }
  return python_bytes;
}

static PyObject *
rclpy_message_info_to_dict(rmw_message_info_t * message_info)
{
  PyObject * dict = Py_BuildValue(
    "{s:L,s:L}",
    "source_timestamp", message_info->source_timestamp,
    "received_timestamp", message_info->received_timestamp);
  if (dict == NULL) {
    PyErr_Format(PyExc_RuntimeError, "Failed to create dictionary object for message info");
    return NULL;
  }
  return dict;
}

/// Take a message and its message_info from a given subscription
/**
 * \param[in] pysubscription Capsule pointing to the subscription to process the message
 * \param[in] pymsg_type Instance of the message type to take
 * \return Tuple of (Python message with all fields populated with received message, message_info),
 *         or None if there was no message to take.
 */
static PyObject *
rclpy_take(PyObject * module, PyObject * args)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }
  PyObject * pysubscription;
  PyObject * pymsg_type;
  PyObject * pyraw;
  PyObject * pytaken_msg = NULL;

  if (!PyArg_ParseTuple(args, "OOO", &pysubscription, &pymsg_type, &pyraw)) {
    return NULL;
  }
  if (!PyCapsule_CheckExact(pysubscription)) {
    PyErr_Format(PyExc_TypeError, "Argument pysubscription is not a valid PyCapsule");
    return NULL;
  }

  rclpy_subscription_t * sub =
    rclpy_handle_get_pointer_from_capsule(pysubscription, "rclpy_subscription_t");
  if (!sub) {
    return NULL;
  }

  rmw_message_info_t message_info;
  if (PyObject_IsTrue(pyraw) == 1) {  // raw=True
    pytaken_msg = rclpy_take_raw_with_info(module_state, &(sub->subscription), &message_info);
  } else {
    destroy_ros_message_signature * destroy_ros_message = NULL;
    void * taken_msg = rclpy_create_from_py(pymsg_type, &destroy_ros_message);
    if (!taken_msg) {
      return NULL;
    }

    rcl_ret_t ret = rcl_take(&(sub->subscription), taken_msg, &message_info, NULL);

    if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
      PyErr_Format(
        module_state->RCLError,
        "Failed to take from a subscription: %s", rcl_get_error_string().str);
      rcl_reset_error();
      destroy_ros_message(taken_msg);
      return NULL;
    }

    if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
      Py_RETURN_NONE;
    }

    pytaken_msg = rclpy_convert_to_py(taken_msg, pymsg_type);
    destroy_ros_message(taken_msg);
  }
  if (!pytaken_msg) {
    // the function has set the Python error
    return NULL;
  }

  // make result tuple
  PyObject * mi_dict = rclpy_message_info_to_dict(&message_info);
  if (mi_dict == NULL) {
    Py_DECREF(pytaken_msg);
    return NULL;
  }
  PyObject * tuple = PyTuple_Pack(2, pytaken_msg, mi_dict);
  Py_DECREF(pytaken_msg);
  Py_DECREF(mi_dict);
  return tuple;
}

/// Request shutdown of the client library
/**
 * Raises RuntimeError if the library could not be shutdown
 *
 * \return None
 */
static PyObject *
rclpy_shutdown(PyObject * module, PyObject * args)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }
  PyObject * pycontext;

  if (!PyArg_ParseTuple(args, "O", &pycontext)) {
    return NULL;
  }

  rcl_context_t * context = rclpy_handle_get_pointer_from_capsule(pycontext, "rcl_context_t");
  if (!context) {
    return NULL;
  }

  rcl_ret_t ret = rcl_shutdown(context);
  if (ret != RCL_RET_OK) {
    PyErr_Format(
      module_state->RCLError, "Failed to shutdown: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Fill a given rmw_time_t with python Duration info, if possible.
/**
  * \param[in] pyobject Python Object that should be a Duration, error checking is done
  * \param[out] out_time Valid destination for time data
  * \return Whether or not conversion succeeded.
  */
bool
_convert_py_duration_to_rmw_time(PyObject * pyobject, rmw_time_t * out_time)
{
  rcl_duration_t * duration = PyCapsule_GetPointer(pyobject, "rcl_duration_t");
  if (!duration) {
    return false;
  }
  *out_time = (rmw_time_t) {
    RCL_NS_TO_S(duration->nanoseconds),
    duration->nanoseconds % (1000LL * 1000LL * 1000LL)
  };
  return true;
}

/// Destructor for a QoSProfile
void
_rclpy_destroy_qos_profile(PyObject * pycapsule)
{
  PyMem_Free(PyCapsule_GetPointer(pycapsule, "rmw_qos_profile_t"));
}

/// Return a Python QoSProfile object
/**
 * This function creates a QoSProfile object from the QoS Policies provided
 * \param[in] pyqos_history enum of type QoSHistoryPolicy
 * \param[in] pyqos_depth int size of the DDS message queue
 * \param[in] pyqos_reliability enum of type QoSReliabilityPolicy
 * \param[in] pyqos_durability enum of type QoSDurabilityPolicy
 * \return NULL on failure
 *         Capsule to a rmw_qos_profile_t object
 */
static PyObject *
rclpy_convert_from_py_qos_policy(PyObject * Py_UNUSED(self), PyObject * args)
{
  unsigned PY_LONG_LONG pyqos_history;
  unsigned PY_LONG_LONG pyqos_depth;
  unsigned PY_LONG_LONG pyqos_reliability;
  unsigned PY_LONG_LONG pyqos_durability;
  PyObject * pyqos_lifespan;
  PyObject * pyqos_deadline;
  unsigned PY_LONG_LONG pyqos_liveliness;
  PyObject * pyqos_liveliness_lease_duration;
  int avoid_ros_namespace_conventions;

  if (!PyArg_ParseTuple(
      args, "KKKKOOKOp",
      &pyqos_history,
      &pyqos_depth,
      &pyqos_reliability,
      &pyqos_durability,
      &pyqos_lifespan,
      &pyqos_deadline,
      &pyqos_liveliness,
      &pyqos_liveliness_lease_duration,
      &avoid_ros_namespace_conventions))
  {
    return NULL;
  }

  rmw_qos_profile_t * qos_profile = PyMem_Malloc(sizeof(rmw_qos_profile_t));

  if (!qos_profile) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for QoS profile");
    return NULL;
  }
  // Set to default so that we don't use uninitialized data if new fields are added
  *qos_profile = rmw_qos_profile_default;
  // Overwrite defaults with passed values
  qos_profile->history = pyqos_history;
  qos_profile->depth = pyqos_depth;
  qos_profile->reliability = pyqos_reliability;
  qos_profile->durability = pyqos_durability;

  if (!_convert_py_duration_to_rmw_time(pyqos_lifespan, &qos_profile->lifespan)) {
    return NULL;
  }
  if (!_convert_py_duration_to_rmw_time(pyqos_deadline, &qos_profile->deadline)) {
    return NULL;
  }

  qos_profile->liveliness = pyqos_liveliness;
  if (
    !_convert_py_duration_to_rmw_time(
      pyqos_liveliness_lease_duration,
      &qos_profile->liveliness_lease_duration))
  {
    return NULL;
  }

  qos_profile->avoid_ros_namespace_conventions = avoid_ros_namespace_conventions;
  PyObject * pyqos_profile = PyCapsule_New(
    qos_profile, "rmw_qos_profile_t", _rclpy_destroy_qos_profile);
  return pyqos_profile;
}

/// Convert a C rmw_qos_profile_t Capsule to a Python QoSProfile object.
/**
  * This function is exposed to facilitate testing profile type conversion.
  * \param[in] pyqos_profile Capsule of rmw_qos_profile_t from rclpy_convert_from_py_qos_policy
  * \return QoSProfile object
  */
static PyObject *
rclpy_convert_to_py_qos_policy(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyqos_profile = NULL;
  if (!PyArg_ParseTuple(args, "O", &pyqos_profile)) {
    return NULL;
  }

  rmw_qos_profile_t * profile = PyCapsule_GetPointer(
    pyqos_profile, "rmw_qos_profile_t");
  if (!profile) {
    return NULL;
  }
  return rclpy_common_convert_to_qos_dict(profile);
}

/// Fetch a predefined qos_profile from rmw and convert it to a dictionary with QoSProfile args
/**
 * Raises RuntimeError if there is an rcl error
 *
 * This function takes a string defining a rmw_qos_profile_t and returns the
 * corresponding QoSProfile args in a dictionary
 * \param[in] string with the name of the profile to load
 * \return dictionary with QoSProfile args
 */
static PyObject *
rclpy_get_rmw_qos_profile(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * pyrmw_profile;
  if (!PyArg_ParseTuple(
      args, "z", &pyrmw_profile))
  {
    return NULL;
  }
  PyObject * pyqos_profile = NULL;
  if (0 == strcmp(pyrmw_profile, "qos_profile_sensor_data")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_sensor_data);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_default")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_default);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_system_default")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_system_default);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_services_default")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_services_default);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_unknown")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_unknown);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameters")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_parameters);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameter_events")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_parameter_events);
  } else {
    PyErr_Format(
      PyExc_RuntimeError,
      "Requested unknown rmw_qos_profile: '%s'", pyrmw_profile);
    return NULL;
  }
  return pyqos_profile;
}

/// Manually assert that an entity is alive.
/**
  * When using RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC, the application must call this function
  * at least as often as the qos policy liveliness_lease_duration.
  * The passed entity can be a Publisher.
  *
  * Raises RuntimeError on failure to assert liveliness
  * Raises TypeError if passed object is not a valid Publisher
  *
  * \param[in] pyentity A capsule containing an rcl_publisher_t
  * \return None
  */
static PyObject *
rclpy_assert_liveliness(PyObject * module, PyObject * args)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }
  PyObject * pyentity;

  if (!PyArg_ParseTuple(args, "O", &pyentity)) {
    return NULL;
  }

  if (PyCapsule_IsValid(pyentity, "rclpy_publisher_t")) {
    rclpy_publisher_t * publisher = rclpy_handle_get_pointer_from_capsule(
      pyentity, "rclpy_publisher_t");
    if (RCL_RET_OK != rcl_publisher_assert_liveliness(&publisher->publisher)) {
      PyErr_Format(
        module_state->RCLError,
        "Failed to assert liveliness on the Publisher: %s", rcl_get_error_string().str);
      rcl_reset_error();
      return NULL;
    }
  } else {
    PyErr_Format(
      PyExc_TypeError, "Passed capsule is not a valid Publisher.");
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Create an rclpy.parameter.Parameter from an rcl_variant_t
/**
 * On failure a Python exception is raised and NULL is returned if:
 *
 * Raises ValueError if the variant points to no data.
 *
 * \param[in] name The name of the parameter as a Python unicode string.
 * \param[in] variant The variant object to create a Parameter from.
 * \param[in] parameter_cls The PythonObject for the Parameter class.
 * \param[in] parameter_type_cls The PythonObject for the Parameter.Type class.
 *
 * Returns a pointer to an rclpy.parameter.Parameter with the name, type, and value from
 * the variant or NULL when raising a Python exception.
 */
static PyObject * _parameter_from_rcl_variant(
  PyObject * name, rcl_variant_t * variant, PyObject * parameter_cls,
  PyObject * parameter_type_cls)
{
  // Default to NOT_SET and a value of Py_None to suppress warnings.
  // A Python error will raise if type and value don't agree.
  int type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_NOT_SET;
  PyObject * value = Py_None;
  PyObject * member_value;
  if (variant->bool_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_BOOL;
    value = *(variant->bool_value) ? Py_True : Py_False;
    Py_INCREF(value);
  } else if (variant->integer_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER;
    value = PyLong_FromLongLong(*(variant->integer_value));
    if (!value) {
      return NULL;
    }
  } else if (variant->double_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE;
    value = PyFloat_FromDouble(*(variant->double_value));
    if (!value) {
      return NULL;
    }
  } else if (variant->string_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_STRING;
    value = PyUnicode_FromString(variant->string_value);
    if (!value) {
      return NULL;
    }
  } else if (variant->byte_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_BYTE_ARRAY;
    value = PyList_New(variant->byte_array_value->size);
    if (!value) {
      return NULL;
    }
    for (size_t i = 0; i < variant->byte_array_value->size; ++i) {
      member_value = PyBytes_FromFormat("%u", variant->byte_array_value->values[i]);
      if (!member_value) {
        Py_DECREF(value);
        return NULL;
      }
      PyList_SET_ITEM(value, i, member_value);
    }
  } else if (variant->bool_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_BOOL_ARRAY;
    value = PyList_New(variant->bool_array_value->size);
    if (!value) {
      return NULL;
    }
    for (size_t i = 0; i < variant->bool_array_value->size; ++i) {
      member_value = variant->bool_array_value->values[i] ? Py_True : Py_False;
      Py_INCREF(member_value);
      PyList_SET_ITEM(value, i, member_value);
    }
  } else if (variant->integer_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER_ARRAY;
    value = PyList_New(variant->integer_array_value->size);
    if (!value) {
      return NULL;
    }
    for (size_t i = 0; i < variant->integer_array_value->size; ++i) {
      member_value = PyLong_FromLongLong(variant->integer_array_value->values[i]);
      if (!member_value) {
        Py_DECREF(value);
        return NULL;
      }
      PyList_SET_ITEM(value, i, member_value);
    }
  } else if (variant->double_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE_ARRAY;
    value = PyList_New(variant->double_array_value->size);
    if (!value) {
      return NULL;
    }
    for (size_t i = 0; i < variant->double_array_value->size; ++i) {
      member_value = PyFloat_FromDouble(variant->double_array_value->values[i]);
      if (!member_value) {
        Py_DECREF(value);
        return NULL;
      }
      PyList_SET_ITEM(value, i, member_value);
    }
  } else if (variant->string_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_STRING_ARRAY;
    value = PyList_New(variant->string_array_value->size);
    if (!value) {
      return NULL;
    }
    for (size_t i = 0; i < variant->string_array_value->size; ++i) {
      member_value = PyUnicode_FromString(variant->string_array_value->data[i]);
      if (!member_value) {
        Py_DECREF(value);
        return NULL;
      }
      PyList_SET_ITEM(value, i, member_value);
    }
  } else {
    // INCREF Py_None if no other value was set.
    Py_INCREF(value);
  }

  PyObject * args = Py_BuildValue("(i)", type_enum_value);
  if (!args) {
    Py_DECREF(value);
    return NULL;
  }
  PyObject * type = PyObject_CallObject(parameter_type_cls, args);
  Py_DECREF(args);
  args = Py_BuildValue("OOO", name, type, value);
  Py_DECREF(value);
  Py_DECREF(type);
  if (!args) {
    return NULL;
  }

  PyObject * param = PyObject_CallObject(parameter_cls, args);
  Py_DECREF(args);
  return param;
}

/// Populate a Python dict with a dict of node parameters by node name
/**
 * On failure a Python exception is raised and false is returned
 *
 * \param[in] name The name of the parameter
 * \param[in] variant The variant object to create a Parameter from
 * \param[in] parameter_cls The PythonObject for the Parameter class.
 * \param[in] parameter_type_cls The PythonObject for the Parameter.Type class.
 * \param[out] node_params_dict The PythonObject to populate with node names and parameters.
 *
 * Returns true when parameters are set successfully
 *         false when there was an error during parsing.
 */
static bool
_populate_node_parameters_from_rcl_params(
  const rcl_params_t * params, rcl_allocator_t allocator, PyObject * parameter_cls,
  PyObject * parameter_type_cls, PyObject * node_params_dict)
{
  for (size_t i = 0; i < params->num_nodes; ++i) {
    PyObject * py_node_name;
    if (params->node_names[i][0] != '/') {
      py_node_name = PyUnicode_FromString(
        rcutils_format_string(allocator, "/%s", params->node_names[i]));
    } else {
      py_node_name = PyUnicode_FromString(params->node_names[i]);
    }
    if (!py_node_name) {
      return false;
    }
    PyObject * parameter_dict;
    if (!PyDict_Contains(node_params_dict, py_node_name)) {
      parameter_dict = PyDict_New();
      if (!parameter_dict) {
        Py_DECREF(py_node_name);
        return false;
      }
      if (-1 == PyDict_SetItem(node_params_dict, py_node_name, parameter_dict)) {
        Py_DECREF(parameter_dict);
        Py_DECREF(py_node_name);
        return false;
      }
    } else {
      parameter_dict = PyDict_GetItem(node_params_dict, py_node_name);
      if (!parameter_dict) {
        Py_DECREF(py_node_name);
        PyErr_Format(PyExc_RuntimeError, "Error reading node_paramters from internal dict");
        return false;
      }
      /* This was a borrowed reference. INCREF'd so we can unconditionally DECREF below. */
      Py_INCREF(parameter_dict);
    }
    rcl_node_params_t node_params = params->params[i];
    for (size_t ii = 0; ii < node_params.num_params; ++ii) {
      PyObject * py_param_name = PyUnicode_FromString(node_params.parameter_names[ii]);
      if (!py_param_name) {
        Py_DECREF(py_node_name);
        Py_DECREF(parameter_dict);
        return false;
      }
      PyObject * py_param = _parameter_from_rcl_variant(
        py_param_name, &node_params.parameter_values[ii], parameter_cls,
        parameter_type_cls);
      if (!py_param) {
        Py_DECREF(py_node_name);
        Py_DECREF(parameter_dict);
        Py_DECREF(py_param_name);
        return false;
      }
      if (-1 == PyDict_SetItem(parameter_dict, py_param_name, py_param)) {
        Py_DECREF(py_node_name);
        Py_DECREF(py_param_name);
        Py_DECREF(parameter_dict);
        Py_DECREF(py_param);
        return false;
      }
      Py_DECREF(py_param_name);
      Py_DECREF(py_param);
    }
    Py_DECREF(py_node_name);
    Py_DECREF(parameter_dict);
  }
  return true;
}

/// Populate a Python dict with node parameters parsed from CLI arguments
/**
 * On failure a Python exception is raised and false is returned if:
 *
 * Raises RuntimeError if param_files cannot be extracted from arguments.
 * Raises RuntimeError if yaml files do not parse succesfully.
 *
 * \param[in] module_state the module state for the _rclpy module
 * \param[in] args The arguments to parse for parameters
 * \param[in] allocator Allocator to use for allocating and deallocating within the function.
 * \param[in] parameter_cls The PythonObject for the Parameter class.
 * \param[in] parameter_type_cls The PythonObject for the Parameter.Type class.
 * \param[out] params_by_node_name A Python dict object to place parsed parameters into.
 *
 * Returns true when parameters are parsed successfully (including the trivial case)
 *         false when there was an error during parsing and a Python exception was raised.
 *
 */
static bool
_parse_param_overrides(
  rclpy_module_state_t * module_state,
  const rcl_arguments_t * args, rcl_allocator_t allocator, PyObject * parameter_cls,
  PyObject * parameter_type_cls, PyObject * params_by_node_name)
{
  if (!module_state) {
    PyErr_Format(PyExc_RuntimeError, "_parse_param_overrides got NULL module state");
    return false;
  }
  rcl_params_t * params = NULL;
  if (RCL_RET_OK != rcl_arguments_get_param_overrides(args, &params)) {
    PyErr_Format(
      module_state->RCLError, "Failed to get parameters overrides: %s",
      rcl_get_error_string().str);
    return false;
  }
  if (NULL == params) {
    // No parameter overrides.
    return true;
  }
  bool success = _populate_node_parameters_from_rcl_params(
    params, allocator, parameter_cls, parameter_type_cls, params_by_node_name);
  rcl_yaml_node_struct_fini(params);
  return success;
}

/// Get a list of parameters for the current node from rcl_yaml_param_parser
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises ValueError if the argument is not a node handle.
 * Raises RuntimeError if the parameters file fails to parse
 *
 * \param[in] parameter_cls The rclpy.parameter.Parameter class object.
 * \param[in] node_capsule Capsule pointing to the node handle
 * \return NULL on failure
 *         A dict mapping parameter names to rclpy.parameter.Parameter on success (may be empty).
 */
static PyObject *
rclpy_get_node_parameters(PyObject * module, PyObject * args)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }
  PyObject * parameter_cls;
  PyObject * node_capsule;
  if (!PyArg_ParseTuple(args, "OO", &parameter_cls, &node_capsule)) {
    return NULL;
  }

  rcl_node_t * node = rclpy_handle_get_pointer_from_capsule(node_capsule, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  PyObject * params_by_node_name = PyDict_New();
  if (!params_by_node_name) {
    return NULL;
  }

  if (!PyObject_HasAttrString(parameter_cls, "Type")) {
    PyErr_Format(PyExc_RuntimeError, "Parameter class is missing 'Type' attribute");
    Py_DECREF(params_by_node_name);
    return NULL;
  }
  PyObject * parameter_type_cls = PyObject_GetAttrString(parameter_cls, "Type");
  if (!parameter_type_cls) {
    // PyObject_GetAttrString raises AttributeError on failure.
    Py_DECREF(params_by_node_name);
    return NULL;
  }

  const rcl_node_options_t * node_options = rcl_node_get_options(node);
  const rcl_allocator_t allocator = node_options->allocator;

  if (node_options->use_global_arguments) {
    if (!_parse_param_overrides(
        module_state,
        &(node->context->global_arguments), allocator, parameter_cls,
        parameter_type_cls, params_by_node_name))
    {
      Py_DECREF(parameter_type_cls);
      Py_DECREF(params_by_node_name);
      return NULL;
    }
  }

  if (
    !_parse_param_overrides(
      module_state,
      &(node_options->arguments), allocator, parameter_cls,
      parameter_type_cls, params_by_node_name))
  {
    Py_DECREF(parameter_type_cls);
    Py_DECREF(params_by_node_name);
    return NULL;
  }
  Py_DECREF(parameter_type_cls);

  const char * node_namespace = rcl_node_get_namespace(node);
  char * node_name_with_namespace;
  if ('/' == node_namespace[strlen(node_namespace) - 1]) {
    node_name_with_namespace = rcutils_format_string(
      allocator, "%s%s", node_namespace, rcl_node_get_name(node));
  } else {
    node_name_with_namespace = rcutils_format_string(
      allocator, "%s/%s", node_namespace, rcl_node_get_name(node));
  }

  PyObject * py_node_name_with_namespace = PyUnicode_FromString(node_name_with_namespace);
  allocator.deallocate(node_name_with_namespace, allocator.state);
  if (!py_node_name_with_namespace) {
    Py_DECREF(params_by_node_name);
    return NULL;
  }

  PyObject * node_params = PyDict_New();
  if (!node_params) {
    Py_DECREF(params_by_node_name);
    Py_DECREF(py_node_name_with_namespace);
    return NULL;
  }

  PyObject * py_wildcard_name = PyUnicode_FromString("/**");
  if (!py_wildcard_name) {
    Py_DECREF(params_by_node_name);
    Py_DECREF(py_node_name_with_namespace);
    Py_DECREF(node_params);
    return NULL;
  }

  // Enforce wildcard matching precedence
  // TODO(cottsay) implement further wildcard matching
  PyObject * py_node_names[] = {py_wildcard_name, py_node_name_with_namespace};
  size_t name_count = sizeof(py_node_names) / sizeof(PyObject *);
  for (size_t name_index = 0U; name_index < name_count && NULL != node_params; ++name_index) {
    PyObject * current_key, * current_value;
    Py_ssize_t current_index = 0;
    while (PyDict_Next(params_by_node_name, &current_index, &current_key, &current_value)) {
      if (PyObject_RichCompareBool(current_key, py_node_names[name_index], Py_EQ) == 1) {
        if (-1 == PyDict_Update(node_params, current_value)) {
          Py_DECREF(node_params);
          node_params = NULL;
          break;
        }
      }
    }
  }

  Py_DECREF(params_by_node_name);
  Py_DECREF(py_node_name_with_namespace);
  Py_DECREF(py_wildcard_name);

  return node_params;
}

/// Define the public methods of this module
static PyMethodDef rclpy_methods[] = {
  {
    "rclpy_init", rclpy_init, METH_VARARGS,
    "Initialize RCL."
  },
  {
    "rclpy_logging_configure", rclpy_logging_configure, METH_VARARGS,
    "Initialize RCL logging."
  },
  {
    "rclpy_logging_fini", rclpy_logging_fini, METH_NOARGS,
    "Finalize RCL logging."
  },
  {
    "rclpy_remove_ros_args", rclpy_remove_ros_args, METH_VARARGS,
    "Remove ROS-specific arguments from argument vector."
  },
  {
    "rclpy_create_node", rclpy_create_node, METH_VARARGS,
    "Create a Node."
  },
  {
    "rclpy_get_publisher_logger_name", rclpy_get_publisher_logger_name, METH_VARARGS,
    "Get the logger name associated with the node of a publisher."
  },
  {
    "rclpy_count_publishers", rclpy_count_publishers, METH_VARARGS,
    "Count publishers for a topic."
  },
  {
    "rclpy_count_subscribers", rclpy_count_subscribers, METH_VARARGS,
    "Count subscribers for a topic."
  },
  {
    "rclpy_get_publishers_info_by_topic", rclpy_get_publishers_info_by_topic, METH_VARARGS,
    "Get publishers info for a topic."
  },
  {
    "rclpy_get_subscriptions_info_by_topic", rclpy_get_subscriptions_info_by_topic, METH_VARARGS,
    "Get subscriptions info for a topic."
  },
  {
    "rclpy_create_event", rclpy_create_event, METH_VARARGS,
    "Create an Event."
  },

  {
    "rclpy_take", rclpy_take, METH_VARARGS,
    "rclpy_take."
  },

  {
    "rclpy_take_event", rclpy_take_event, METH_VARARGS,
    "Get the pending data for a ready QoS Event."
  },

  {
    "rclpy_shutdown", rclpy_shutdown, METH_VARARGS,
    "rclpy_shutdown."
  },

  {
    "rclpy_get_node_parameters", rclpy_get_node_parameters, METH_VARARGS,
    "Get the initial parameters for a node from the command line."
  },

  {
    "rclpy_get_rmw_implementation_identifier", rclpy_get_rmw_implementation_identifier,
    METH_NOARGS, "Retrieve the identifier for the active RMW implementation."
  },

  {
    "rclpy_convert_from_py_qos_policy", rclpy_convert_from_py_qos_policy, METH_VARARGS,
    "Convert a QoSPolicy Python object into a rmw_qos_profile_t."
  },
  {
    "rclpy_convert_to_py_qos_policy", rclpy_convert_to_py_qos_policy, METH_VARARGS,
    "Convert a rmw_qos_profile_t into a QoSPolicy Python object."
  },
  {
    "rclpy_get_rmw_qos_profile", rclpy_get_rmw_qos_profile, METH_VARARGS,
    "Get QOS profile."
  },
  {
    "rclpy_assert_liveliness", rclpy_assert_liveliness, METH_VARARGS,
    "Assert the liveliness of an entity."
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
