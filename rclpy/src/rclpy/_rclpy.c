// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <Python.h>

#include <rcl/error_handling.h>
#include <rcl/expand_topic_name.h>
#include <rcl/graph.h>
#include <rcl/node.h>
#include <rcl/publisher.h>
#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rcl/validate_topic_name.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcl_interfaces/msg/parameter_type__struct.h>
#include <rcutils/allocator.h>
#include <rcutils/format_string.h>
#include <rcutils/strdup.h>
#include <rcutils/types.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rmw/types.h>
#include <rmw/validate_full_topic_name.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>
#include <rosidl_generator_c/message_type_support_struct.h>

#include "rclpy_common/common.h"
#include "./_rclpy_qos_event.c"

static PyObject * RCLInvalidROSArgsError;
static PyObject * UnknownROSArgsError;
static PyObject * NodeNameNonExistentError;
static PyObject * RCLError;

void
_rclpy_context_capsule_destructor(PyObject * capsule)
{
  rcl_context_t * context = (rcl_context_t *)PyCapsule_GetPointer(capsule, "rcl_context_t");
  if (!context) {
    return;
  }
  if (NULL != context->impl) {
    rcl_ret_t ret;
    if (rcl_context_is_valid(context)) {
      // shutdown first, if still valid
      ret = rcl_shutdown(context);
      if (RCL_RET_OK != ret) {
        fprintf(stderr,
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "failed to shutdown rcl_context_t (%d) during PyCapsule destructor: %s\n",
          ret,
          rcl_get_error_string().str);
        rcl_reset_error();
      }
    }
    ret = rcl_context_fini(context);
    if (RCL_RET_OK != ret) {
      fprintf(stderr,
        "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
        "failed to fini rcl_context_t (%d) during PyCapsule destructor: %s\n",
        ret,
        rcl_get_error_string().str);
      rcl_reset_error();
    }
  }
  PyMem_FREE(context);
}

/// Create a rcl_context_t.
/**
 * A successful call will return a Capsule with the pointer to the created
 * rcl_context_t structure.
 *
 * The returned context is zero-initialized for use with rclpy_init().
 *
 * Raises RuntimeError if creating the context fails.
 *
 * \return a list with the capsule and memory location, or
 * \return NULL on failure
 */
static PyObject *
rclpy_create_context(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcl_context_t * context = (rcl_context_t *)PyMem_Malloc(sizeof(rcl_context_t));
  if (!context) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for context");
    return NULL;
  }
  *context = rcl_get_zero_initialized_context();
  // if it fails, error is set and NULL is returned as it should
  return PyCapsule_New(context, "rcl_context_t", _rclpy_context_capsule_destructor);
}

/// PyCapsule destructor for guard condition
static void
_rclpy_destroy_guard_condition(PyObject * pyentity)
{
  rcl_guard_condition_t * gc = (rcl_guard_condition_t *)PyCapsule_GetPointer(
    pyentity, "rcl_guard_condition_t");
  if (!gc) {
    // Don't want to raise an exception, who knows where it will get raised.
    PyErr_Clear();
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_guard_condition failed to get pointer");
    return;
  }

  rcl_ret_t ret = rcl_guard_condition_fini(gc);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini guard condition: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(gc);
}

/// Create a general purpose guard condition
/**
 * A successful call will return a Capsule with the pointer of the created
 * rcl_guard_condition_t * structure
 *
 * Raises RuntimeError if initializing the guard condition fails
 *
 * \return a capsule, or
 * \return NULL on failure
 */
static PyObject *
rclpy_create_guard_condition(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pycontext;

  if (!PyArg_ParseTuple(args, "O", &pycontext)) {
    return NULL;
  }

  rcl_context_t * context = (rcl_context_t *)PyCapsule_GetPointer(pycontext, "rcl_context_t");
  if (!context) {
    return NULL;
  }

  rcl_guard_condition_t * gc =
    (rcl_guard_condition_t *)PyMem_Malloc(sizeof(rcl_guard_condition_t));
  if (!gc) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for guard condition");
    return NULL;
  }
  *gc = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t gc_options = rcl_guard_condition_get_default_options();

  rcl_ret_t ret = rcl_guard_condition_init(gc, context, gc_options);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to create guard_condition: %s", rcl_get_error_string().str);
    rcl_reset_error();
    PyMem_Free(gc);
    return NULL;
  }

  PyObject * pygc = PyCapsule_New(gc, "rcl_guard_condition_t", _rclpy_destroy_guard_condition);
  if (!pygc) {
    ret = rcl_guard_condition_fini(gc);
    PyMem_Free(gc);
    return NULL;
  }
  return pygc;
}

/// Trigger a general purpose guard condition
/**
 * Raises ValueError if pygc is not a guard condition capsule
 * Raises RuntimeError if the guard condition could not be triggered
 *
 * \param[in] pygc Capsule pointing to guard condtition
 */
static PyObject *
rclpy_trigger_guard_condition(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pygc;

  if (!PyArg_ParseTuple(args, "O", &pygc)) {
    return NULL;
  }

  rcl_guard_condition_t * gc = (rcl_guard_condition_t *)PyCapsule_GetPointer(
    pygc, "rcl_guard_condition_t");
  if (!gc) {
    return NULL;
  }
  rcl_ret_t ret = rcl_trigger_guard_condition(gc);

  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to trigger guard_condition: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

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
      // NULL in list means array was partially inititialized when an error occurred
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
/* \param[in] pyargs a sequence of string args
 * \param[in] unknown_ros_args_count the number of unknown ROS args
 * \param[in] unknown_ros_args_indices the indices to unknown ROS args
 */
void _rclpy_raise_unknown_ros_args(
  PyObject * pyargs,
  const int * unknown_ros_args_indices,
  int unknown_ros_args_count)
{
  PyObject * unknown_ros_pyargs = NULL;

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
    UnknownROSArgsError,
    "Found unknown ROS arguments: %R",
    unknown_ros_pyargs);
cleanup:
  Py_XDECREF(unknown_ros_pyargs);
  Py_XDECREF(pyargs);
}

/// Parse a sequence of strings into rcl_arguments_t struct
/* Raises TypeError of pyargs is not a sequence
 * Raises OverflowError if len(pyargs) > INT_MAX
 * \param[in] pyargs a Python sequence of strings
 * \param[out] parsed_args a zero initialized pointer to rcl_arguments_t
 */
rcl_ret_t
_rclpy_parse_args(PyObject * pyargs, rcl_arguments_t * parsed_args)
{
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
        RCLInvalidROSArgsError,
        "Failed to parse ROS arguments: %s",
        rcl_get_error_string().str);
    } else {
      PyErr_Format(
        RCLError,
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
        RCLError,
        "Failed to get unparsed ROS arguments: %s",
        rcl_get_error_string().str);
      rcl_reset_error();
      goto cleanup;
    }
    _rclpy_raise_unknown_ros_args(pyargs, unparsed_ros_args_indices, unparsed_ros_args_count);
    allocator.deallocate(unparsed_ros_args_indices, allocator.state);
    ret = RCL_RET_ERROR;
  }
cleanup:
  _rclpy_arg_list_fini(num_args, arg_values);
  return ret;
}

static PyObject *
rclpy_remove_ros_args(PyObject * Py_UNUSED(self), PyObject * args)
{
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
    PyErr_Format(RCLError, "Failed to init: %s", rcl_get_error_string().str);
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
    PyErr_Format(RCLError, "Failed to init: %s", rcl_get_error_string().str);
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
    PyErr_Format(RCLError, "Failed to init: %s", rcl_get_error_string().str);
    rcl_reset_error();
    Py_DECREF(pyresult_list);
    return NULL;
  }

  return pyresult_list;
}

/// Initialize rcl with default options, ignoring parameters
/**
 * Raises RuntimeError if rcl could not be initialized
 */
static PyObject *
rclpy_init(PyObject * Py_UNUSED(self), PyObject * args)
{
  // Expect two arguments, one is a list of strings and the other is a context.
  PyObject * pyargs;
  PyObject * pyseqlist;
  PyObject * pycontext;
  if (!PyArg_ParseTuple(args, "OO", &pyargs, &pycontext)) {
    // Exception raised
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

  rcl_context_t * context = (rcl_context_t *)PyCapsule_GetPointer(pycontext, "rcl_context_t");
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
    if (RCL_RET_OK != ret) {
      PyErr_Format(
        RCLError, "Failed to initialize init_options: %s", rcl_get_error_string().str);
      rcl_reset_error();
    } else {
      ret = rcl_init(num_args, arg_values, &init_options, context);
      if (ret != RCL_RET_OK) {
        PyErr_Format(RCLError, "Failed to init: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
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

/// PyCapsule destructor for node
static void
_rclpy_destroy_node(PyObject * pyentity)
{
  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(
    pyentity, "rcl_node_t");
  if (!node) {
    // Don't want to raise an exception, who knows where it will get raised.
    PyErr_Clear();
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_node failed to get pointer");
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
rclpy_create_node(PyObject * Py_UNUSED(self), PyObject * args)
{
  rcl_ret_t ret;
  const char * node_name;
  const char * namespace_;
  PyObject * pycontext;
  PyObject * py_cli_args;
  int use_global_arguments;
  int enable_rosout;

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

  rcl_context_t * context = (rcl_context_t *)PyCapsule_GetPointer(pycontext, "rcl_context_t");
  if (!context) {
    return NULL;
  }

  rcl_arguments_t arguments = rcl_get_zero_initialized_arguments();

  ret = _rclpy_parse_args(py_cli_args, &arguments);
  if (RCL_RET_OK != ret) {
    // exception set
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyMem_Malloc(sizeof(rcl_node_t));
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
      PyErr_Format(PyExc_MemoryError,
        "%s", rcl_get_error_string().str);
    } else if (ret == RCL_RET_NODE_INVALID_NAME) {
      PyErr_Format(PyExc_ValueError,
        "invalid node name: %s", rcl_get_error_string().str);
    } else if (ret == RCL_RET_NODE_INVALID_NAMESPACE) {
      PyErr_Format(PyExc_ValueError,
        "invalid node namespace: %s", rcl_get_error_string().str);
    } else {
      PyErr_Format(RCLError,
        "Unknown error creating node: %s", rcl_get_error_string().str);
    }
    rcl_reset_error();
    PyMem_Free(node);

    if (RCL_RET_OK != rcl_arguments_fini(&arguments)) {
      rcl_reset_error();
      // Warn because an exception is already raised
      // Warning should use line number of the current stack frame
      int stack_level = 1;
      PyErr_WarnFormat(
        PyExc_RuntimeWarning, stack_level, "Failed to fini arguments during error handling: %s",
        rcl_get_error_string().str);
    }
    return NULL;
  }
  if (RCL_RET_OK != rcl_arguments_fini(&arguments)) {
    rcl_reset_error();
    // Warn because the node was successfully created
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini arguments: %s",
      rcl_get_error_string().str);
  }
  return PyCapsule_New(node, "rcl_node_t", _rclpy_destroy_node);
}

/// Get the name of a node.
/**
 * Raises ValueError if pynode is not a node capsule
 *
 * \param[in] pynode Capsule pointing to the node to get the name from
 * \return None on failure
 *         String containing the name of the node otherwise
 */
static PyObject *
rclpy_get_node_name(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "O", &pynode)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  const char * node_name = rcl_node_get_name(node);
  if (!node_name) {
    Py_RETURN_NONE;
  }

  return PyUnicode_FromString(node_name);
}

/// Get the namespace of a node.
/**
 * Raises ValueError if pynode is not a node capsule
 *
 * \param[in] pynode Capsule pointing to the node to get the namespace from
 * \return namespace, or
 * \return None on failure
 */
static PyObject *
rclpy_get_node_namespace(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "O", &pynode)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  const char * node_namespace = rcl_node_get_namespace(node);
  if (!node_namespace) {
    Py_RETURN_NONE;
  }

  return PyUnicode_FromString(node_namespace);
}

/// Get the name of the logger associated with a node.
/**
 * Raises ValueError if pynode is not a node capsule
 *
 * \param[in] pynode Capsule pointing to the node to get the logger name of
 * \return logger_name, or
 * \return None on failure
 */
static PyObject *
rclpy_get_node_logger_name(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  if (!PyArg_ParseTuple(args, "O", &pynode)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  const char * node_logger_name = rcl_node_get_logger_name(node);
  if (!node_logger_name) {
    Py_RETURN_NONE;
  }

  return PyUnicode_FromString(node_logger_name);
}

typedef rcl_ret_t (* count_func)(const rcl_node_t * node, const char * topic_name, size_t * count);

static PyObject *
_count_subscribers_publishers(PyObject * args, const char * type, count_func count_function)
{
  PyObject * pynode;
  const char * topic_name;

  if (!PyArg_ParseTuple(args, "Os", &pynode, &topic_name)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  size_t count = 0;
  rcl_ret_t ret = count_function(node, topic_name, &count);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError, "Failed to count %s: %s",
      type, rcl_get_error_string().str);
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
rclpy_count_publishers(PyObject * Py_UNUSED(self), PyObject * args)
{
  return _count_subscribers_publishers(args, "publishers", rcl_count_publishers);
}

/// Count subscribers for a topic.
/**
 *
 * \param[in] pynode Capsule pointing to the node to get the namespace from
 * \param[in] topic_name string fully qualified topic name
 * \return count of subscribers
 */
static PyObject *
rclpy_count_subscribers(PyObject * Py_UNUSED(self), PyObject * args)
{
  return _count_subscribers_publishers(args, "subscribers", rcl_count_subscribers);
}

/// Validate a topic name and return error message and index of invalidation.
/**
 * Does not have to be a fully qualified topic name.
 * The topic name is not expanded.
 *
 * Raises MemoryError if memory could not be allocated
 * Raises RuntimeError if an unexpected error happened while validating the topic name
 *
 * \param[in] topic_name name of the topic to be validated
 * \return tuple of error message and invalid index if invalid, or
 * \return None if valid
 */
static PyObject *
rclpy_get_validation_error_for_topic_name(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytopic_name;

  if (!PyArg_ParseTuple(args, "O", &pytopic_name)) {
    return NULL;
  }

  const char * topic_name = PyUnicode_AsUTF8(pytopic_name);
  if (!topic_name) {
    return NULL;
  }

  int validation_result;
  size_t invalid_index;
  rcl_ret_t ret = rcl_validate_topic_name(topic_name, &validation_result, &invalid_index);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rcl_get_error_string().str);
    } else {
      PyErr_Format(RCLError, "%s", rcl_get_error_string().str);
    }
    rcl_reset_error();
    return NULL;
  }

  if (validation_result == RCL_TOPIC_NAME_VALID) {
    Py_RETURN_NONE;
  }

  const char * validation_message = rcl_topic_name_validation_result_string(validation_result);

  PyObject * pyresult_list = PyList_New(2);
  if (!pyresult_list) {
    return NULL;
  }
  PyObject * pyvalidation_message = PyUnicode_FromString(validation_message);
  if (!pyvalidation_message) {
    Py_DECREF(pyresult_list);
    return NULL;
  }
  PyObject * pyinvalid_index = PyLong_FromSize_t(invalid_index);
  if (!pyinvalid_index) {
    Py_DECREF(pyresult_list);
    Py_DECREF(pyvalidation_message);
    return NULL;
  }
  PyList_SET_ITEM(pyresult_list, 0, pyvalidation_message);
  PyList_SET_ITEM(pyresult_list, 1, pyinvalid_index);

  return pyresult_list;
}

/// Validate a full topic name and return error message and index of invalidation.
/**
 * Must be a fully qualified topic name.
 *
 * Raises MemoryError if memory could not be allocated
 * Raises RuntimeError if an unexpected error happened while validating the topic name
 *
 * \param[in] topic_name name of the topic to be validated
 * \return tuple of error message and invalid index if invalid, or
 * \return None if valid
 */
static PyObject *
rclpy_get_validation_error_for_full_topic_name(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytopic_name;

  if (!PyArg_ParseTuple(args, "O", &pytopic_name)) {
    return NULL;
  }

  const char * topic_name = PyUnicode_AsUTF8(pytopic_name);
  if (!topic_name) {
    return NULL;
  }

  int validation_result;
  size_t invalid_index;
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, &invalid_index);
  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rmw_get_error_string().str);
    } else {
      PyErr_Format(RCLError, "%s", rmw_get_error_string().str);
    }
    rmw_reset_error();
    return NULL;
  }

  if (validation_result == RMW_NAMESPACE_VALID) {
    Py_RETURN_NONE;
  }

  const char * validation_message = rmw_full_topic_name_validation_result_string(validation_result);

  PyObject * pyresult_list = PyList_New(2);
  if (!pyresult_list) {
    return NULL;
  }
  PyObject * pyvalidation_message = PyUnicode_FromString(validation_message);
  if (!pyvalidation_message) {
    Py_DECREF(pyresult_list);
    return NULL;
  }
  PyObject * pyinvalid_index = PyLong_FromSize_t(invalid_index);
  if (!pyinvalid_index) {
    Py_DECREF(pyresult_list);
    Py_DECREF(pyvalidation_message);
    return NULL;
  }
  PyList_SET_ITEM(pyresult_list, 0, pyvalidation_message);
  PyList_SET_ITEM(pyresult_list, 1, pyinvalid_index);

  return pyresult_list;
}

/// Validate a namespace and return error message and index of invalidation.
/**
 * Raises MemoryError if memory could not be allocated
 * Raises RuntimeError if an unexpected error happened while validating the namespace
 *
 * \param[in] namespace namespace to be validated
 * \return tuple of error message and invalid index if invalid, or
 * \return None if valid
 */
static PyObject *
rclpy_get_validation_error_for_namespace(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynamespace;

  if (!PyArg_ParseTuple(args, "O", &pynamespace)) {
    return NULL;
  }

  const char * namespace_ = PyUnicode_AsUTF8(pynamespace);
  if (!namespace_) {
    return NULL;
  }

  int validation_result;
  size_t invalid_index;
  rmw_ret_t ret = rmw_validate_namespace(namespace_, &validation_result, &invalid_index);
  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rmw_get_error_string().str);
    } else {
      PyErr_Format(RCLError, "%s", rmw_get_error_string().str);
    }
    rmw_reset_error();
    return NULL;
  }

  if (validation_result == RMW_NAMESPACE_VALID) {
    Py_RETURN_NONE;
  }

  const char * validation_message = rmw_namespace_validation_result_string(validation_result);

  PyObject * pyresult_list = PyList_New(2);
  if (!pyresult_list) {
    return NULL;
  }
  PyObject * pyvalidation_message = PyUnicode_FromString(validation_message);
  if (!pyvalidation_message) {
    Py_DECREF(pyresult_list);
    return NULL;
  }
  PyObject * pyinvalid_index = PyLong_FromSize_t(invalid_index);
  if (!pyinvalid_index) {
    Py_DECREF(pyresult_list);
    Py_DECREF(pyvalidation_message);
    return NULL;
  }
  PyList_SET_ITEM(pyresult_list, 0, pyvalidation_message);
  PyList_SET_ITEM(pyresult_list, 1, pyinvalid_index);

  return pyresult_list;
}

/// Validate a node name and return error message and index of invalidation.
/**
 * Raises MemoryError if memory could not be allocated
 * Raises RuntimeError if an unexpected error happened while validating the node name
 *
 * \param[in] node_name name of the node to be validated
 * \return tuple of error message and invalid index if invalid, or
 * \return None if valid
 */
static PyObject *
rclpy_get_validation_error_for_node_name(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode_name;

  if (!PyArg_ParseTuple(args, "O", &pynode_name)) {
    return NULL;
  }

  const char * node_name = PyUnicode_AsUTF8(pynode_name);
  if (!node_name) {
    return NULL;
  }

  int validation_result;
  size_t invalid_index;
  rmw_ret_t ret = rmw_validate_node_name(node_name, &validation_result, &invalid_index);
  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rmw_get_error_string().str);
    } else {
      PyErr_Format(RCLError, "%s", rmw_get_error_string().str);
    }
    rmw_reset_error();
    return NULL;
  }

  if (validation_result == RMW_NODE_NAME_VALID) {
    Py_RETURN_NONE;
  }

  const char * validation_message = rmw_node_name_validation_result_string(validation_result);

  PyObject * pyresult_list = PyList_New(2);
  if (!pyresult_list) {
    return NULL;
  }
  PyObject * pyvalidation_message = PyUnicode_FromString(validation_message);
  if (!pyvalidation_message) {
    Py_DECREF(pyresult_list);
    return NULL;
  }
  PyObject * pyinvalid_index = PyLong_FromSize_t(invalid_index);
  if (!pyinvalid_index) {
    Py_DECREF(pyresult_list);
    Py_DECREF(pyvalidation_message);
    return NULL;
  }
  PyList_SET_ITEM(pyresult_list, 0, pyvalidation_message);
  PyList_SET_ITEM(pyresult_list, 1, pyinvalid_index);

  return pyresult_list;
}

static char *
_expand_topic_name_with_exceptions(const char * topic, const char * node, const char * namespace)
{
  char * expanded_topic = NULL;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcutils_allocator_t rcutils_allocator = rcutils_get_default_allocator();
  rcutils_string_map_t substitutions_map = rcutils_get_zero_initialized_string_map();

  rcutils_ret_t rcutils_ret = rcutils_string_map_init(&substitutions_map, 0, rcutils_allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    if (rcutils_ret == RCUTILS_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rcutils_get_error_string().str);
    } else {
      PyErr_Format(RCLError, "%s", rcutils_get_error_string().str);
    }
    rcutils_reset_error();
    return NULL;
  }
  rcl_ret_t ret = rcl_get_default_topic_name_substitutions(&substitutions_map);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rcl_get_error_string().str);
    } else {
      PyErr_Format(RCLError, "%s", rcl_get_error_string().str);
    }
    rcl_reset_error();
    // finalize the string map before returning
    rcutils_ret = rcutils_string_map_fini(&substitutions_map);
    if (rcutils_ret != RCUTILS_RET_OK) {
      fprintf(stderr,
        "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
        "failed to fini string_map (%d) during error handling: %s\n",
        rcutils_ret,
        rcutils_get_error_string().str);
      rcutils_reset_error();
    }
    return NULL;
  }

  ret = rcl_expand_topic_name(
    topic,
    node,
    namespace,
    &substitutions_map,
    allocator,
    &expanded_topic);

  rcutils_ret = rcutils_string_map_fini(&substitutions_map);
  if (rcutils_ret != RCUTILS_RET_OK) {
    PyErr_Format(PyExc_RuntimeError, "%s", rcutils_get_error_string().str);
    rcutils_reset_error();
    allocator.deallocate(expanded_topic, allocator.state);
    return NULL;
  }
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rcl_get_error_string().str);
    } else if (  // NOLINT
      ret == RCL_RET_TOPIC_NAME_INVALID ||
      ret == RCL_RET_UNKNOWN_SUBSTITUTION)
    {
      PyErr_Format(PyExc_ValueError,
        "topic name '%s' is invalid: %s", topic, rcl_get_error_string().str);
    } else if (ret == RCL_RET_NODE_INVALID_NAME) {
      PyErr_Format(PyExc_ValueError,
        "node name '%s' is invalid: %s", node, rcl_get_error_string().str);
    } else if (ret == RCL_RET_NODE_INVALID_NAMESPACE) {
      PyErr_Format(PyExc_ValueError,
        "node namespace '%s' is invalid: %s", namespace, rcl_get_error_string().str);
    } else {
      PyErr_Format(RCLError, "%s", rcl_get_error_string().str);
    }
    rcl_reset_error();
    return NULL;
  }

  return expanded_topic;
}

/// Expand a topic name
/**
 * Raises ValueError if the topic name, node name, or namespace are not valid.
 *
 * \param[in] topic_name topic string to be expanded
 * \param[in] node_name name of the node to be used during expansion
 * \param[in] node_namespace namespace of the node to be used during expansion
 * \return expanded node namespace
 */
static PyObject *
rclpy_expand_topic_name(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytopic_name;
  PyObject * pynode_name;
  PyObject * pynode_namespace;

  if (!PyArg_ParseTuple(args, "OOO", &pytopic_name, &pynode_name, &pynode_namespace)) {
    return NULL;
  }

  const char * topic = PyUnicode_AsUTF8(pytopic_name);
  if (!topic) {
    return NULL;
  }

  const char * node_name = PyUnicode_AsUTF8(pynode_name);
  if (!node_name) {
    return NULL;
  }

  const char * node_namespace = PyUnicode_AsUTF8(pynode_namespace);
  if (!node_namespace) {
    return NULL;
  }

  char * expanded_topic = _expand_topic_name_with_exceptions(topic, node_name, node_namespace);

  if (!expanded_topic) {
    // exception already set
    return NULL;
  }

  PyObject * result = PyUnicode_FromString(expanded_topic);
  if (!result) {
    return NULL;
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();
  allocator.deallocate(expanded_topic, allocator.state);

  return result;
}

/// PyCapsule destructor for publisher
static void
_rclpy_destroy_publisher(PyObject * pyentity)
{
  rclpy_publisher_t * pub = (rclpy_publisher_t *)PyCapsule_GetPointer(
    pyentity, "rclpy_publisher_t");
  if (!pub) {
    // Don't want to raise an exception, who knows where it will get raised.
    PyErr_Clear();
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_publisher failed to get pointer");
    return;
  }

  rcl_ret_t ret = rcl_publisher_fini(&(pub->publisher), pub->node);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini publisher: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(pub);
}

/// Create a publisher
/**
 * This function will create a publisher and attach it to the provided topic name
 * This publisher will use the typesupport defined in the message module
 * provided as pymsg_type to send messages over the wire.
 *
 * Raises ValueError if the topic name is invalid
 * Raises ValueError if the capsules are not the correct types
 * Raises RuntimeError if the publisher cannot be created
 *
 * \param[in] pynode Capsule pointing to the node to add the publisher to
 * \param[in] pymsg_type Message type associated with the publisher
 * \param[in] pytopic Python object containing the name of the topic
 * to attach the publisher to
 * \param[in] pyqos_profile QoSProfile object with the profile of this publisher
 * \return Capsule of the pointer to the created rcl_publisher_t * structure, or
 * \return NULL on failure
 */
static PyObject *
rclpy_create_publisher(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pymsg_type;
  PyObject * pytopic;
  PyObject * pyqos_profile;

  if (!PyArg_ParseTuple(args, "OOOO", &pynode, &pymsg_type, &pytopic, &pyqos_profile)) {
    return NULL;
  }

  const char * topic = PyUnicode_AsUTF8(pytopic);
  if (!topic) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__");
  if (!pymetaclass) {
    return NULL;
  }

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");
  Py_DECREF(pymetaclass);
  if (!pyts) {
    return NULL;
  }

  rosidl_message_type_support_t * ts =
    (rosidl_message_type_support_t *)PyCapsule_GetPointer(pyts, NULL);
  Py_DECREF(pyts);
  if (!ts) {
    return NULL;
  }

  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  if (PyCapsule_IsValid(pyqos_profile, "rmw_qos_profile_t")) {
    void * p = PyCapsule_GetPointer(pyqos_profile, "rmw_qos_profile_t");
    rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)p;
    publisher_ops.qos = *qos_profile;
    // TODO(jacobperron): It is not obvious why the capsule reference should be destroyed here.
    // Instead, a safer pattern would be to destroy the QoS object with its own destructor.
    PyMem_Free(p);
    if (PyCapsule_SetPointer(pyqos_profile, Py_None)) {
      // exception set by PyCapsule_SetPointer
      return NULL;
    }
  }

  rclpy_publisher_t * pub = (rclpy_publisher_t *)PyMem_Malloc(sizeof(rclpy_publisher_t));
  if (!pub) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for publisher");
    return NULL;
  }
  pub->publisher = rcl_get_zero_initialized_publisher();
  pub->node = node;

  rcl_ret_t ret = rcl_publisher_init(&(pub->publisher), node, ts, topic, &publisher_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      PyErr_Format(PyExc_ValueError,
        "Failed to create publisher due to invalid topic name '%s': %s",
        topic, rcl_get_error_string().str);
    } else {
      PyErr_Format(RCLError,
        "Failed to create publisher: %s", rcl_get_error_string().str);
    }
    rcl_reset_error();
    PyMem_Free(pub);
    return NULL;
  }
  return PyCapsule_New(pub, "rclpy_publisher_t", _rclpy_destroy_publisher);
}

/// Publish a message
/**
 * Raises ValueError if pypublisher is not a publisher capsule
 * Raises RuntimeError if the message cannot be published
 *
 * \param[in] pypublisher Capsule pointing to the publisher
 * \param[in] pymsg message to send
 * \return NULL
 */
static PyObject *
rclpy_publish(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pypublisher;
  PyObject * pymsg;

  if (!PyArg_ParseTuple(args, "OO", &pypublisher, &pymsg)) {
    return NULL;
  }

  rclpy_publisher_t * pub = (rclpy_publisher_t *)PyCapsule_GetPointer(
    pypublisher, "rclpy_publisher_t");
  if (!pub) {
    return NULL;
  }

  destroy_ros_message_signature * destroy_ros_message = NULL;
  void * raw_ros_message = rclpy_convert_from_py(pymsg, &destroy_ros_message);
  if (!raw_ros_message) {
    return NULL;
  }

  rcl_ret_t ret = rcl_publish(&(pub->publisher), raw_ros_message, NULL);
  destroy_ros_message(raw_ros_message);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to publish: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Count subscribers from a publisher.
/**
 *
 * \param[in] pynode Capsule pointing to the publisher
 * \return count of subscribers
 */
static PyObject *
rclpy_publisher_get_subscription_count(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pypublisher;

  if (!PyArg_ParseTuple(args, "O", &pypublisher)) {
    return NULL;
  }

  const rclpy_publisher_t * pub = (rclpy_publisher_t *)PyCapsule_GetPointer(
    pypublisher, "rclpy_publisher_t");
  if (!pub) {
    return NULL;
  }

  size_t count = 0;
  rmw_ret_t ret = rcl_publisher_get_subscription_count(&pub->publisher, &count);
  if (RMW_RET_OK != ret) {
    PyErr_Format(RCLError, "%s", rmw_get_error_string().str);
    rmw_reset_error();
    return NULL;
  }
  return PyLong_FromSize_t(count);
}

/// PyCapsule destructor for timer
static void
_rclpy_destroy_timer(PyObject * pyentity)
{
  rcl_timer_t * tmr = (rcl_timer_t *)PyCapsule_GetPointer(
    pyentity, "rcl_timer_t");
  if (!tmr) {
    // Don't want to raise an exception, who knows where it will get raised.
    PyErr_Clear();
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_timer failed to get pointer");
    return;
  }

  rcl_ret_t ret = rcl_timer_fini(tmr);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini timer: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(tmr);
}

/// Create a timer
/**
 * When successful a Capsule pointing to the pointer of the created rcl_timer_t * structure
 * is returned
 *
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises RuntimeError on initialization failure
 * Raises TypeError if argument of invalid type
 * Raises ValueError if argument cannot be converted to uint64_t
 *
 * \param[in] clock pycapsule containing an rcl_clock_t
 * \param[in] period_nsec unsigned PyLong object storing the period of the
 *   timer in nanoseconds in a 64-bit unsigned integer
 * \return a capsule
 * \return NULL on failure
 */
static PyObject *
rclpy_create_timer(PyObject * Py_UNUSED(self), PyObject * args)
{
  unsigned PY_LONG_LONG period_nsec;
  PyObject * pyclock;
  PyObject * pycontext;

  if (!PyArg_ParseTuple(args, "OOK", &pyclock, &pycontext, &period_nsec)) {
    return NULL;
  }

  rcl_context_t * context = (rcl_context_t *)PyCapsule_GetPointer(pycontext, "rcl_context_t");
  if (!context) {
    return NULL;
  }

  rcl_clock_t * clock = (rcl_clock_t *) PyCapsule_GetPointer(pyclock, "rcl_clock_t");
  if (!clock) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *) PyMem_Malloc(sizeof(rcl_timer_t));
  if (!timer) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for timer");
    return NULL;
  }
  *timer = rcl_get_zero_initialized_timer();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_timer_init(timer, clock, context, period_nsec, NULL, allocator);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to create timer: %s", rcl_get_error_string().str);
    rcl_reset_error();
    PyMem_Free(timer);
    return NULL;
  }

  PyObject * pytimer = PyCapsule_New(timer, "rcl_timer_t", _rclpy_destroy_timer);
  if (!pytimer) {
    ret = rcl_timer_fini(timer);
    (void)ret;
    PyMem_Free(timer);
    return NULL;
  }
  return pytimer;
}

/// Returns the period of the timer in nanoseconds
/**
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RuntimeError if the timer period cannot be retrieved
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return NULL on failure:
 *         PyLong integer in nanoseconds on success
 */
static PyObject *
rclpy_get_timer_period(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, "rcl_timer_t");
  if (!timer) {
    return NULL;
  }
  int64_t timer_period;
  rcl_ret_t ret = rcl_timer_get_period(timer, &timer_period);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to get timer period: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  return PyLong_FromUnsignedLongLong(timer_period);
}

/// Cancel the timer
/**
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RuntimeError if the timmer cannot be canceled
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return NULL on failure:
 *         NULL on success
 */
static PyObject *
rclpy_cancel_timer(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, "rcl_timer_t");
  if (!timer) {
    return NULL;
  }
  rcl_ret_t ret = rcl_timer_cancel(timer);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to reset timer: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Checks if timer is cancelled
/**
 * Raises ValueError if pytimer is not a timer capsule
 * Raises Runtime error if there is an rcl error
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return False on failure:
 *         True on success
 */
static PyObject *
rclpy_is_timer_canceled(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, "rcl_timer_t");
  if (!timer) {
    return NULL;
  }
  bool is_canceled;
  rcl_ret_t ret = rcl_timer_is_canceled(timer, &is_canceled);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to check timer ready: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  if (is_canceled) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

/// Reset the timer
/**
 * Raise ValueError if capsule is not a timer
 * Raises Runtime error if the timer cannot be reset
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return None
 */
static PyObject *
rclpy_reset_timer(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, "rcl_timer_t");
  if (!timer) {
    return NULL;
  }
  rcl_ret_t ret = rcl_timer_reset(timer);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to reset timer: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Checks if timer reached its timeout
/**
 *  Raises ValueError if pytimer is not a timer capsule
 *  Raises RuntimeError if there is an rcl error
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return True if the timer is ready
 */
static PyObject *
rclpy_is_timer_ready(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, "rcl_timer_t");
  if (!timer) {
    return NULL;
  }
  bool is_ready;
  rcl_ret_t ret = rcl_timer_is_ready(timer, &is_ready);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to check timer ready: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  if (is_ready) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

/// Set the last call time and start counting again
/**
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RuntimeError if there is an rcl error
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return NULL on failure:
 *         NULL on success
 */
static PyObject *
rclpy_call_timer(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, "rcl_timer_t");
  if (!timer) {
    return NULL;
  }
  rcl_ret_t ret = rcl_timer_call(timer);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to call timer: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Update the timer period
/**
 * The change in period will take effect after the next timer call
 *
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RuntimeError if the timer period could not be changed
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \param[in] period_nsec unsigned PyLongLong containing the new period in nanoseconds
 * \return None
 */
static PyObject *
rclpy_change_timer_period(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  unsigned PY_LONG_LONG period_nsec;
  if (!PyArg_ParseTuple(args, "OK", &pytimer, &period_nsec)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, "rcl_timer_t");
  if (!timer) {
    return NULL;
  }
  int64_t old_period;
  rcl_ret_t ret = rcl_timer_exchange_period(timer, period_nsec, &old_period);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to exchange timer period: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Get the time before the timer will be ready
/**
 * the returned time can be negative, this means that the timer is ready and hasn't been called yet
 *
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RuntimeError there is an rcl error
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return PyLongLong containing the time until next call in nanoseconds
 */
static PyObject *
rclpy_time_until_next_call(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, "rcl_timer_t");
  if (!timer) {
    return NULL;
  }
  int64_t remaining_time;
  rcl_ret_t ret = rcl_timer_get_time_until_next_call(timer, &remaining_time);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to get time until next timer call: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  return PyLong_FromLongLong(remaining_time);
}

/// Get the time since the timer has been called
/**
 * Raises RuntimeError if there is an rcl error
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return unsigned PyLongLong containing the time since last call in nanoseconds
 */
static PyObject *
rclpy_time_since_last_call(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytimer;
  if (!PyArg_ParseTuple(args, "O", &pytimer)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pytimer, "rcl_timer_t");
  if (!timer) {
    return NULL;
  }
  int64_t elapsed_time;
  rcl_ret_t ret = rcl_timer_get_time_since_last_call(timer, &elapsed_time);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to get time since last timer call: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  return PyLong_FromUnsignedLongLong(elapsed_time);
}

/// PyCapsule destructor for subscription
static void
_rclpy_destroy_subscription(PyObject * pyentity)
{
  rclpy_subscription_t * sub = (rclpy_subscription_t *)PyCapsule_GetPointer(
    pyentity, "rclpy_subscription_t");
  if (!sub) {
    // Don't want to raise an exception, who knows where it will get raised.
    PyErr_Clear();
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_subscrition failed to get pointer");
    return;
  }

  rcl_ret_t ret = rcl_subscription_fini(&(sub->subscription), sub->node);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini subscription: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(sub);
}

/// Create a subscription
/**
 * This function will create a subscription for the given topic name.
 * This subscription will use the typesupport defined in the message module
 * provided as pymsg_type to send messages over the wire.
 *
 * On a successful call a list with two elements is returned:
 *
 * - a Capsule pointing to the pointer of the created rcl_subscription_t * structure
 * - an integer representing the memory address of the created rcl_subscription_t
 *
 * Raises ValueError if the capsules are not the correct types
 * Raises RuntimeError if the subscription could not be created
 *
 * \param[in] pynode Capsule pointing to the node to add the subscriber to
 * \param[in] pymsg_type Message module associated with the subscriber
 * \param[in] pytopic Python object containing the topic name
 * \param[in] pyqos_profile QoSProfile Python object for this subscription
 * \return list with the capsule and memory address, or
 * \return NULL on failure
 */
static PyObject *
rclpy_create_subscription(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pymsg_type;
  PyObject * pytopic;
  PyObject * pyqos_profile;

  if (!PyArg_ParseTuple(args, "OOOO", &pynode, &pymsg_type, &pytopic, &pyqos_profile)) {
    return NULL;
  }

  const char * topic = PyUnicode_AsUTF8(pytopic);
  if (!topic) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__");
  if (!pymetaclass) {
    return NULL;
  }

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");
  Py_DECREF(pymetaclass);
  if (!pyts) {
    return NULL;
  }

  rosidl_message_type_support_t * ts =
    (rosidl_message_type_support_t *)PyCapsule_GetPointer(pyts, NULL);
  Py_DECREF(pyts);
  if (!ts) {
    return NULL;
  }

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();

  if (PyCapsule_IsValid(pyqos_profile, "rmw_qos_profile_t")) {
    void * p = PyCapsule_GetPointer(pyqos_profile, "rmw_qos_profile_t");
    rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)p;
    subscription_ops.qos = *qos_profile;
    // TODO(jacobperron): It is not obvious why the capsule reference should be destroyed here.
    // Instead, a safer pattern would be to destroy the QoS object with its own destructor.
    PyMem_Free(p);
    if (PyCapsule_SetPointer(pyqos_profile, Py_None)) {
      // exception set by PyCapsule_SetPointer
      return NULL;
    }
  }

  rclpy_subscription_t * sub =
    (rclpy_subscription_t *)PyMem_Malloc(sizeof(rclpy_subscription_t));
  if (!sub) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for subscription");
    return NULL;
  }
  sub->subscription = rcl_get_zero_initialized_subscription();
  sub->node = node;

  rcl_ret_t ret = rcl_subscription_init(&(sub->subscription), node, ts, topic, &subscription_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      PyErr_Format(PyExc_ValueError,
        "Failed to create subscription due to invalid topic name '%s': %s",
        topic, rcl_get_error_string().str);
    } else {
      PyErr_Format(RCLError,
        "Failed to create subscription: %s", rcl_get_error_string().str);
    }
    rcl_reset_error();
    PyMem_Free(sub);
    return NULL;
  }

  PyObject * pysubscription = PyCapsule_New(
    sub, "rclpy_subscription_t", _rclpy_destroy_subscription);
  if (!pysubscription) {
    ret = rcl_subscription_fini(&(sub->subscription), node);
    (void)ret;
    PyMem_Free(sub);
    return NULL;
  }
  return pysubscription;
}

/// PyCapsule destructor for client
static void
_rclpy_destroy_client(PyObject * pyentity)
{
  rclpy_client_t * cli = (rclpy_client_t *)PyCapsule_GetPointer(
    pyentity, "rclpy_client_t");
  if (!cli) {
    // Don't want to raise an exception, who knows where it will get raised.
    PyErr_Clear();
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_client failed to get pointer");
    return;
  }

  rcl_ret_t ret = rcl_client_fini(&(cli->client), cli->node);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini client: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(cli);
}

/// Create a client
/**
 * This function will create a client for the given service name.
 * This client will use the typesupport defined in the service module
 * provided as pysrv_type to send messages over the wire.
 *
 * On a successful call a Capsule pointing to the pointer of the created rclpy_client_t * is
 * returned.
 *
 * Raises ValueError if the capsules are not the correct types
 * Raises RuntimeError if the client could not be created
 *
 * \param[in] pynode Capsule pointing to the node to add the client to
 * \param[in] pysrv_type Service module associated with the client
 * \param[in] pyservice_name Python object containing the service name
 * \param[in] pyqos_profile QoSProfile Python object for this client
 * \return capsule or,
 * \return NULL on failure
 */
static PyObject *
rclpy_create_client(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pysrv_type;
  PyObject * pyservice_name;
  PyObject * pyqos_profile;

  if (!PyArg_ParseTuple(args, "OOOO", &pynode, &pysrv_type, &pyservice_name, &pyqos_profile)) {
    return NULL;
  }

  const char * service_name = PyUnicode_AsUTF8(pyservice_name);
  if (!service_name) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  PyObject * pymetaclass = PyObject_GetAttrString(pysrv_type, "__class__");
  if (!pymetaclass) {
    return NULL;
  }

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");
  Py_DECREF(pymetaclass);
  if (!pyts) {
    return NULL;
  }

  rosidl_service_type_support_t * ts =
    (rosidl_service_type_support_t *)PyCapsule_GetPointer(pyts, NULL);
  Py_DECREF(pyts);
  if (!ts) {
    return NULL;
  }

  rcl_client_options_t client_ops = rcl_client_get_default_options();

  if (PyCapsule_IsValid(pyqos_profile, "rmw_qos_profile_t")) {
    void * p = PyCapsule_GetPointer(pyqos_profile, "rmw_qos_profile_t");
    rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)p;
    client_ops.qos = *qos_profile;
    // TODO(jacobperron): It is not obvious why the capsule reference should be destroyed here.
    // Instead, a safer pattern would be to destroy the QoS object with its own destructor.
    PyMem_Free(p);
    if (PyCapsule_SetPointer(pyqos_profile, Py_None)) {
      // exception set by PyCapsule_SetPointer
      return NULL;
    }
  }

  rclpy_client_t * client = (rclpy_client_t *)PyMem_Malloc(sizeof(rclpy_client_t));
  if (!client) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for client");
    return NULL;
  }
  client->client = rcl_get_zero_initialized_client();
  client->node = node;

  rcl_ret_t ret = rcl_client_init(&(client->client), node, ts, service_name, &client_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      PyErr_Format(PyExc_ValueError,
        "Failed to create client due to invalid service name '%s': %s",
        service_name, rcl_get_error_string().str);
    } else {
      PyErr_Format(RCLError,
        "Failed to create client: %s", rcl_get_error_string().str);
    }
    rcl_reset_error();
    PyMem_Free(client);
    return NULL;
  }
  PyObject * pyclient = PyCapsule_New(client, "rclpy_client_t", _rclpy_destroy_client);
  if (!pyclient) {
    ret = rcl_client_fini(&(client->client), node);
    (void)ret;
    PyMem_Free(client);
    return NULL;
  }

  return pyclient;
}

/// Publish a request message
/**
 * Raises ValueError if pyclient is not a client capsule
 * Raises RuntimeError if the request could not be sent
 *
 * \param[in] pyclient Capsule pointing to the client
 * \param[in] pyrequest request message to send
 * \return sequence_number PyLong object representing the index of the sent request
 */
static PyObject *
rclpy_send_request(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclient;
  PyObject * pyrequest;

  if (!PyArg_ParseTuple(args, "OO", &pyclient, &pyrequest)) {
    return NULL;
  }
  rclpy_client_t * client = (rclpy_client_t *)PyCapsule_GetPointer(pyclient, "rclpy_client_t");
  if (!client) {
    return NULL;
  }

  destroy_ros_message_signature * destroy_ros_message = NULL;
  void * raw_ros_request = rclpy_convert_from_py(pyrequest, &destroy_ros_message);
  if (!raw_ros_request) {
    return NULL;
  }

  int64_t sequence_number;
  rcl_ret_t ret = rcl_send_request(&(client->client), raw_ros_request, &sequence_number);
  destroy_ros_message(raw_ros_request);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to send request: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  return PyLong_FromLongLong(sequence_number);
}

/// PyCapsule destructor for service
static void
_rclpy_destroy_service(PyObject * pyentity)
{
  rclpy_service_t * srv = (rclpy_service_t *)PyCapsule_GetPointer(
    pyentity, "rclpy_service_t");
  if (!srv) {
    // Don't want to raise an exception, who knows where it will get raised.
    PyErr_Clear();
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_service failed to get pointer");
    return;
  }

  rcl_ret_t ret = rcl_service_fini(&(srv->service), srv->node);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini service: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(srv);
}

/// Create a service server
/**
 * This function will create a service server for the given service name.
 * This service will use the typesupport defined in the service module
 * provided as pysrv_type to send messages over the wire.
 *
 *
 * On a successful call a Capsule pointing to the pointer of the created rcl_service_t *
 * is returned.
 *
 * Raises ValueError if the capsules are not the correct types
 * Raises RuntimeError if the service could not be created
 *
 * \param[in] pynode Capsule pointing to the node to add the service to
 * \param[in] pysrv_type Service module associated with the service
 * \param[in] pyservice_name Python object for the service name
 * \param[in] pyqos_profile QoSProfile Python object for this service
 * \return capsule, or
 * \return NULL on failure
 */
static PyObject *
rclpy_create_service(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pysrv_type;
  PyObject * pyservice_name;
  PyObject * pyqos_profile;

  if (!PyArg_ParseTuple(args, "OOOO", &pynode, &pysrv_type, &pyservice_name, &pyqos_profile)) {
    return NULL;
  }

  const char * service_name = PyUnicode_AsUTF8(pyservice_name);
  if (!service_name) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  PyObject * pymetaclass = PyObject_GetAttrString(pysrv_type, "__class__");
  if (!pymetaclass) {
    return NULL;
  }

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");
  Py_DECREF(pymetaclass);
  if (!pyts) {
    return NULL;
  }

  rosidl_service_type_support_t * ts =
    (rosidl_service_type_support_t *)PyCapsule_GetPointer(pyts, NULL);
  Py_DECREF(pyts);
  if (!ts) {
    return NULL;
  }

  rcl_service_options_t service_ops = rcl_service_get_default_options();

  if (PyCapsule_IsValid(pyqos_profile, "rmw_qos_profile_t")) {
    void * p = PyCapsule_GetPointer(pyqos_profile, "rmw_qos_profile_t");
    rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)p;
    service_ops.qos = *qos_profile;
    // TODO(jacobperron): It is not obvious why the capsule reference should be destroyed here.
    // Instead, a safer pattern would be to destroy the QoS object with its own destructor.
    PyMem_Free(p);
    if (PyCapsule_SetPointer(pyqos_profile, Py_None)) {
      // exception set by PyCapsule_SetPointer
      return NULL;
    }
  }

  rclpy_service_t * srv = (rclpy_service_t *)PyMem_Malloc(sizeof(rclpy_service_t));
  if (!srv) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for service");
    return NULL;
  }
  srv->service = rcl_get_zero_initialized_service();
  srv->node = node;

  rcl_ret_t ret = rcl_service_init(&(srv->service), node, ts, service_name, &service_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      PyErr_Format(PyExc_ValueError,
        "Failed to create service due to invalid topic name '%s': %s",
        service_name, rcl_get_error_string().str);
    } else {
      PyErr_Format(RCLError,
        "Failed to create service: %s", rcl_get_error_string().str);
    }
    PyMem_Free(srv);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pyservice = PyCapsule_New(srv, "rclpy_service_t", _rclpy_destroy_service);
  if (!pyservice) {
    ret = rcl_service_fini(&(srv->service), node);
    (void)ret;
    PyMem_Free(srv);
    return NULL;
  }
  return pyservice;
}

/// Publish a response message
/**
 * Raises ValueError if the capsules are not the correct types
 * Raises RuntimeError if the response could not be sent
 *
 * \param[in] pyservice Capsule pointing to the service
 * \param[in] pyresponse reply message to send
 * \param[in] pyheader Capsule pointing to the rmw_request_id_t header of the request we respond to
 * \return NULL
 */
static PyObject *
rclpy_send_response(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyservice;
  PyObject * pyresponse;
  PyObject * pyheader;

  if (!PyArg_ParseTuple(args, "OOO", &pyservice, &pyresponse, &pyheader)) {
    return NULL;
  }
  rclpy_service_t * srv = (rclpy_service_t *)PyCapsule_GetPointer(pyservice, "rclpy_service_t");
  if (!srv) {
    return NULL;
  }

  rmw_request_id_t * header = (rmw_request_id_t *)PyCapsule_GetPointer(
    pyheader, "rmw_request_id_t");
  if (!header) {
    return NULL;
  }

  destroy_ros_message_signature * destroy_ros_message = NULL;
  void * raw_ros_response = rclpy_convert_from_py(pyresponse, &destroy_ros_message);
  if (!raw_ros_response) {
    return NULL;
  }

  rcl_ret_t ret = rcl_send_response(&(srv->service), header, raw_ros_response);
  destroy_ros_message(raw_ros_response);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to send request: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Check if a service server is available
/**
 * Raises ValueError if the arguments are not capsules
 *
 * \param[in] pyclient Capsule pointing to the client
 * \return True if the service server is available
 */
static PyObject *
rclpy_service_server_is_available(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclient;

  if (!PyArg_ParseTuple(args, "O", &pyclient)) {
    return NULL;
  }

  rclpy_client_t * client = (rclpy_client_t *)PyCapsule_GetPointer(pyclient, "rclpy_client_t");
  if (!client) {
    return NULL;
  }

  bool is_ready;
  rcl_ret_t ret = rcl_service_server_is_available(client->node, &(client->client), &is_ready);

  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to check service availability: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  if (is_ready) {
    Py_RETURN_TRUE;
  }
  Py_RETURN_FALSE;
}

/// Destructor for a clock
static void
_rclpy_destroy_clock(PyObject * pycapsule)
{
  rcl_clock_t * clock = (rcl_clock_t *)PyCapsule_GetPointer(pycapsule, "rcl_clock_t");
  if (!clock) {
    // exception was set by PyCapsule_GetPointer
    PyErr_Clear();
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to get clock pointer in destructor");
    rcl_reset_error();
    return;
  }

  rcl_ret_t ret_clock = rcl_clock_fini(clock);
  PyMem_Free(clock);
  if (ret_clock != RCL_RET_OK) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini clock: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
  }
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

/// Return a Capsule pointing to a zero initialized rcl_wait_set_t structure
static PyObject *
rclpy_get_zero_initialized_wait_set(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyMem_Malloc(sizeof(rcl_wait_set_t));
  if (!wait_set) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for wait set");
    return NULL;
  }
  *wait_set = rcl_get_zero_initialized_wait_set();
  PyObject * pywait_set = PyCapsule_New(wait_set, "rcl_wait_set_t", NULL);

  return pywait_set;
}

/// Initialize a wait set
/**
 * Raises RuntimeError if the wait set could not be initialized
 *
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \param[in] node_name string name of the node to be created
 * \param[in] number_of_subscriptions int
 * \param[in] number_of_guard_conditions int
 * \param[in] number_of_timers int
 * \param[in] number_of_clients int
 * \param[in] number_of_services int
 * \return None
 */
static PyObject *
rclpy_wait_set_init(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;
  unsigned PY_LONG_LONG number_of_subscriptions;
  unsigned PY_LONG_LONG number_of_guard_conditions;
  unsigned PY_LONG_LONG number_of_timers;
  unsigned PY_LONG_LONG number_of_clients;
  unsigned PY_LONG_LONG number_of_services;
  unsigned PY_LONG_LONG number_of_events;
  PyObject * pycontext;

  if (!PyArg_ParseTuple(
      args, "OKKKKKKO", &pywait_set, &number_of_subscriptions,
      &number_of_guard_conditions, &number_of_timers,
      &number_of_clients, &number_of_services, &number_of_events, &pycontext))
  {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, "rcl_wait_set_t");
  if (!wait_set) {
    return NULL;
  }

  rcl_context_t * context = (rcl_context_t *)PyCapsule_GetPointer(pycontext, "rcl_context_t");
  if (!context) {
    return NULL;
  }

  rcl_ret_t ret = rcl_wait_set_init(
    wait_set,
    number_of_subscriptions,
    number_of_guard_conditions,
    number_of_timers,
    number_of_clients,
    number_of_services,
    number_of_events,
    context,
    rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to initialize wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Clear all the pointers in the wait set
/**
 * Raises RuntimeError if any rcl error occurs
 *
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \return NULL
 */
static PyObject *
rclpy_wait_set_clear_entities(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;

  if (!PyArg_ParseTuple(args, "O", &pywait_set)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, "rcl_wait_set_t");
  if (!wait_set) {
    return NULL;
  }
  rcl_ret_t ret = rcl_wait_set_clear(wait_set);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to clear wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_TRUE;
}

/// Add an entity to the wait set structure
/**
 * Raises RuntimeError if the entity type is unknown or any rcl error occurrs
 *
 * \param[in] entity_type string defining the entity ["subscription, client, service"]
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \param[in] pyentity Capsule pointing to the entity to add
 * \return Index in waitset entity was added at
 */
static PyObject *
rclpy_wait_set_add_entity(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * entity_type;
  PyObject * pywait_set;
  PyObject * pyentity;
  size_t index;

  if (!PyArg_ParseTuple(args, "zOO", &entity_type, &pywait_set, &pyentity)) {
    return NULL;
  }
  rcl_ret_t ret;
  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, "rcl_wait_set_t");
  if (!wait_set) {
    return NULL;
  }
  if (0 == strcmp(entity_type, "subscription")) {
    rclpy_subscription_t * sub =
      (rclpy_subscription_t *)PyCapsule_GetPointer(pyentity, "rclpy_subscription_t");
    if (!sub) {
      return NULL;
    }
    ret = rcl_wait_set_add_subscription(wait_set, &(sub->subscription), &index);
  } else if (0 == strcmp(entity_type, "client")) {
    rclpy_client_t * client =
      (rclpy_client_t *)PyCapsule_GetPointer(pyentity, "rclpy_client_t");
    if (!client) {
      return NULL;
    }
    ret = rcl_wait_set_add_client(wait_set, &(client->client), &index);
  } else if (0 == strcmp(entity_type, "service")) {
    rclpy_service_t * srv =
      (rclpy_service_t *)PyCapsule_GetPointer(pyentity, "rclpy_service_t");
    if (!srv) {
      return NULL;
    }
    ret = rcl_wait_set_add_service(wait_set, &(srv->service), &index);
  } else if (0 == strcmp(entity_type, "timer")) {
    rcl_timer_t * timer =
      (rcl_timer_t *)PyCapsule_GetPointer(pyentity, "rcl_timer_t");
    if (!timer) {
      return NULL;
    }
    ret = rcl_wait_set_add_timer(wait_set, timer, &index);
  } else if (0 == strcmp(entity_type, "guard_condition")) {
    rcl_guard_condition_t * guard_condition =
      (rcl_guard_condition_t *)PyCapsule_GetPointer(pyentity, "rcl_guard_condition_t");
    if (!guard_condition) {
      return NULL;
    }
    ret = rcl_wait_set_add_guard_condition(wait_set, guard_condition, &index);
  } else if (0 == strcmp(entity_type, "event")) {
    rcl_event_t * event = (rcl_event_t *)PyCapsule_GetPointer(pyentity, "rcl_event_t");
    ret = rcl_wait_set_add_event(wait_set, event, &index);
  } else {
    ret = RCL_RET_ERROR;  // to avoid a linter warning
    PyErr_Format(RCLError,
      "'%s' is not a known entity", entity_type);
    return NULL;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to add '%s' to wait set: %s", entity_type, rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  return PyLong_FromSize_t(index);
}

/// Check if an entity in the wait set is ready by its index
/**
 * This must be called after waiting on the wait set.
 * Raises RuntimeError if the entity type is unknown
 * Raises IndexError if the given index is beyond the number of entities in the set
 *
 * \param[in] entity_type string defining the entity ["subscription, client, service"]
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \param[in] pyindex location in the wait set of the entity to check
 * \return True if the entity at the index in the wait set is not NULL
 */
static PyObject *
rclpy_wait_set_is_ready(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * entity_type;
  PyObject * pywait_set;
  PyObject * pyindex;
  size_t index;

  if (!PyArg_ParseTuple(args, "zOO", &entity_type, &pywait_set, &pyindex)) {
    return NULL;
  }

  index = PyLong_AsSize_t(pyindex);
  if (PyErr_Occurred()) {
    // Error already set
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, "rcl_wait_set_t");
  if (!wait_set) {
    return NULL;
  }
  void ** entities = NULL;
  size_t num_entities = 0;
  if (0 == strcmp(entity_type, "subscription")) {
    entities = (void *)wait_set->subscriptions;
    num_entities = wait_set->size_of_subscriptions;
  } else if (0 == strcmp(entity_type, "client")) {
    entities = (void *)wait_set->clients;
    num_entities = wait_set->size_of_clients;
  } else if (0 == strcmp(entity_type, "service")) {
    entities = (void *)wait_set->services;
    num_entities = wait_set->size_of_services;
  } else if (0 == strcmp(entity_type, "timer")) {
    entities = (void *)wait_set->timers;
    num_entities = wait_set->size_of_timers;
  } else if (0 == strcmp(entity_type, "guard_condition")) {
    entities = (void *)wait_set->guard_conditions;
    num_entities = wait_set->size_of_guard_conditions;
  } else if (0 == strcmp(entity_type, "event")) {
    entities = (void *)wait_set->events;
    num_entities = wait_set->size_of_events;
  } else {
    PyErr_Format(RCLError,
      "'%s' is not a known entity", entity_type);
    return NULL;
  }

  if (!entities) {
    PyErr_Format(RCLError, "Wait set '%s' isn't allocated", entity_type);
    return NULL;
  }
  if (index >= num_entities) {
    PyErr_Format(PyExc_IndexError, "%s index too big %zu >= %zu", entity_type, index, num_entities);
    return NULL;
  }
  if (NULL != entities[index]) {
    Py_RETURN_TRUE;
  }
  Py_RETURN_FALSE;
}

/// Destroy the wait set structure
/**
 * Raises RuntimeError if the wait set could not be destroyed
 *
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \return None
 */
static PyObject *
rclpy_destroy_wait_set(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;

  if (!PyArg_ParseTuple(args, "O", &pywait_set)) {
    return NULL;
  }
  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, "rcl_wait_set_t");
  if (!wait_set) {
    return NULL;
  }

  rcl_ret_t ret = rcl_wait_set_fini(wait_set);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to fini wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyMem_Free(wait_set);

  if (PyCapsule_SetPointer(pywait_set, Py_None)) {
    // exception set by PyCapsule_SetPointer
    return NULL;
  }

  Py_RETURN_NONE;
}

#define GET_LIST_READY_ENTITIES(ENTITY_TYPE) \
  size_t idx; \
  size_t idx_max; \
  idx_max = wait_set->size_of_ ## ENTITY_TYPE ## s; \
  const rcl_ ## ENTITY_TYPE ## _t ** struct_ptr = wait_set->ENTITY_TYPE ## s; \
  for (idx = 0; idx < idx_max; idx ++) { \
    if (struct_ptr[idx]) { \
      PyObject * obj = PyLong_FromVoidPtr((void *) struct_ptr[idx]); \
      if (obj) { \
        int rc = PyList_Append(entity_ready_list, obj); \
        Py_DECREF(obj); \
        if (rc != 0) { \
          Py_DECREF(entity_ready_list); \
          return NULL; \
        } \
      } else { \
        Py_DECREF(entity_ready_list); \
        return NULL; \
      } \
    } \
  } \
  return entity_ready_list;
/// Get list of non-null entities in wait set
/**
 * Raises ValueError if pywait_set is not a wait set capsule
 * Raises RuntimeError if the entity type is not known
 *
 * \param[in] entity_type string defining the entity ["subscription, client, service"]
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \return List of wait set entities pointers ready for take
 */
static PyObject *
rclpy_get_ready_entities(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * entity_type;
  PyObject * pywait_set;
  if (!PyArg_ParseTuple(args, "zO", &entity_type, &pywait_set)) {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, "rcl_wait_set_t");
  if (!wait_set) {
    return NULL;
  }

  PyObject * entity_ready_list = PyList_New(0);
  if (0 == strcmp(entity_type, "subscription")) {
    GET_LIST_READY_ENTITIES(subscription)
  } else if (0 == strcmp(entity_type, "client")) {
    GET_LIST_READY_ENTITIES(client)
  } else if (0 == strcmp(entity_type, "service")) {
    GET_LIST_READY_ENTITIES(service)
  } else if (0 == strcmp(entity_type, "timer")) {
    GET_LIST_READY_ENTITIES(timer)
  } else if (0 == strcmp(entity_type, "guard_condition")) {
    GET_LIST_READY_ENTITIES(guard_condition)
  }
  Py_DECREF(entity_ready_list);
  PyErr_Format(RCLError,
    "'%s' is not a known entity", entity_type);
  return NULL;
}

/// Wait until timeout is reached or event happened
/**
 * Raises ValueError if pywait_set is not a wait set capsule
 * Raises RuntimeError if there was an error while waiting
 *
 * This function will wait for an event to happen or for the timeout to expire.
 * A negative timeout means wait forever, a timeout of 0 means no wait
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \param[in] timeout optional time to wait before waking up (in nanoseconds)
 * \return NULL
 */
static PyObject *
rclpy_wait(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pywait_set;
  PY_LONG_LONG timeout = -1;

  if (!PyArg_ParseTuple(args, "O|K", &pywait_set, &timeout)) {
    return NULL;
  }
  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, "rcl_wait_set_t");
  if (!wait_set) {
    return NULL;
  }
  rcl_ret_t ret;

  // Could be a long wait, release the GIL
  Py_BEGIN_ALLOW_THREADS;
  ret = rcl_wait(wait_set, timeout);
  Py_END_ALLOW_THREADS;

  if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
    PyErr_Format(RCLError,
      "Failed to wait on wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Take a raw message from a given subscription (internal- for rclpy_take with raw=True)
/**
 * \param[in] rcl subscription pointer pointing to the subscription to process the message
 * \return Python byte array with the raw serialized message contents
 */
static PyObject *
rclpy_take_raw(rcl_subscription_t * subscription)
{
  // Create a serialized message object
  rcl_serialized_message_t msg = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_ret_t ret = rmw_serialized_message_init(&msg, 0u, &allocator);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to initialize message: %s", rcl_get_error_string().str);
    rcl_reset_error();
    rmw_ret_t r_fini = rmw_serialized_message_fini(&msg);
    if (r_fini != RMW_RET_OK) {
      PyErr_Format(RCLError, "Failed to deallocate message buffer: %d", r_fini);
    }
    return NULL;
  }

  ret = rcl_take_serialized_message(subscription, &msg, NULL, NULL);
  if (ret != RMW_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to take_serialized from a subscription: %s", rcl_get_error_string().str);
    rcl_reset_error();
    rmw_ret_t r_fini = rmw_serialized_message_fini(&msg);
    if (r_fini != RMW_RET_OK) {
      PyErr_Format(RCLError, "Failed to deallocate message buffer: %d", r_fini);
    }
    return NULL;
  }
  PyObject * python_bytes = PyBytes_FromStringAndSize((char *)(msg.buffer), msg.buffer_length);
  rmw_ret_t r_fini = rmw_serialized_message_fini(&msg);
  if (r_fini != RMW_RET_OK) {
    PyErr_Format(RCLError, "Failed to deallocate message buffer: %d", r_fini);
    if (python_bytes) {
      Py_DECREF(python_bytes);
    }
    return NULL;
  }
  return python_bytes;
}

/// Take a message from a given subscription
/**
 * \param[in] pysubscription Capsule pointing to the subscription to process the message
 * \param[in] pymsg_type Instance of the message type to take
 * \return Python message with all fields populated with received message
 */
static PyObject *
rclpy_take(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pysubscription;
  PyObject * pymsg_type;
  PyObject * pyraw;

  if (!PyArg_ParseTuple(args, "OOO", &pysubscription, &pymsg_type, &pyraw)) {
    return NULL;
  }
  if (!PyCapsule_CheckExact(pysubscription)) {
    PyErr_Format(PyExc_TypeError, "Argument pysubscription is not a valid PyCapsule");
    return NULL;
  }

  rclpy_subscription_t * sub =
    (rclpy_subscription_t *)PyCapsule_GetPointer(pysubscription, "rclpy_subscription_t");
  if (!sub) {
    return NULL;
  }

  if (PyObject_IsTrue(pyraw) == 1) {  // raw=True
    return rclpy_take_raw(&(sub->subscription));
  }

  destroy_ros_message_signature * destroy_ros_message = NULL;
  void * taken_msg = rclpy_create_from_py(pymsg_type, &destroy_ros_message);
  if (!taken_msg) {
    return NULL;
  }

  rcl_ret_t ret = rcl_take(&(sub->subscription), taken_msg, NULL, NULL);

  if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    PyErr_Format(RCLError,
      "Failed to take from a subscription: %s", rcl_get_error_string().str);
    rcl_reset_error();
    destroy_ros_message(taken_msg);
    return NULL;
  }

  if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    PyObject * pytaken_msg = rclpy_convert_to_py(taken_msg, pymsg_type);
    destroy_ros_message(taken_msg);
    if (!pytaken_msg) {
      // the function has set the Python error
      return NULL;
    }

    return pytaken_msg;
  }

  // if take failed, just do nothing
  destroy_ros_message(taken_msg);
  Py_RETURN_NONE;
}

/// Take a request from a given service
/**
 * Raises ValueError if pyservice is not a service capsule
 *
 * \param[in] pyservice Capsule pointing to the service to process the request
 * \param[in] pyrequest_type Instance of the message type to take
 * \return List with 2 elements:
 *            first element: a Python request message with all fields populated with received request
 *            second element: a Capsule pointing to the header (rmw_request_id) of the processed request
 */
static PyObject *
rclpy_take_request(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyservice;
  PyObject * pyrequest_type;

  if (!PyArg_ParseTuple(args, "OO", &pyservice, &pyrequest_type)) {
    return NULL;
  }

  rclpy_service_t * srv =
    (rclpy_service_t *)PyCapsule_GetPointer(pyservice, "rclpy_service_t");
  if (!srv) {
    return NULL;
  }

  destroy_ros_message_signature * destroy_ros_message = NULL;
  void * taken_request = rclpy_create_from_py(pyrequest_type, &destroy_ros_message);
  if (!taken_request) {
    return NULL;
  }

  rmw_request_id_t * header = (rmw_request_id_t *)PyMem_Malloc(sizeof(rmw_request_id_t));
  if (!header) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for request header");
    return NULL;
  }
  rcl_ret_t ret = rcl_take_request(&(srv->service), header, taken_request);

  if (ret != RCL_RET_OK && ret != RCL_RET_SERVICE_TAKE_FAILED) {
    PyErr_Format(RCLError,
      "Service failed to take request: %s", rcl_get_error_string().str);
    rcl_reset_error();
    destroy_ros_message(taken_request);
    PyMem_Free(header);
    return NULL;
  }

  if (ret != RCL_RET_SERVICE_TAKE_FAILED) {
    PyObject * pytaken_request = rclpy_convert_to_py(taken_request, pyrequest_type);
    destroy_ros_message(taken_request);
    if (!pytaken_request) {
      PyMem_Free(header);
      return NULL;
    }

    PyObject * pylist = PyList_New(2);
    if (!pylist) {
      PyMem_Free(header);
      Py_DECREF(pytaken_request);
      return NULL;
    }
    PyObject * pyheader = PyCapsule_New(header, "rmw_request_id_t", NULL);
    if (!pyheader) {
      PyMem_Free(header);
      Py_DECREF(pytaken_request);
      Py_DECREF(pylist);
      return NULL;
    }
    PyList_SET_ITEM(pylist, 0, pytaken_request);
    PyList_SET_ITEM(pylist, 1, pyheader);

    return pylist;
  }
  // if take_request failed, just do nothing
  PyMem_Free(header);
  destroy_ros_message(taken_request);
  Py_RETURN_NONE;
}

/// Take a response from a given client
/**
 * Raises ValueError if pyclient is not a client capsule
 *
 * \param[in] pyclient Capsule pointing to the client to process the response
 * \param[in] pyresponse_type Instance of the message type to take
 * \return 2-tuple sequence number and received response or None, None if there is no response
 */
static PyObject *
rclpy_take_response(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclient;
  PyObject * pyresponse_type;

  if (!PyArg_ParseTuple(args, "OO", &pyclient, &pyresponse_type)) {
    return NULL;
  }
  rclpy_client_t * client =
    (rclpy_client_t *)PyCapsule_GetPointer(pyclient, "rclpy_client_t");
  if (!client) {
    return NULL;
  }

  destroy_ros_message_signature * destroy_ros_message = NULL;
  void * taken_response = rclpy_create_from_py(pyresponse_type, &destroy_ros_message);
  if (!taken_response) {
    return NULL;
  }

  rmw_request_id_t * header = (rmw_request_id_t *)PyMem_Malloc(sizeof(rmw_request_id_t));
  if (!header) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for response header");
    return NULL;
  }
  rcl_ret_t ret = rcl_take_response(&(client->client), header, taken_response);
  int64_t sequence = header->sequence_number;
  PyMem_Free(header);

  // Create the tuple to return
  PyObject * pytuple = PyTuple_New(2);
  if (!pytuple) {
    return NULL;
  }

  if (ret != RCL_RET_CLIENT_TAKE_FAILED) {
    PyObject * pytaken_response = rclpy_convert_to_py(taken_response, pyresponse_type);
    destroy_ros_message(taken_response);
    if (!pytaken_response) {
      // the function has set the Python error
      Py_DECREF(pytuple);
      return NULL;
    }

    PyObject * pysequence = PyLong_FromLongLong(sequence);
    if (!pysequence) {
      Py_DECREF(pytaken_response);
      Py_DECREF(pytuple);
      return NULL;
    }
    PyTuple_SET_ITEM(pytuple, 0, pysequence);
    PyTuple_SET_ITEM(pytuple, 1, pytaken_response);
    return pytuple;
  }
  Py_INCREF(Py_None);
  PyTuple_SET_ITEM(pytuple, 0, Py_None);
  Py_INCREF(Py_None);
  PyTuple_SET_ITEM(pytuple, 1, Py_None);
  destroy_ros_message(taken_response);
  return pytuple;
}

/// Status of the the client library
/**
 * \return True if rcl is running properly, False otherwise
 */
static PyObject *
rclpy_ok(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pycontext;

  if (!PyArg_ParseTuple(args, "O", &pycontext)) {
    return NULL;
  }

  rcl_context_t * context = (rcl_context_t *)PyCapsule_GetPointer(pycontext, "rcl_context_t");
  if (!context) {
    return NULL;
  }

  bool ok = rcl_context_is_valid(context);
  if (ok) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

/// Request shutdown of the client library
/**
 * Raises RuntimeError if the library could not be shutdown
 *
 * \return None
 */
static PyObject *
rclpy_shutdown(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pycontext;

  if (!PyArg_ParseTuple(args, "O", &pycontext)) {
    return NULL;
  }

  rcl_context_t * context = (rcl_context_t *)PyCapsule_GetPointer(pycontext, "rcl_context_t");
  if (!context) {
    return NULL;
  }

  rcl_ret_t ret = rcl_shutdown(context);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to shutdown: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  Py_RETURN_NONE;
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
rclpy_get_node_names_and_namespaces(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "O", &pynode)) {
    return NULL;
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }
  rcutils_string_array_t node_names =
    rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t node_namespaces =
    rcutils_get_zero_initialized_string_array();
  rcl_ret_t ret = rcl_get_node_names(node, allocator, &node_names, &node_namespaces);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to get_node_names: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  rcutils_ret_t fini_names_ret;
  rcutils_ret_t fini_namespaces_ret;
  PyObject * pynode_names_and_namespaces = PyList_New(node_names.size);
  if (!pynode_names_and_namespaces) {
    goto cleanup;
  }
  size_t idx;
  for (idx = 0; idx < node_names.size; ++idx) {
    PyObject * pytuple = PyTuple_New(2);
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
    // Steals the reference
    PyList_SET_ITEM(pynode_names_and_namespaces, idx, pytuple);
  }

cleanup:
  fini_names_ret = rcutils_string_array_fini(&node_names);
  fini_namespaces_ret = rcutils_string_array_fini(&node_namespaces);
  if (PyErr_Occurred()) {
    Py_XDECREF(pynode_names_and_namespaces);
    return NULL;
  }
  if (fini_names_ret != RCUTILS_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to destroy node_names: %s", rcl_get_error_string().str);
    Py_DECREF(pynode_names_and_namespaces);
    rcl_reset_error();
    return NULL;
  }
  if (fini_namespaces_ret != RCUTILS_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to destroy node_namespaces: %s", rcl_get_error_string().str);
    Py_DECREF(pynode_names_and_namespaces);
    rcl_reset_error();
    return NULL;
  }

  return pynode_names_and_namespaces;
}

/// Get a list of service names and types associated with the given node name.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RuntimeError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node
 * \param[in] node_name of a remote node to get publishers for
 * \return Python list of tuples.
 *   The first element of each tuple is the service name (string) and the second element
 *   is a list of service types (list of strings).
 */
static PyObject *
rclpy_get_service_names_and_types_by_node(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  char * node_name;
  char * node_namespace;

  if (!PyArg_ParseTuple(args, "Oss", &pynode, &node_name, &node_namespace)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  rcl_names_and_types_t service_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret =
    rcl_get_service_names_and_types_by_node(node, &allocator, node_name, node_namespace,
      &service_names_and_types);
  if (ret != RCL_RET_OK) {
    PyObject * error = RCLError;
    if (ret == RCL_RET_NODE_NAME_NON_EXISTENT) {
      error = NodeNameNonExistentError;
    }
    PyErr_Format(error,
      "Failed to get_service_names_and_types: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pyservice_names_and_types = rclpy_convert_to_py_names_and_types(
    &service_names_and_types);
  if (!rclpy_names_and_types_fini(&service_names_and_types)) {
    Py_XDECREF(pyservice_names_and_types);
    return NULL;
  }
  return pyservice_names_and_types;
}

/// Get a list of service client names and types associated with the given node name.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RuntimeError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node
 * \param[in] node_name of a remote node to get publishers for
 * \return Python list of tuples.
 *   The first element of each tuple is the service name (string) and the second element
 *   is a list of service types (list of strings).
*/
static PyObject *
rclpy_get_client_names_and_types_by_node(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  char * node_name;
  char * node_namespace;

  if (!PyArg_ParseTuple(args, "Oss", &pynode, &node_name, &node_namespace)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  rcl_names_and_types_t client_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret =
    rcl_get_client_names_and_types_by_node(node, &allocator, node_name, node_namespace,
      &client_names_and_types);
  if (ret != RCL_RET_OK) {
    PyObject * error = RCLError;
    if (ret == RCL_RET_NODE_NAME_NON_EXISTENT) {
      error = NodeNameNonExistentError;
    }
    PyErr_Format(error,
      "Failed to get_client_names_and_types: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pyclient_names_and_types = rclpy_convert_to_py_names_and_types(
    &client_names_and_types);
  if (!rclpy_names_and_types_fini(&client_names_and_types)) {
    Py_XDECREF(pyclient_names_and_types);
    return NULL;
  }
  return pyclient_names_and_types;
}

/// Get a list of topic names and types having at least one subscription from the given node name.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RuntimeError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node
 * \param[in] no_demangle if true topic names and types returned will not be demangled
 * \param[in] node_name of a remote node to get subscriptions for
 * \param[in] node_namespace namespace of the remote node
 * \return Python list of tuples.
 *   The first element of each tuple is the topic name (string) and the second element
 *   is a list of topic types (list of strings).
 */
static PyObject *
rclpy_get_subscriber_names_and_types_by_node(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pyno_demangle;
  char * node_name;
  char * node_namespace;
  if (!PyArg_ParseTuple(args, "OOss", &pynode, &pyno_demangle, &node_name, &node_namespace)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }
  bool no_demangle = PyObject_IsTrue(pyno_demangle);
  rcl_names_and_types_t topic_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret =
    rcl_get_subscriber_names_and_types_by_node(node, &allocator, no_demangle, node_name,
      node_namespace, &topic_names_and_types);
  if (ret != RCL_RET_OK) {
    PyObject * error = RCLError;
    if (ret == RCL_RET_NODE_NAME_NON_EXISTENT) {
      error = NodeNameNonExistentError;
    }
    PyErr_Format(error,
      "Failed to get_subscriber_names_and_types: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pytopic_names_and_types = rclpy_convert_to_py_names_and_types(&topic_names_and_types);
  if (!rclpy_names_and_types_fini(&topic_names_and_types)) {
    Py_XDECREF(pytopic_names_and_types);
    return NULL;
  }
  return pytopic_names_and_types;
}

/// Get a list of topic names and types having at least one publisher from the given node name.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RuntimeError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node
 * \param[in] no_demangle if true topic names and types returned will not be demangled
 * \param[in] node_name of a remote node to get publishers for
 * \param[in] node_namespace namespace of the remote node
 * \return Python list of tuples.
 *   The first element of each tuple is the topic name (string) and the second element
 *   is a list of topic types (list of strings).
 */
static PyObject *
rclpy_get_publisher_names_and_types_by_node(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pyno_demangle;
  char * node_name;
  char * node_namespace;
  if (!PyArg_ParseTuple(args, "OOss", &pynode, &pyno_demangle, &node_name, &node_namespace)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }
  bool no_demangle = PyObject_IsTrue(pyno_demangle);
  rcl_names_and_types_t topic_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret =
    rcl_get_publisher_names_and_types_by_node(node, &allocator, no_demangle, node_name,
      node_namespace, &topic_names_and_types);
  if (ret != RCL_RET_OK) {
    PyObject * error = RCLError;
    if (ret == RCL_RET_NODE_NAME_NON_EXISTENT) {
      error = NodeNameNonExistentError;
    }
    PyErr_Format(error,
      "Failed to get_publisher_names_and_types: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pytopic_names_and_types = rclpy_convert_to_py_names_and_types(&topic_names_and_types);
  if (!rclpy_names_and_types_fini(&topic_names_and_types)) {
    Py_XDECREF(pytopic_names_and_types);
    return NULL;
  }
  return pytopic_names_and_types;
}

/// Get a list of topics associated with the given node name.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RuntimeError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node
 * \param[in] no_demangle if true topic names and types returned will not be demangled
 * \return Python list of tuples.
 *   The first element of each tuple is the topic name (string) and the second element
 *   is a list of topic types (list of strings).
 */
static PyObject *
rclpy_get_topic_names_and_types(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pyno_demangle;

  if (!PyArg_ParseTuple(args, "OO", &pynode, &pyno_demangle)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }
  bool no_demangle = PyObject_IsTrue(pyno_demangle);

  rcl_names_and_types_t topic_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret =
    rcl_get_topic_names_and_types(node, &allocator, no_demangle, &topic_names_and_types);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to get_topic_names_and_types: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pytopic_names_and_types = PyList_New(topic_names_and_types.names.size);
  if (!pytopic_names_and_types) {
    goto cleanup;
  }
  size_t i;
  for (i = 0; i < topic_names_and_types.names.size; ++i) {
    PyObject * pytuple = PyTuple_New(2);
    if (!pytuple) {
      goto cleanup;
    }
    PyObject * pytopic_name = PyUnicode_FromString(topic_names_and_types.names.data[i]);
    if (!pytopic_name) {
      Py_DECREF(pytuple);
      goto cleanup;
    }
    PyTuple_SET_ITEM(pytuple, 0, pytopic_name);
    PyObject * pytypes_list = PyList_New(topic_names_and_types.types[i].size);
    if (!pytypes_list) {
      Py_DECREF(pytuple);
      goto cleanup;
    }
    size_t j;
    for (j = 0; j < topic_names_and_types.types[i].size; ++j) {
      PyObject * pytopic_type = PyUnicode_FromString(topic_names_and_types.types[i].data[j]);
      if (!pytopic_type) {
        Py_DECREF(pytuple);
        Py_DECREF(pytypes_list);
        goto cleanup;
      }
      PyList_SET_ITEM(pytypes_list, j, pytopic_type);
    }
    PyTuple_SET_ITEM(pytuple, 1, pytypes_list);
    PyList_SET_ITEM(pytopic_names_and_types, i, pytuple);
  }

cleanup:
  ret = rcl_names_and_types_fini(&topic_names_and_types);
  if (PyErr_Occurred()) {
    Py_XDECREF(pytopic_names_and_types);
    return NULL;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to destroy topic_names_and_types: %s", rcl_get_error_string().str);
    Py_DECREF(pytopic_names_and_types);
    rcl_reset_error();
    return NULL;
  }

  return pytopic_names_and_types;
}

/// Get a list of services associated with the given node name.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RuntimeError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node
 * \return Python list of tuples.
 *   The first element of each tuple is the service name (string) and the second element
 *   is a list of service types (list of strings).
 */
static PyObject *
rclpy_get_service_names_and_types(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "O", &pynode)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  rcl_names_and_types_t service_names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret =
    rcl_get_service_names_and_types(node, &allocator, &service_names_and_types);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to get_service_names_and_types: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pyservice_names_and_types = PyList_New(service_names_and_types.names.size);
  if (!pyservice_names_and_types) {
    goto cleanup;
  }
  size_t i;
  for (i = 0; i < service_names_and_types.names.size; ++i) {
    PyObject * pytuple = PyTuple_New(2);
    if (!pytuple) {
      goto cleanup;
    }
    PyObject * pyservice_name = PyUnicode_FromString(service_names_and_types.names.data[i]);
    if (!pyservice_name) {
      Py_DECREF(pytuple);
      goto cleanup;
    }
    PyTuple_SET_ITEM(pytuple, 0, pyservice_name);
    PyObject * pytypes_list = PyList_New(service_names_and_types.types[i].size);
    if (!pytypes_list) {
      Py_DECREF(pytuple);
      goto cleanup;
    }
    size_t j;
    for (j = 0; j < service_names_and_types.types[i].size; ++j) {
      PyObject * pyservice_type = PyUnicode_FromString(service_names_and_types.types[i].data[j]);
      if (!pyservice_type) {
        Py_DECREF(pytuple);
        Py_DECREF(pyservice_name);
        Py_DECREF(pytypes_list);
        goto cleanup;
      }
      PyList_SET_ITEM(pytypes_list, j, pyservice_type);
    }
    PyTuple_SET_ITEM(pytuple, 1, pytypes_list);
    PyList_SET_ITEM(pyservice_names_and_types, i, pytuple);
  }

cleanup:
  ret = rcl_names_and_types_fini(&service_names_and_types);
  if (PyErr_Occurred()) {
    Py_XDECREF(pyservice_names_and_types);
    return NULL;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to destroy service_names_and_types: %s", rcl_get_error_string().str);
    Py_DECREF(pyservice_names_and_types);
    rcl_reset_error();
    return NULL;
  }

  return pyservice_names_and_types;
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
  rcl_duration_t * duration = (rcl_duration_t *)PyCapsule_GetPointer(pyobject, "rcl_duration_t");
  if (!duration) {
    return false;
  }
  *out_time = (rmw_time_t) {
    RCL_NS_TO_S(duration->nanoseconds),
    duration->nanoseconds % (1000LL * 1000LL * 1000LL)
  };
  return true;
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

  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)PyMem_Malloc(sizeof(rmw_qos_profile_t));

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
  if (!_convert_py_duration_to_rmw_time(pyqos_liveliness_lease_duration,
    &qos_profile->liveliness_lease_duration))
  {
    return NULL;
  }

  qos_profile->avoid_ros_namespace_conventions = avoid_ros_namespace_conventions;
  PyObject * pyqos_profile = PyCapsule_New(qos_profile, "rmw_qos_profile_t", NULL);
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

  rmw_qos_profile_t * profile = (rmw_qos_profile_t *)PyCapsule_GetPointer(
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
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameters")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_parameters);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameter_events")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_parameter_events);
  } else {
    PyErr_Format(RCLError,
      "Requested unknown rmw_qos_profile: '%s'", pyrmw_profile);
    return NULL;
  }
  return pyqos_profile;
}

/// Manually assert that an entity is alive.
/**
  * When using RMW_QOS_POLICY_MANUAL_BY_*, the application must call this function at least as
  * often as the qos policy liveliness_lease_duration.
  * The passed entity can be a Publisher or a Node.
  *
  * Raises RuntimeError on failure to assert liveliness
  * Raises TypeError if passed object is not a valid Publisher or Node
  *
  * \param[in] pyentity A capsule containing an rcl_node_t or rcl_publisher_t
  * \return None
  */
static PyObject *
rclpy_assert_liveliness(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyentity;

  if (!PyArg_ParseTuple(args, "O", &pyentity)) {
    return NULL;
  }

  if (PyCapsule_IsValid(pyentity, "rcl_node_t")) {
    rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pyentity, "rcl_node_t");
    if (RCL_RET_OK != rcl_node_assert_liveliness(node)) {
      PyErr_Format(RCLError,
        "Failed assert liveliness on the Node: %s", rcl_get_error_string().str);
      rcl_reset_error();
      return NULL;
    }
  } else if (PyCapsule_IsValid(pyentity, "rclpy_publisher_t")) {
    rclpy_publisher_t * publisher = (rclpy_publisher_t *)PyCapsule_GetPointer(
      pyentity, "rclpy_publisher_t");
    if (RCL_RET_OK != rcl_publisher_assert_liveliness(&publisher->publisher)) {
      PyErr_Format(RCLError,
        "Failed to assert liveliness on the Publisher: %s", rcl_get_error_string().str);
      rcl_reset_error();
      return NULL;
    }
  } else {
    PyErr_Format(PyExc_TypeError,
      "Passed capsule is not a valid Node or Publisher.");
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Destructor for a time point
void
_rclpy_destroy_time_point(PyObject * pycapsule)
{
  PyMem_Free(PyCapsule_GetPointer(pycapsule, "rcl_time_point_t"));
}

/// Create a time point
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises RuntimeError on initialization failure
 * Raises TypeError if argument of invalid type
 * Raises OverflowError if nanoseconds argument cannot be converted to uint64_t
 *
 * \param[in] nanoseconds unsigned PyLong object storing the nanoseconds value
 *   of the time point in a 64-bit unsigned integer
 * \param[in] clock_type enum of type ClockType
 * \return Capsule of the pointer to the created rcl_time_point_t * structure, or
 * \return NULL on failure
 */
static PyObject *
rclpy_create_time_point(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pylong_nanoseconds;
  unsigned PY_LONG_LONG clock_type;

  if (!PyArg_ParseTuple(args, "OK", &pylong_nanoseconds, &clock_type)) {
    return NULL;
  }

  unsigned PY_LONG_LONG nanoseconds = PyLong_AsUnsignedLongLong(pylong_nanoseconds);
  if (PyErr_Occurred()) {
    return NULL;
  }

  rcl_time_point_t * time_point = (rcl_time_point_t *) PyMem_Malloc(sizeof(rcl_time_point_t));
  if (!time_point) {
    PyErr_Format(RCLError, "Failed to allocate memory for time point.");
    return NULL;
  }

  time_point->nanoseconds = nanoseconds;
  time_point->clock_type = clock_type;

  return PyCapsule_New(time_point, "rcl_time_point_t", _rclpy_destroy_time_point);
}

/// Returns the nanoseconds value of the time point
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises ValueError if pytime_point is not a time point capsule
 *
 * \param[in] pytime_point Capsule pointing to the time point
 * \return NULL on failure:
 *         PyLong integer in nanoseconds on success
 */
static PyObject *
rclpy_time_point_get_nanoseconds(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pytime_point;
  if (!PyArg_ParseTuple(args, "O", &pytime_point)) {
    return NULL;
  }

  rcl_time_point_t * time_point = (rcl_time_point_t *)PyCapsule_GetPointer(
    pytime_point, "rcl_time_point_t");
  if (!time_point) {
    return NULL;
  }

  return PyLong_FromUnsignedLongLong(time_point->nanoseconds);
}

/// Destructor for a duration
void
_rclpy_destroy_duration(PyObject * pycapsule)
{
  PyMem_Free(PyCapsule_GetPointer(pycapsule, "rcl_duration_t"));
}

/// Create a duration
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises RuntimeError on initialization failure
 * Raises TypeError if argument of invalid type
 * Raises OverflowError if nanoseconds argument cannot be converted to uint64_t
 *
 * \param[in] nanoseconds unsigned PyLong object storing the nanoseconds value
 *   of the duration in a 64-bit signed integer
 * \return Capsule of the pointer to the created rcl_duration_t * structure, or
 * \return NULL on failure
 */
static PyObject *
rclpy_create_duration(PyObject * Py_UNUSED(self), PyObject * args)
{
  PY_LONG_LONG nanoseconds;

  if (!PyArg_ParseTuple(args, "L", &nanoseconds)) {
    return NULL;
  }

  rcl_duration_t * duration = (rcl_duration_t *) PyMem_Malloc(sizeof(rcl_duration_t));
  if (!duration) {
    PyErr_Format(RCLError, "Failed to allocate memory for duration.");
    return NULL;
  }

  duration->nanoseconds = nanoseconds;

  return PyCapsule_New(duration, "rcl_duration_t", _rclpy_destroy_duration);
}

/// Returns the nanoseconds value of the duration
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises ValueError if pyduration is not a duration capsule
 *
 * \param[in] pyduration Capsule pointing to the duration
 * \return NULL on failure:
 *         PyLong integer in nanoseconds on success
 */
static PyObject *
rclpy_duration_get_nanoseconds(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyduration;
  if (!PyArg_ParseTuple(args, "O", &pyduration)) {
    return NULL;
  }

  rcl_duration_t * duration = (rcl_duration_t *)PyCapsule_GetPointer(
    pyduration, "rcl_duration_t");
  if (!duration) {
    return NULL;
  }

  return PyLong_FromLongLong(duration->nanoseconds);
}

/// Create a clock
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises RuntimeError on initialization failure
 * Raises TypeError if argument of invalid type
 *
 * This function creates a Clock object of the specified type
 * \param[in] clock_type enum of type ClockType
 * \return NULL on failure
 *         Capsule to an rcl_clock_t object
 */
static PyObject *
rclpy_create_clock(PyObject * Py_UNUSED(self), PyObject * args)
{
  unsigned PY_LONG_LONG clock_type;

  if (!PyArg_ParseTuple(args, "K", &clock_type)) {
    return NULL;
  }

  rcl_clock_t * clock = (rcl_clock_t *)PyMem_Malloc(sizeof(rcl_clock_t));
  if (!clock) {
    PyErr_Format(RCLError, "Failed to allocate memory for clock.");
    return NULL;
  }
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_clock_init(clock_type, clock, &allocator);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to initialize clock: %s", rcl_get_error_string().str);
    rcl_reset_error();
    PyMem_Free(clock);
    return NULL;
  }

  return PyCapsule_New(clock, "rcl_clock_t", _rclpy_destroy_clock);
}

/// Returns the current value of the clock
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises ValueError if pyclock is not a clock capsule
 * Raises RuntimeError if the clock's value cannot be retrieved
 *
 * \param[in] pyclock Capsule pointing to the clock
 * \return NULL on failure:
 *         Capsule to a time point on success
 */
static PyObject *
rclpy_clock_get_now(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclock;
  if (!PyArg_ParseTuple(args, "O", &pyclock)) {
    return NULL;
  }

  rcl_clock_t * clock = (rcl_clock_t *)PyCapsule_GetPointer(
    pyclock, "rcl_clock_t");
  if (!clock) {
    return NULL;
  }

  rcl_time_point_t * time_point = (rcl_time_point_t *) PyMem_Malloc(sizeof(rcl_time_point_t));
  if (!time_point) {
    PyErr_Format(RCLError, "Failed to allocate memory for time point.");
    return NULL;
  }
  time_point->clock_type = clock->type;

  rcl_ret_t ret = rcl_clock_get_now(clock, &time_point->nanoseconds);

  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to get current value of clock: %s", rcl_get_error_string().str);
    rcl_reset_error();
    PyMem_Free(time_point);
    return NULL;
  }

  return PyCapsule_New(time_point, "rcl_time_point_t", _rclpy_destroy_time_point);
}

/// Returns if a clock using ROS time has the ROS time override enabled.
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises ValueError if pyclock is not a clock capsule
 * Raises RuntimeError if the clock's ROS time override state cannot be retrieved
 *
 * \param[in] pyclock Capsule pointing to the clock
 * \return NULL on failure
 *         True if enabled
 *         False if not enabled
 */
static PyObject *
rclpy_clock_get_ros_time_override_is_enabled(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclock;
  if (!PyArg_ParseTuple(args, "O", &pyclock)) {
    return NULL;
  }

  rcl_clock_t * clock = (rcl_clock_t *)PyCapsule_GetPointer(
    pyclock, "rcl_clock_t");
  if (!clock) {
    return NULL;
  }

  bool is_enabled;
  rcl_ret_t ret = rcl_is_enabled_ros_time_override(clock, &is_enabled);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to get if ROS time override is enabled for clock: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  if (is_enabled) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

/// Set if a clock using ROS time has the ROS time override enabled.
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises ValueError if pyclock is not a clock capsule
 * Raises RuntimeError if the clock's ROS time override cannot be set
 *
 * \param[in] pyclock Capsule pointing to the clock to set
 * \param[in] enabled Override state to set
 * \return NULL on failure
 *         None on success
 */
static PyObject *
rclpy_clock_set_ros_time_override_is_enabled(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclock;
  int enabled;
  if (!PyArg_ParseTuple(args, "Op", &pyclock, &enabled)) {
    return NULL;
  }

  rcl_clock_t * clock = (rcl_clock_t *)PyCapsule_GetPointer(
    pyclock, "rcl_clock_t");
  if (!clock) {
    return NULL;
  }

  rcl_ret_t ret;
  if (enabled) {
    ret = rcl_enable_ros_time_override(clock);
  } else {
    ret = rcl_disable_ros_time_override(clock);
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to set ROS time override for clock: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  if (PyErr_Occurred()) {
    // Time jump callbacks raised
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Set the ROS time override for a clock using ROS time.
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises ValueError if pyclock is not a clock capsule, or
 * pytime_point is not a time point capsule
 * Raises RuntimeError if the clock's ROS time override cannot be set
 *
 * \param[in] pyclock Capsule pointing to the clock to set
 * \param[in] pytime_point Capsule pointing to the time point
 * \return NULL on failure
 *         None on success
 */
static PyObject *
rclpy_clock_set_ros_time_override(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclock;
  PyObject * pytime_point;
  if (!PyArg_ParseTuple(args, "OO", &pyclock, &pytime_point)) {
    return NULL;
  }

  rcl_clock_t * clock = (rcl_clock_t *)PyCapsule_GetPointer(
    pyclock, "rcl_clock_t");
  if (!clock) {
    return NULL;
  }

  rcl_time_point_t * time_point = (rcl_time_point_t *)PyCapsule_GetPointer(
    pytime_point, "rcl_time_point_t");
  if (!time_point) {
    return NULL;
  }

  rcl_ret_t ret = rcl_set_ros_time_override(clock, time_point->nanoseconds);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to set ROS time override for clock: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  if (PyErr_Occurred()) {
    // Time jump callbacks raised
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Called when a time jump occurs.
void
_rclpy_on_time_jump(
  const struct rcl_time_jump_t * time_jump,
  bool before_jump,
  void * user_data)
{
  if (PyErr_Occurred()) {
    // An earlier time jump callback already raised an exception
    return;
  }
  PyObject * pyjump_handle = user_data;
  if (before_jump) {
    // Call pre jump callback with no arguments
    PyObject * pycallback = PyObject_GetAttrString(pyjump_handle, "_pre_callback");
    if (!pycallback) {
      // Raised
      return;
    }
    if (Py_None == pycallback) {
      Py_DECREF(pycallback);
      // Callback is None
      return;
    }
    // May set exception
    PyObject_CallObject(pycallback, NULL);
    Py_DECREF(pycallback);
  } else {
    // Call post jump callback with JumpInfo as an argument
    PyObject * pycallback = PyObject_GetAttrString(pyjump_handle, "_post_callback");
    if (!pycallback) {
      // Raised
      return;
    }
    if (Py_None == pycallback) {
      Py_DECREF(pycallback);
      // Callback is None
      return;
    }
    // Build python dictionary with time jump info
    const char * clock_change;
    switch (time_jump->clock_change) {
      case RCL_ROS_TIME_NO_CHANGE:
        clock_change = "RCL_ROS_TIME_NO_CHANGE";
        break;
      case RCL_ROS_TIME_ACTIVATED:
        clock_change = "RCL_ROS_TIME_ACTIVATED";
        break;
      case RCL_ROS_TIME_DEACTIVATED:
        clock_change = "RCL_ROS_TIME_DEACTIVATED";
        break;
      case RCL_SYSTEM_TIME_NO_CHANGE:
        clock_change = "RCL_SYSTEM_TIME_NO_CHANGE";
        break;
      default:
        PyErr_Format(RCLError, "Unknown time jump type %d", time_jump->clock_change);
        Py_DECREF(pycallback);
        return;
    }
    PY_LONG_LONG delta = time_jump->delta.nanoseconds;
    PyObject * pyjump_info = Py_BuildValue(
      "{zzzL}", "clock_change", clock_change, "delta", delta);
    if (!pyjump_info) {
      Py_DECREF(pycallback);
      return;
    }
    PyObject * pyargs = PyTuple_Pack(1, pyjump_info);
    if (!pyargs) {
      Py_DECREF(pyjump_info);
      Py_DECREF(pycallback);
      return;
    }
    // May set exception
    PyObject_CallObject(pycallback, pyargs);
    Py_DECREF(pyjump_info);
    Py_DECREF(pyargs);
    Py_DECREF(pycallback);
  }
}

/// Add a time jump callback to a clock.
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises ValueError if pyclock is not a clock capsule, or
 * any argument is invalid
 * Raises RuntimeError if the callback cannot be added
 *
 * \param[in] pyclock Capsule pointing to the clock to set
 * \param[in] pyjump_handle Instance of rclpy.clock.JumpHandle
 * \param[in] on_clock_change True if callback should be called when ROS time is toggled
 * \param[in] min_forward minimum nanoseconds to trigger forward jump callback
 * \param[in] min_backward minimum negative nanoseconds to trigger backward jump callback
 * \return NULL on failure
 *         None on success
 */
static PyObject *
rclpy_add_clock_callback(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclock;
  PyObject * pyjump_handle;
  int on_clock_change;
  PY_LONG_LONG min_forward;
  PY_LONG_LONG min_backward;
  if (!PyArg_ParseTuple(args, "OOpLL", &pyclock, &pyjump_handle, &on_clock_change, &min_forward,
    &min_backward))
  {
    return NULL;
  }

  rcl_clock_t * clock = (rcl_clock_t *)PyCapsule_GetPointer(
    pyclock, "rcl_clock_t");
  if (!clock) {
    return NULL;
  }

  rcl_jump_threshold_t threshold;
  threshold.on_clock_change = on_clock_change;
  threshold.min_forward.nanoseconds = min_forward;
  threshold.min_backward.nanoseconds = min_backward;

  rcl_ret_t ret = rcl_clock_add_jump_callback(
    clock, threshold, _rclpy_on_time_jump, pyjump_handle);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to add time jump callback: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Remove a time jump callback from a clock.
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises ValueError if pyclock is not a clock capsule, or
 * any argument is invalid
 * Raises RuntimeError if the callback cannot be added
 *
 * \param[in] pyclock Capsule pointing to the clock to set
 * \param[in] pyjump_handle Instance of rclpy.clock.JumpHandle
 * \return NULL on failure
 *         None on success
 */
static PyObject *
rclpy_remove_clock_callback(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyclock;
  PyObject * pyjump_handle;
  if (!PyArg_ParseTuple(args, "OO", &pyclock, &pyjump_handle)) {
    return NULL;
  }

  rcl_clock_t * clock = (rcl_clock_t *)PyCapsule_GetPointer(
    pyclock, "rcl_clock_t");
  if (!clock) {
    return NULL;
  }

  rcl_ret_t ret = rcl_clock_remove_jump_callback(
    clock, _rclpy_on_time_jump, pyjump_handle);
  if (ret != RCL_RET_OK) {
    PyErr_Format(RCLError,
      "Failed to remove time jump callback: %s", rcl_get_error_string().str);
    rcl_reset_error();
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
        PyErr_Format(RCLError, "Error reading node_paramters from internal dict");
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
      PyObject * py_param = _parameter_from_rcl_variant(py_param_name,
          &node_params.parameter_values[ii], parameter_cls, parameter_type_cls);
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
  const rcl_arguments_t * args, rcl_allocator_t allocator, PyObject * parameter_cls,
  PyObject * parameter_type_cls, PyObject * params_by_node_name)
{
  rcl_params_t * params = NULL;
  if (RCL_RET_OK != rcl_arguments_get_param_overrides(args, &params)) {
    PyErr_Format(RCLError, "Failed to get parameters overrides: %s",
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
rclpy_get_node_parameters(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * parameter_cls;
  PyObject * node_capsule;
  if (!PyArg_ParseTuple(args, "OO", &parameter_cls, &node_capsule)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(node_capsule, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  PyObject * params_by_node_name = PyDict_New();
  if (!params_by_node_name) {
    return NULL;
  }

  if (!PyObject_HasAttrString(parameter_cls, "Type")) {
    PyErr_Format(RCLError, "Parameter class is missing 'Type' attribute");
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
        &(node->context->global_arguments), allocator, parameter_cls,
        parameter_type_cls, params_by_node_name))
    {
      Py_DECREF(parameter_type_cls);
      Py_DECREF(params_by_node_name);
      return NULL;
    }
  }

  if (!_parse_param_overrides(&(node_options->arguments), allocator, parameter_cls,
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
    node_name_with_namespace = rcutils_format_string(allocator, "%s%s",
        node_namespace, rcl_node_get_name(node));
  } else {
    node_name_with_namespace = rcutils_format_string(allocator, "%s/%s",
        node_namespace, rcl_node_get_name(node));
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

  PyObject * current_key, * current_value;
  Py_ssize_t current_index = 0;
  while (PyDict_Next(params_by_node_name, &current_index, &current_key, &current_value)) {
    // TODO(cottsay) implement further wildcard matching
    if (PyObject_RichCompareBool(current_key, py_wildcard_name, Py_EQ) == 1 ||
      PyObject_RichCompareBool(current_key, py_node_name_with_namespace, Py_EQ) == 1)
    {
      if (-1 == PyDict_Update(node_params, current_value)) {
        Py_DECREF(node_params);
        node_params = NULL;
        break;
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
    "rclpy_create_context", rclpy_create_context, METH_VARARGS,
    "Create a rcl context."
  },
  {
    "rclpy_init", rclpy_init, METH_VARARGS,
    "Initialize RCL."
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
    "rclpy_get_node_name", rclpy_get_node_name, METH_VARARGS,
    "Get the name of a node."
  },
  {
    "rclpy_get_node_namespace", rclpy_get_node_namespace, METH_VARARGS,
    "Get the namespace of a node."
  },
  {
    "rclpy_get_node_logger_name", rclpy_get_node_logger_name, METH_VARARGS,
    "Get the logger name associated with a node."
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
    "rclpy_expand_topic_name", rclpy_expand_topic_name, METH_VARARGS,
    "Expand a topic name."
  },
  {
    "rclpy_get_validation_error_for_topic_name",
    rclpy_get_validation_error_for_topic_name, METH_VARARGS,
    "Get the error message and invalid index of a topic name or None if valid."
  },
  {
    "rclpy_get_validation_error_for_full_topic_name",
    rclpy_get_validation_error_for_full_topic_name, METH_VARARGS,
    "Get the error message and invalid index of a full topic name or None if valid."
  },
  {
    "rclpy_get_validation_error_for_namespace",
    rclpy_get_validation_error_for_namespace, METH_VARARGS,
    "Get the error message and invalid index of a namespace or None if valid."
  },
  {
    "rclpy_get_validation_error_for_node_name",
    rclpy_get_validation_error_for_node_name, METH_VARARGS,
    "Get the error message and invalid index of a node name or None if valid."
  },
  {
    "rclpy_create_publisher", rclpy_create_publisher, METH_VARARGS,
    "Create a Publisher."
  },
  {
    "rclpy_create_subscription", rclpy_create_subscription, METH_VARARGS,
    "Create a Subscription."
  },
  {
    "rclpy_create_service", rclpy_create_service, METH_VARARGS,
    "Create a Service."
  },
  {
    "rclpy_create_client", rclpy_create_client, METH_VARARGS,
    "Create a Client."
  },
  {
    "rclpy_create_timer", rclpy_create_timer, METH_VARARGS,
    "Create a Timer."
  },
  {
    "rclpy_create_event", rclpy_create_event, METH_VARARGS,
    "Create an Event."
  },

  {
    "rclpy_create_guard_condition", rclpy_create_guard_condition, METH_VARARGS,
    "Create a general purpose guard_condition."
  },
  {
    "rclpy_trigger_guard_condition", rclpy_trigger_guard_condition, METH_VARARGS,
    "Trigger a general purpose guard_condition."
  },

  {
    "rclpy_publish", rclpy_publish, METH_VARARGS,
    "Publish a message."
  },
  {
    "rclpy_publisher_get_subscription_count", rclpy_publisher_get_subscription_count, METH_VARARGS,
    "Count subscribers from a publisher."
  },
  {
    "rclpy_send_request", rclpy_send_request, METH_VARARGS,
    "Send a request."
  },
  {
    "rclpy_send_response", rclpy_send_response, METH_VARARGS,
    "Send a response."
  },

  {
    "rclpy_service_server_is_available", rclpy_service_server_is_available, METH_VARARGS,
    "Return true if the service server is available."
  },

  {
    "rclpy_get_zero_initialized_wait_set", rclpy_get_zero_initialized_wait_set, METH_NOARGS,
    "rclpy_get_zero_initialized_wait_set."
  },

  {
    "rclpy_wait_set_init", rclpy_wait_set_init, METH_VARARGS,
    "rclpy_wait_set_init."
  },

  {
    "rclpy_wait_set_clear_entities", rclpy_wait_set_clear_entities, METH_VARARGS,
    "rclpy_wait_set_clear_entities."
  },

  {
    "rclpy_wait_set_add_entity", rclpy_wait_set_add_entity, METH_VARARGS,
    "rclpy_wait_set_add_entity."
  },

  {
    "rclpy_wait_set_is_ready", rclpy_wait_set_is_ready, METH_VARARGS,
    "rclpy_wait_set_is_ready."
  },

  {
    "rclpy_destroy_wait_set", rclpy_destroy_wait_set, METH_VARARGS,
    "rclpy_destroy_wait_set."
  },

  {
    "rclpy_get_ready_entities", rclpy_get_ready_entities, METH_VARARGS,
    "List non null entities in wait set."
  },

  {
    "rclpy_reset_timer", rclpy_reset_timer, METH_VARARGS,
    "Reset a timer."
  },

  {
    "rclpy_call_timer", rclpy_call_timer, METH_VARARGS,
    "Call a timer and starts counting again."
  },

  {
    "rclpy_change_timer_period", rclpy_change_timer_period, METH_VARARGS,
    "Set the period of a timer."
  },

  {
    "rclpy_is_timer_ready", rclpy_is_timer_ready, METH_VARARGS,
    "Check if a timer as reached timeout."
  },

  {
    "rclpy_cancel_timer", rclpy_cancel_timer, METH_VARARGS,
    "Cancel a timer."
  },

  {
    "rclpy_is_timer_canceled", rclpy_is_timer_canceled, METH_VARARGS,
    "Check if a timer is canceled."
  },

  {
    "rclpy_time_until_next_call", rclpy_time_until_next_call, METH_VARARGS,
    "Get the remaining time before timer is ready."
  },

  {
    "rclpy_time_since_last_call", rclpy_time_since_last_call, METH_VARARGS,
    "Get the elapsed time since last timer call."
  },

  {
    "rclpy_get_timer_period", rclpy_get_timer_period, METH_VARARGS,
    "Get the period of a timer."
  },

  {
    "rclpy_wait", rclpy_wait, METH_VARARGS,
    "rclpy_wait."
  },

  {
    "rclpy_take", rclpy_take, METH_VARARGS,
    "rclpy_take."
  },

  {
    "rclpy_take_request", rclpy_take_request, METH_VARARGS,
    "rclpy_take_request."
  },

  {
    "rclpy_take_response", rclpy_take_response, METH_VARARGS,
    "rclpy_take_response."
  },
  {
    "rclpy_take_event", rclpy_take_event, METH_VARARGS,
    "Get the pending data for a ready QoS Event."
  },

  {
    "rclpy_ok", rclpy_ok, METH_VARARGS,
    "rclpy_ok."
  },

  {
    "rclpy_shutdown", rclpy_shutdown, METH_VARARGS,
    "rclpy_shutdown."
  },

  {
    "rclpy_get_node_names_and_namespaces", rclpy_get_node_names_and_namespaces, METH_VARARGS,
    "Get node names and namespaces list from graph API."
  },
  {
    "rclpy_get_node_parameters", rclpy_get_node_parameters, METH_VARARGS,
    "Get the initial parameters for a node from the command line."
  },
  {
    "rclpy_get_subscriber_names_and_types_by_node", rclpy_get_subscriber_names_and_types_by_node,
    METH_VARARGS,
    "Get subscriber list of specified node from graph API."
  },
  {
    "rclpy_get_publisher_names_and_types_by_node", rclpy_get_publisher_names_and_types_by_node,
    METH_VARARGS,
    "Get publisher list of specified node from graph API."
  },
  {
    "rclpy_get_service_names_and_types_by_node", rclpy_get_service_names_and_types_by_node,
    METH_VARARGS,
    "Get service server list of specified node from graph API."
  },
  {
    "rclpy_get_client_names_and_types_by_node", rclpy_get_client_names_and_types_by_node,
    METH_VARARGS,
    "Get a service client list of a specified node from graph API."
  },
  {
    "rclpy_get_topic_names_and_types", rclpy_get_topic_names_and_types, METH_VARARGS,
    "Get topic list from graph API."
  },
  {
    "rclpy_get_service_names_and_types", rclpy_get_service_names_and_types, METH_VARARGS,
    "Get service list from graph API."
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

  {
    "rclpy_create_time_point", rclpy_create_time_point, METH_VARARGS,
    "Create a time point."
  },

  {
    "rclpy_time_point_get_nanoseconds", rclpy_time_point_get_nanoseconds, METH_VARARGS,
    "Get the nanoseconds value of a time point."
  },

  {
    "rclpy_create_duration", rclpy_create_duration, METH_VARARGS,
    "Create a duration."
  },

  {
    "rclpy_duration_get_nanoseconds", rclpy_duration_get_nanoseconds, METH_VARARGS,
    "Get the nanoseconds value of a duration."
  },

  {
    "rclpy_create_clock", rclpy_create_clock, METH_VARARGS,
    "Create a clock."
  },

  {
    "rclpy_clock_get_now", rclpy_clock_get_now, METH_VARARGS,
    "Get the current value of a clock."
  },

  {
    "rclpy_clock_get_ros_time_override_is_enabled", rclpy_clock_get_ros_time_override_is_enabled,
    METH_VARARGS, "Get if a clock using ROS time has the ROS time override enabled."
  },

  {
    "rclpy_clock_set_ros_time_override_is_enabled", rclpy_clock_set_ros_time_override_is_enabled,
    METH_VARARGS, "Set if a clock using ROS time has the ROS time override enabled."
  },

  {
    "rclpy_clock_set_ros_time_override", rclpy_clock_set_ros_time_override, METH_VARARGS,
    "Set the current time of a clock using ROS time."
  },

  {
    "rclpy_add_clock_callback", rclpy_add_clock_callback, METH_VARARGS,
    "Add a time jump callback to a clock."
  },

  {
    "rclpy_remove_clock_callback", rclpy_remove_clock_callback, METH_VARARGS,
    "Remove a time jump callback from a clock."
  },

  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rclpy__doc__,
  "ROS 2 Python client library.");

/// Define the Python module
static struct PyModuleDef _rclpymodule = {
  PyModuleDef_HEAD_INIT,
  "_rclpy",
  rclpy__doc__,
  -1,   /* -1 means that the module keeps state in global variables */
  rclpy_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy(void)
{
  PyObject * m = PyModule_Create(&_rclpymodule);
  if (NULL == m) {
    return NULL;
  }

  RCLError = PyErr_NewExceptionWithDoc(
    "_rclpy.RCLError",
    "Thrown when there is error in calling the rcl layers.",
    PyExc_RuntimeError, NULL);
  if (PyModule_AddObject(m, "RCLError", RCLError)) {
    Py_DECREF(m);
    return NULL;
  }

  RCLInvalidROSArgsError = PyErr_NewExceptionWithDoc(
    "_rclpy.RCLInvalidROSArgsError",
    "Thrown when invalid ROS arguments are found by rcl.",
    RCLError, NULL);
  if (NULL == RCLInvalidROSArgsError) {
    Py_DECREF(m);
    return NULL;
  }
  if (PyModule_AddObject(m, "RCLInvalidROSArgsError", RCLInvalidROSArgsError) != 0) {
    Py_DECREF(m);
    return NULL;
  }

  UnknownROSArgsError = PyErr_NewExceptionWithDoc(
    "_rclpy.UnknownROSArgsError",
    "Thrown when unknown ROS arguments are found.",
    RCLError, NULL);
  if (NULL == UnknownROSArgsError) {
    Py_DECREF(m);
    return NULL;
  }
  if (PyModule_AddObject(m, "UnknownROSArgsError", UnknownROSArgsError) != 0) {
    Py_DECREF(m);
    return NULL;
  }

  NodeNameNonExistentError = PyErr_NewExceptionWithDoc(
    "_rclpy.NodeNameNonExistentError",
    "Thrown when a node name is not found.",
    RCLError, NULL);
  if (PyModule_AddObject(m, "NodeNameNonExistentError", NodeNameNonExistentError)) {
    Py_DECREF(m);
    return NULL;
  }

  if (PyErr_Occurred()) {
    Py_DECREF(m);
    return NULL;
  }
  return m;
}
