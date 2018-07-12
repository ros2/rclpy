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
#include <rcl/rcl.h>
#include <rcl/validate_topic_name.h>
#include <rcutils/strdup.h>
#include <rcutils/types.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/validate_full_topic_name.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>
#include <rosidl_generator_c/message_type_support_struct.h>

#include <signal.h>

static rcl_guard_condition_t * g_sigint_gc_handle;

#ifdef _WIN32
_crt_signal_t g_original_signal_handler = NULL;
#else
sig_t g_original_signal_handler = NULL;
#endif  // _WIN32

/// Catch signals
static void catch_function(int signo)
{
  (void) signo;
  if (NULL != g_sigint_gc_handle) {
    rcl_ret_t ret = rcl_trigger_guard_condition(g_sigint_gc_handle);
    if (ret != RCL_RET_OK) {
      PyErr_Format(PyExc_RuntimeError,
        "Failed to trigger guard_condition: %s", rcl_get_error_string_safe());
      rcl_reset_error();
    }
  }
  if (NULL != g_original_signal_handler) {
    g_original_signal_handler(signo);
  }
}

typedef void * create_ros_message_signature (void);
typedef void destroy_ros_message_signature (void *);
typedef bool convert_from_py_signature (PyObject *, void *);
typedef PyObject * convert_to_py_signature (void *);

static void * get_capsule_pointer(PyObject * pymetaclass, const char * attr)
{
  PyObject * pyattr = PyObject_GetAttrString(pymetaclass, attr);
  if (!pyattr) {
    return NULL;
  }
  void * ptr = PyCapsule_GetPointer(pyattr, NULL);
  Py_DECREF(pyattr);
  return ptr;
}

/// Create a sigint guard condition
/**
 * A successful call will return a list with two elements:
 *
 * - a Capsule with the pointer of the created rcl_guard_condition_t * structure
 * - an integer representing the memory address of the rcl_guard_condition_t
 *
 * Raises RuntimeError if initializing the guard condition fails
 *
 * \return a list with the capsule and memory location, or
 * \return NULL on failure
 */
static PyObject *
rclpy_get_sigint_guard_condition(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcl_guard_condition_t * sigint_gc =
    (rcl_guard_condition_t *)PyMem_Malloc(sizeof(rcl_guard_condition_t));
  *sigint_gc = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t sigint_gc_options = rcl_guard_condition_get_default_options();

  rcl_ret_t ret = rcl_guard_condition_init(sigint_gc, sigint_gc_options);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create guard_condition: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    PyMem_Free(sigint_gc);
    return NULL;
  }
  g_sigint_gc_handle = sigint_gc;
  PyObject * pylist = PyList_New(2);
  PyList_SET_ITEM(pylist, 0, PyCapsule_New(sigint_gc, "rcl_guard_condition_t", NULL));
  PyList_SET_ITEM(pylist, 1, PyLong_FromUnsignedLongLong((uint64_t)&sigint_gc->impl));

  return pylist;
}

/// Create a general purpose guard condition
/**
 * A successful call will return a list with two elements:
 *
 * - a Capsule with the pointer of the created rcl_guard_condition_t * structure
 * - an integer representing the memory address of the rcl_guard_condition_t
 *
 * Raises RuntimeError if initializing the guard condition fails
 *
 * \remark Call rclpy_destroy_entity() to destroy a guard condition
 * \sa rclpy_destroy_entity()
 * \return a list with the capsule and memory location, or
 * \return NULL on failure
 */
static PyObject *
rclpy_create_guard_condition(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcl_guard_condition_t * gc =
    (rcl_guard_condition_t *)PyMem_Malloc(sizeof(rcl_guard_condition_t));
  *gc = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t gc_options = rcl_guard_condition_get_default_options();

  rcl_ret_t ret = rcl_guard_condition_init(gc, gc_options);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create guard_condition: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    PyMem_Free(gc);
    return NULL;
  }
  PyObject * pylist = PyList_New(2);

  PyList_SET_ITEM(pylist, 0, PyCapsule_New(gc, "rcl_guard_condition_t", NULL));
  PyList_SET_ITEM(pylist, 1, PyLong_FromUnsignedLongLong((uint64_t)&gc->impl));

  return pylist;
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to trigger guard_condition: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Deallocate a list of allocated strings.
void
_rclpy_arg_list_fini(int num_args, char ** argv)
{
  if (NULL == argv) {
    return;
  }
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // Free each arg individually
  for (int i = 0; i < num_args; ++i) {
    char * arg = argv[i];
    if (NULL == arg) {
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
  if (NULL == pyargs) {
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
    if (NULL == *arg_values) {
      PyErr_Format(PyExc_MemoryError, "Failed to allocate space for arguments");
      Py_DECREF(pyargs);
      return RCL_RET_BAD_ALLOC;
    }

    for (int i = 0; i < *num_args; ++i) {
      // Returns borrowed reference, do not decref
      PyObject * pyarg = PyList_GetItem(pyargs, i);
      if (NULL == pyarg) {
        _rclpy_arg_list_fini(i, *arg_values);
        Py_DECREF(pyargs);
        // Exception raised
        return RCL_RET_ERROR;
      }
      const char * arg_str = PyUnicode_AsUTF8(pyarg);
      (*arg_values)[i] = rcutils_strdup(arg_str, allocator);
      if (NULL == (*arg_values)[i]) {
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

/// Parse a sequence of strings into rcl_arguments_t struct
/* Raises TypeError of pyargs is not a sequence
 * Raises OverflowError if len(pyargs) > INT_MAX
 * \param[in] pyargs a python sequence of strings
 * \param[out] parsed_args a zero initialized pointer to rcl_arguments_t
 */
rcl_ret_t
_rclpy_parse_args(PyObject * pyargs, rcl_arguments_t * parsed_args)
{
  rcl_ret_t ret;

  rcl_allocator_t allocator = rcl_get_default_allocator();
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

  // Adding const via cast to eliminate warning about incompatible pointer type
  const char ** const_arg_values = (const char **)arg_values;
  ret = rcl_parse_arguments(num_args, const_arg_values, allocator, parsed_args);

  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError, "Failed to init: %s", rcl_get_error_string_safe());
    rcl_reset_error();
  }
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
  PyObject * result_list = NULL;

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
    PyErr_Format(PyExc_RuntimeError, "Failed to init: %s", rcl_get_error_string_safe());
    rcl_reset_error();
  } else {
    int nonros_argc = 0;
    const char ** nonros_argv = NULL;

    ret = rcl_remove_ros_arguments(
      const_arg_values,
      &parsed_args,
      allocator,
      &nonros_argc,
      &nonros_argv);

    if (RCL_RET_OK != ret) {
      PyErr_Format(PyExc_RuntimeError, "Failed to init: %s", rcl_get_error_string_safe());
      rcl_reset_error();
    } else {
      result_list = PyList_New(nonros_argc);
      for (int ii = 0; ii < nonros_argc; ++ii) {
        PyList_SET_ITEM(result_list, ii, PyUnicode_FromString(nonros_argv[ii]));
      }
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
    }

    ret = rcl_arguments_fini(&parsed_args);
    if (RCL_RET_OK != ret) {
      PyErr_Format(PyExc_RuntimeError, "Failed to init: %s", rcl_get_error_string_safe());
      rcl_reset_error();
    }
  }

  _rclpy_arg_list_fini(num_args, arg_values);

  if (PyErr_Occurred()) {
    return NULL;
  }
  return result_list;
}


/// Initialize rcl with default options, ignoring parameters
/**
 * Raises RuntimeError if rcl could not be initialized
 */
static PyObject *
rclpy_init(PyObject * Py_UNUSED(self), PyObject * args)
{
  // Expect one argument which is a list of strings
  PyObject * pyargs;
  if (!PyArg_ParseTuple(args, "O", &pyargs)) {
    // Exception raised
    return NULL;
  }

  pyargs = PySequence_List(pyargs);
  if (NULL == pyargs) {
    // Exception raised
    return NULL;
  }
  Py_ssize_t pysize_num_args = PyList_Size(pyargs);
  if (pysize_num_args > INT_MAX) {
    PyErr_Format(PyExc_OverflowError, "Too many arguments");
    return NULL;
  }
  int num_args = (int)pysize_num_args;

  rcl_allocator_t allocator = rcl_get_default_allocator();
  const char ** arg_values = NULL;
  bool have_args = true;
  if (num_args > 0) {
    arg_values = allocator.allocate(sizeof(char *) * num_args, allocator.state);
    if (NULL == arg_values) {
      PyErr_Format(PyExc_MemoryError, "Failed to allocate space for arguments");
      Py_DECREF(pyargs);
      return NULL;
    }

    for (int i = 0; i < num_args; ++i) {
      // Returns borrowed reference, do not decref
      PyObject * pyarg = PyList_GetItem(pyargs, i);
      if (NULL == pyarg) {
        have_args = false;
        break;
      }
      // Borrows a pointer, do not free arg_values[i]
      arg_values[i] = PyUnicode_AsUTF8(pyarg);
    }
  }

  if (have_args) {
    rcl_ret_t ret = rcl_init(num_args, arg_values, allocator);
    if (ret != RCL_RET_OK) {
      PyErr_Format(PyExc_RuntimeError, "Failed to init: %s", rcl_get_error_string_safe());
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
  Py_DECREF(pyargs);

  // Register our signal handler that will forward to the original one.
  g_original_signal_handler = signal(SIGINT, catch_function);

  if (PyErr_Occurred()) {
    return NULL;
  }
  Py_RETURN_NONE;
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
  PyObject * py_cli_args;
  int use_global_arguments;

  if (
    !PyArg_ParseTuple(args, "ssOp", &node_name, &namespace_, &py_cli_args, &use_global_arguments))
  {
    return NULL;
  }

  rcl_arguments_t arguments = rcl_get_zero_initialized_arguments();

  ret = _rclpy_parse_args(py_cli_args, &arguments);
  if (RCL_RET_OK != ret) {
    // exception set
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyMem_Malloc(sizeof(rcl_node_t));
  *node = rcl_get_zero_initialized_node();
  rcl_node_options_t options = rcl_node_get_default_options();
  options.use_global_arguments = use_global_arguments;
  options.arguments = arguments;
  ret = rcl_node_init(node, node_name, namespace_, &options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError,
        "%s", rcl_get_error_string_safe());
    } else if (ret == RCL_RET_NODE_INVALID_NAME) {
      PyErr_Format(PyExc_ValueError,
        "invalid node name: %s", rcl_get_error_string_safe());
    } else if (ret == RCL_RET_NODE_INVALID_NAMESPACE) {
      PyErr_Format(PyExc_ValueError,
        "invalid node namespace: %s", rcl_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError,
        "Unknown error creating node: %s", rcl_get_error_string_safe());
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
        rcl_get_error_string_safe());
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
      rcl_get_error_string_safe());
  }
  return PyCapsule_New(node, "rcl_node_t", NULL);
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
    PyErr_Format(PyExc_RuntimeError, "Failed to count %s: %s",
      type, rcl_get_error_string_safe());
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
  PyObject * topic_name_py;

  if (!PyArg_ParseTuple(args, "O", &topic_name_py)) {
    return NULL;
  }

  if (!PyUnicode_Check(topic_name_py)) {
    PyErr_Format(PyExc_TypeError, "Argument topic_name is not a PyUnicode object");
    return NULL;
  }
  char * topic_name = (char *)PyUnicode_1BYTE_DATA(topic_name_py);

  int validation_result;
  size_t invalid_index;
  rcl_ret_t ret = rcl_validate_topic_name(topic_name, &validation_result, &invalid_index);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rcl_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError, "%s", rcl_get_error_string_safe());
    }
    rcl_reset_error();
    return NULL;
  }

  if (validation_result == RCL_TOPIC_NAME_VALID) {
    Py_RETURN_NONE;
  }

  const char * validation_message = rcl_topic_name_validation_result_string(validation_result);

  PyObject * result_list = PyList_New(2);
  PyList_SET_ITEM(result_list, 0, PyUnicode_FromString(validation_message));
  PyList_SET_ITEM(result_list, 1, PyLong_FromSize_t(invalid_index));

  return result_list;
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
  PyObject * topic_name_py;

  if (!PyArg_ParseTuple(args, "O", &topic_name_py)) {
    return NULL;
  }

  if (!PyUnicode_Check(topic_name_py)) {
    PyErr_Format(PyExc_TypeError, "Argument topic_name is not a PyUnicode object");
    return NULL;
  }
  char * topic_name = (char *)PyUnicode_1BYTE_DATA(topic_name_py);

  int validation_result;
  size_t invalid_index;
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, &invalid_index);
  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rmw_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError, "%s", rmw_get_error_string_safe());
    }
    rmw_reset_error();
    return NULL;
  }

  if (validation_result == RMW_NAMESPACE_VALID) {
    Py_RETURN_NONE;
  }

  const char * validation_message = rmw_full_topic_name_validation_result_string(validation_result);

  PyObject * result_list = PyList_New(2);
  PyList_SET_ITEM(result_list, 0, PyUnicode_FromString(validation_message));
  PyList_SET_ITEM(result_list, 1, PyLong_FromSize_t(invalid_index));

  return result_list;
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
  PyObject * namespace_py;

  if (!PyArg_ParseTuple(args, "O", &namespace_py)) {
    return NULL;
  }

  if (!PyUnicode_Check(namespace_py)) {
    PyErr_Format(PyExc_TypeError, "Argument namespace_ is not a PyUnicode object");
    return NULL;
  }
  char * namespace_ = (char *)PyUnicode_1BYTE_DATA(namespace_py);

  int validation_result;
  size_t invalid_index;
  rmw_ret_t ret = rmw_validate_namespace(namespace_, &validation_result, &invalid_index);
  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rmw_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError, "%s", rmw_get_error_string_safe());
    }
    rmw_reset_error();
    return NULL;
  }

  if (validation_result == RMW_NAMESPACE_VALID) {
    Py_RETURN_NONE;
  }

  const char * validation_message = rmw_namespace_validation_result_string(validation_result);

  PyObject * result_list = PyList_New(2);
  PyList_SET_ITEM(result_list, 0, PyUnicode_FromString(validation_message));
  PyList_SET_ITEM(result_list, 1, PyLong_FromSize_t(invalid_index));

  return result_list;
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
  PyObject * node_name_py;

  if (!PyArg_ParseTuple(args, "O", &node_name_py)) {
    return NULL;
  }

  if (!PyUnicode_Check(node_name_py)) {
    PyErr_Format(PyExc_TypeError, "Argument node_name is not a PyUnicode object");
    return NULL;
  }
  char * node_name = (char *)PyUnicode_1BYTE_DATA(node_name_py);

  int validation_result;
  size_t invalid_index;
  rmw_ret_t ret = rmw_validate_node_name(node_name, &validation_result, &invalid_index);
  if (ret != RMW_RET_OK) {
    if (ret == RMW_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rmw_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError, "%s", rmw_get_error_string_safe());
    }
    rmw_reset_error();
    return NULL;
  }

  if (validation_result == RMW_NODE_NAME_VALID) {
    Py_RETURN_NONE;
  }

  const char * validation_message = rmw_node_name_validation_result_string(validation_result);

  PyObject * result_list = PyList_New(2);
  PyList_SET_ITEM(result_list, 0, PyUnicode_FromString(validation_message));
  PyList_SET_ITEM(result_list, 1, PyLong_FromSize_t(invalid_index));

  return result_list;
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
      PyErr_Format(PyExc_MemoryError, "%s", rcutils_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError, "%s", rcutils_get_error_string_safe());
    }
    rcutils_reset_error();
    return NULL;
  }
  rcl_ret_t ret = rcl_get_default_topic_name_substitutions(&substitutions_map);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rcl_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError, "%s", rcl_get_error_string_safe());
    }
    rcl_reset_error();
    // finalize the string map before returning
    rcutils_ret = rcutils_string_map_fini(&substitutions_map);
    if (rcutils_ret != RCUTILS_RET_OK) {
      fprintf(stderr,
        "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
        "failed to fini string_map (%d) during error handling: %s\n",
        rcutils_ret,
        rcutils_get_error_string_safe());
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
    PyErr_Format(PyExc_RuntimeError, "%s", rcutils_get_error_string_safe());
    rcutils_reset_error();
    allocator.deallocate(expanded_topic, allocator.state);
    return NULL;
  }
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_BAD_ALLOC) {
      PyErr_Format(PyExc_MemoryError, "%s", rcl_get_error_string_safe());
    } else if (  // NOLINT
      ret == RCL_RET_TOPIC_NAME_INVALID ||
      ret == RCL_RET_UNKNOWN_SUBSTITUTION)
    {
      PyErr_Format(PyExc_ValueError,
        "topic name '%s' is invalid: %s", topic, rcl_get_error_string_safe());
    } else if (ret == RCL_RET_NODE_INVALID_NAME) {
      PyErr_Format(PyExc_ValueError,
        "node name '%s' is invalid: %s", node, rcl_get_error_string_safe());
    } else if (ret == RCL_RET_NODE_INVALID_NAMESPACE) {
      PyErr_Format(PyExc_ValueError,
        "node namespace '%s' is invalid: %s", namespace, rcl_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError, "%s", rcl_get_error_string_safe());
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
  PyObject * topic_name;
  PyObject * node_name_py;
  PyObject * node_namespace_py;

  if (!PyArg_ParseTuple(args, "OOO", &topic_name, &node_name_py, &node_namespace_py)) {
    return NULL;
  }

  if (!PyUnicode_Check(topic_name)) {
    PyErr_Format(PyExc_TypeError, "Argument topic_name is not a PyUnicode object");
    return NULL;
  }
  char * topic = (char *)PyUnicode_1BYTE_DATA(topic_name);

  if (!PyUnicode_Check(node_name_py)) {
    PyErr_Format(PyExc_TypeError, "Argument node_name is not a PyUnicode object");
    return NULL;
  }
  char * node_name = (char *)PyUnicode_1BYTE_DATA(node_name_py);

  if (!PyUnicode_Check(node_namespace_py)) {
    PyErr_Format(PyExc_TypeError, "Argument node_namespace is not a PyUnicode object");
    return NULL;
  }
  char * node_namespace = (char *)PyUnicode_1BYTE_DATA(node_namespace_py);

  char * expanded_topic = _expand_topic_name_with_exceptions(topic, node_name, node_namespace);

  if (!expanded_topic) {
    // exception already set
    return NULL;
  }

  PyObject * result = PyUnicode_FromString(expanded_topic);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  allocator.deallocate(expanded_topic, allocator.state);

  return result;
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

  if (!PyUnicode_Check(pytopic)) {
    PyErr_Format(PyExc_TypeError, "Argument pytopic is not a PyUnicode object");
    return NULL;
  }

  char * topic = (char *)PyUnicode_1BYTE_DATA(pytopic);

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__");

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");

  rosidl_message_type_support_t * ts =
    (rosidl_message_type_support_t *)PyCapsule_GetPointer(pyts, NULL);

  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  if (PyCapsule_IsValid(pyqos_profile, "rmw_qos_profile_t")) {
    void * p = PyCapsule_GetPointer(pyqos_profile, "rmw_qos_profile_t");
    rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)p;
    publisher_ops.qos = *qos_profile;
    PyMem_Free(p);
    if (PyCapsule_SetPointer(pyqos_profile, Py_None)) {
      // exception set by PyCapsule_SetPointer
      return NULL;
    }
  }

  rcl_publisher_t * publisher = (rcl_publisher_t *)PyMem_Malloc(sizeof(rcl_publisher_t));
  *publisher = rcl_get_zero_initialized_publisher();

  rcl_ret_t ret = rcl_publisher_init(publisher, node, ts, topic, &publisher_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      PyErr_Format(PyExc_ValueError,
        "Failed to create publisher due to invalid topic name '%s': %s",
        topic, rcl_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError,
        "Failed to create publisher: %s", rcl_get_error_string_safe());
    }
    rcl_reset_error();
    PyMem_Free(publisher);
    return NULL;
  }
  return PyCapsule_New(publisher, "rcl_publisher_t", NULL);
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

  rcl_publisher_t * publisher = (rcl_publisher_t *)PyCapsule_GetPointer(
    pypublisher, "rcl_publisher_t");
  if (!publisher) {
    return NULL;
  }

  PyObject * pymsg_type = PyObject_GetAttrString(pymsg, "__class__");

  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__");
  Py_DECREF(pymsg_type);

  create_ros_message_signature * create_ros_message = get_capsule_pointer(
    pymetaclass, "_CREATE_ROS_MESSAGE");
  assert(create_ros_message != NULL &&
    "unable to retrieve create_ros_message function, type_support mustn't have been imported");

  destroy_ros_message_signature * destroy_ros_message = get_capsule_pointer(
    pymetaclass, "_DESTROY_ROS_MESSAGE");
  assert(destroy_ros_message != NULL &&
    "unable to retrieve destroy_ros_message function, type_support mustn't have been imported");

  convert_from_py_signature * convert_from_py = get_capsule_pointer(
    pymetaclass, "_CONVERT_FROM_PY");
  assert(convert_from_py != NULL &&
    "unable to retrieve convert_from_py function, type_support mustn't have been imported");

  Py_DECREF(pymetaclass);

  void * raw_ros_message = create_ros_message();
  if (!raw_ros_message) {
    return PyErr_NoMemory();
  }

  if (!convert_from_py(pymsg, raw_ros_message)) {
    // the function has set the Python error
    destroy_ros_message(raw_ros_message);
    return NULL;
  }

  rcl_ret_t ret = rcl_publish(publisher, raw_ros_message);
  destroy_ros_message(raw_ros_message);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to publish: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Create a timer
/**
 * When successful a list with two elements is returned:
 *
 * - a Capsule pointing to the pointer of the created rcl_timer_t * structure
 * - an integer representing the memory address of the created rcl_timer_t
 *
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises RuntimeError on initialization failure
 * Raises TypeError if argument of invalid type
 * Raises ValueError if argument cannot be converted to uint64_t
 *
 * \param[in] period_nsec unsigned PyLong object storing the period of the
 *   timer in nanoseconds in a 64-bit unsigned integer
 * \return a list of the capsule and the memory address
 * \return NULL on failure
 */
static PyObject *
rclpy_create_timer(PyObject * Py_UNUSED(self), PyObject * args)
{
  unsigned PY_LONG_LONG period_nsec;

  if (!PyArg_ParseTuple(args, "K", &period_nsec)) {
    return NULL;
  }

  rcl_timer_t * timer = (rcl_timer_t *) PyMem_Malloc(sizeof(rcl_timer_t));
  *timer = rcl_get_zero_initialized_timer();

  rcl_ret_t ret = rcl_timer_init(timer, period_nsec, NULL, rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to create subscriptions: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    PyMem_Free(timer);
    return NULL;
  }
  PyObject * pylist = PyList_New(2);
  PyList_SET_ITEM(pylist, 0, PyCapsule_New(timer, "rcl_timer_t", NULL));
  PyList_SET_ITEM(pylist, 1, PyLong_FromUnsignedLongLong((uint64_t)&timer->impl));

  return pylist;
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get timer period: %s", rcl_get_error_string_safe());
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to reset timer: %s", rcl_get_error_string_safe());
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to check timer ready: %s", rcl_get_error_string_safe());
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to reset timer: %s", rcl_get_error_string_safe());
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to check timer ready: %s", rcl_get_error_string_safe());
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to call timer: %s", rcl_get_error_string_safe());
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
 * Raises RuntimeError if the timer perioud could not be changed
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to exchange timer period: %s", rcl_get_error_string_safe());
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get time until next timer call: %s", rcl_get_error_string_safe());
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
  int64_t elapsed_time;
  rcl_ret_t ret = rcl_timer_get_time_since_last_call(timer, &elapsed_time);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get time since last timer call: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  return PyLong_FromUnsignedLongLong(elapsed_time);
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

  if (!PyUnicode_Check(pytopic)) {
    PyErr_Format(PyExc_TypeError, "Argument pytopic is not a PyUnicode object");
    return NULL;
  }

  char * topic = (char *)PyUnicode_1BYTE_DATA(pytopic);

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__");

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");

  rosidl_message_type_support_t * ts =
    (rosidl_message_type_support_t *)PyCapsule_GetPointer(pyts, NULL);

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();

  if (PyCapsule_IsValid(pyqos_profile, "rmw_qos_profile_t")) {
    void * p = PyCapsule_GetPointer(pyqos_profile, "rmw_qos_profile_t");
    rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)p;
    subscription_ops.qos = *qos_profile;
    PyMem_Free(p);
    if (PyCapsule_SetPointer(pyqos_profile, Py_None)) {
      // exception set by PyCapsule_SetPointer
      return NULL;
    }
  }

  rcl_subscription_t * subscription =
    (rcl_subscription_t *)PyMem_Malloc(sizeof(rcl_subscription_t));
  *subscription = rcl_get_zero_initialized_subscription();

  rcl_ret_t ret = rcl_subscription_init(subscription, node, ts, topic, &subscription_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      PyErr_Format(PyExc_ValueError,
        "Failed to create subscription due to invalid topic name '%s': %s",
        topic, rcl_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError,
        "Failed to create subscription: %s", rcl_get_error_string_safe());
    }
    rcl_reset_error();
    PyMem_Free(subscription);
    return NULL;
  }
  PyObject * pylist = PyList_New(2);
  PyList_SET_ITEM(pylist, 0, PyCapsule_New(subscription, "rcl_subscription_t", NULL));
  PyList_SET_ITEM(pylist, 1, PyLong_FromUnsignedLongLong((uint64_t)&subscription->impl));

  return pylist;
}

/// Create a client
/**
 * This function will create a client for the given service name.
 * This client will use the typesupport defined in the service module
 * provided as pysrv_type to send messages over the wire.
 *
 * On a successful call a list with two elements is returned:
 *
 * - a Capsule pointing to the pointer of the created rcl_client_t * structure
 * - an integer representing the memory address of the created rcl_client_t
 *
 * Raises ValueError if the capsules are not the correct types
 * Raises RuntimeError if the client could not be created
 *
 * \param[in] pynode Capsule pointing to the node to add the client to
 * \param[in] pysrv_type Service module associated with the client
 * \param[in] pyservice_name Python object containing the service name
 * \param[in] pyqos_profile QoSProfile Python object for this client
 * \return capsule and memory address, or
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

  if (!PyUnicode_Check(pyservice_name)) {
    PyErr_Format(PyExc_TypeError, "Argument pyservice_name is not a PyUnicode object");
    return NULL;
  }

  char * service_name = (char *)PyUnicode_1BYTE_DATA(pyservice_name);

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  PyObject * pymetaclass = PyObject_GetAttrString(pysrv_type, "__class__");

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");

  rosidl_service_type_support_t * ts =
    (rosidl_service_type_support_t *)PyCapsule_GetPointer(pyts, NULL);

  rcl_client_options_t client_ops = rcl_client_get_default_options();

  if (PyCapsule_IsValid(pyqos_profile, "rmw_qos_profile_t")) {
    void * p = PyCapsule_GetPointer(pyqos_profile, "rmw_qos_profile_t");
    rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)p;
    client_ops.qos = *qos_profile;
    PyMem_Free(p);
    if (PyCapsule_SetPointer(pyqos_profile, Py_None)) {
      // exception set by PyCapsule_SetPointer
      return NULL;
    }
  }

  rcl_client_t * client = (rcl_client_t *)PyMem_Malloc(sizeof(rcl_client_t));
  *client = rcl_get_zero_initialized_client();

  rcl_ret_t ret = rcl_client_init(client, node, ts, service_name, &client_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      PyErr_Format(PyExc_ValueError,
        "Failed to create client due to invalid service name '%s': %s",
        service_name, rcl_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError,
        "Failed to create client: %s", rcl_get_error_string_safe());
    }
    rcl_reset_error();
    PyMem_Free(client);
    return NULL;
  }
  PyObject * pylist = PyList_New(2);
  PyList_SET_ITEM(pylist, 0, PyCapsule_New(client, "rcl_client_t", NULL));
  PyList_SET_ITEM(pylist, 1, PyLong_FromUnsignedLongLong((uint64_t)&client->impl));

  return pylist;
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
  rcl_client_t * client = (rcl_client_t *)PyCapsule_GetPointer(pyclient, "rcl_client_t");
  if (!client) {
    return NULL;
  }

  PyObject * pyrequest_type = PyObject_GetAttrString(pyrequest, "__class__");
  assert(pyrequest_type != NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pyrequest_type, "__class__");
  assert(pymetaclass != NULL);

  create_ros_message_signature * create_ros_message = get_capsule_pointer(
    pymetaclass, "_CREATE_ROS_MESSAGE");
  assert(create_ros_message != NULL &&
    "unable to retrieve create_ros_message function, type_support mustn't have been imported");

  destroy_ros_message_signature * destroy_ros_message = get_capsule_pointer(
    pymetaclass, "_DESTROY_ROS_MESSAGE");
  assert(destroy_ros_message != NULL &&
    "unable to retrieve destroy_ros_message function, type_support mustn't have been imported");

  convert_from_py_signature * convert_from_py = get_capsule_pointer(
    pymetaclass, "_CONVERT_FROM_PY");
  assert(convert_from_py != NULL &&
    "unable to retrieve convert_from_py function, type_support mustn't have been imported");

  Py_DECREF(pymetaclass);

  void * raw_ros_request = create_ros_message();
  if (!raw_ros_request) {
    return PyErr_NoMemory();
  }

  if (!convert_from_py(pyrequest, raw_ros_request)) {
    // the function has set the Python error
    destroy_ros_message(raw_ros_request);
    return NULL;
  }

  int64_t sequence_number;
  rcl_ret_t ret = rcl_send_request(client, raw_ros_request, &sequence_number);
  destroy_ros_message(raw_ros_request);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to send request: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  return PyLong_FromLongLong(sequence_number);
}

/// Create a service server
/**
 * This function will create a service server for the given service name.
 * This service will use the typesupport defined in the service module
 * provided as pysrv_type to send messages over the wire.
 *
 *
 * On a successful call a list with two elements is returned:
 *
 * - a Capsule pointing to the pointer of the created rcl_service_t * structure
 * - an integer representing the memory address of the created rcl_service_t
 *
 * Raises ValueError if the capsules are not the correct types
 * Raises RuntimeError if the service could not be created
 *
 * \param[in] pynode Capsule pointing to the node to add the service to
 * \param[in] pysrv_type Service module associated with the service
 * \param[in] pyservice_name Python object for the service name
 * \param[in] pyqos_profile QoSProfile Python object for this service
 * \return capsule and memory address, or
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

  if (!PyUnicode_Check(pyservice_name)) {
    PyErr_Format(PyExc_TypeError, "Argument pyservice_name is not a PyUnicode object");
    return NULL;
  }

  char * service_name = (char *)PyUnicode_1BYTE_DATA(pyservice_name);

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");

  PyObject * pymetaclass = PyObject_GetAttrString(pysrv_type, "__class__");

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");

  rosidl_service_type_support_t * ts =
    (rosidl_service_type_support_t *)PyCapsule_GetPointer(pyts, NULL);

  rcl_service_options_t service_ops = rcl_service_get_default_options();

  if (PyCapsule_IsValid(pyqos_profile, "rmw_qos_profile_t")) {
    void * p = PyCapsule_GetPointer(pyqos_profile, "rmw_qos_profile_t");
    rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)p;
    service_ops.qos = *qos_profile;
    PyMem_Free(p);
    if (PyCapsule_SetPointer(pyqos_profile, Py_None)) {
      // exception set by PyCapsule_SetPointer
      return NULL;
    }
  }

  rcl_service_t * service = (rcl_service_t *)PyMem_Malloc(sizeof(rcl_service_t));
  *service = rcl_get_zero_initialized_service();
  rcl_ret_t ret = rcl_service_init(service, node, ts, service_name, &service_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      PyErr_Format(PyExc_ValueError,
        "Failed to create service due to invalid topic name '%s': %s",
        service_name, rcl_get_error_string_safe());
    } else {
      PyErr_Format(PyExc_RuntimeError,
        "Failed to create service: %s", rcl_get_error_string_safe());
    }
    PyMem_Free(service);
    rcl_reset_error();
    return NULL;
  }
  PyObject * pylist = PyList_New(2);
  PyList_SET_ITEM(pylist, 0, PyCapsule_New(service, "rcl_service_t", NULL));
  PyList_SET_ITEM(pylist, 1, PyLong_FromUnsignedLongLong((uint64_t)&service->impl));

  return pylist;
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
  rcl_service_t * service = (rcl_service_t *)PyCapsule_GetPointer(
    pyservice, "rcl_service_t");
  if (!service) {
    return NULL;
  }

  rmw_request_id_t * header = (rmw_request_id_t *)PyCapsule_GetPointer(
    pyheader, "rmw_request_id_t");
  if (!header) {
    return NULL;
  }
  PyObject * pyresponse_type = PyObject_GetAttrString(pyresponse, "__class__");
  assert(pyresponse_type != NULL);

  PyObject * pymetaclass = PyObject_GetAttrString(pyresponse_type, "__class__");
  assert(pymetaclass != NULL);

  Py_DECREF(pyresponse_type);

  create_ros_message_signature * create_ros_message = get_capsule_pointer(
    pymetaclass, "_CREATE_ROS_MESSAGE");
  assert(create_ros_message != NULL &&
    "unable to retrieve create_ros_message function, type_support mustn't have been imported");

  destroy_ros_message_signature * destroy_ros_message = get_capsule_pointer(
    pymetaclass, "_DESTROY_ROS_MESSAGE");
  assert(destroy_ros_message != NULL &&
    "unable to retrieve destroy_ros_message function, type_support mustn't have been imported");

  convert_from_py_signature * convert_from_py = get_capsule_pointer(
    pymetaclass, "_CONVERT_FROM_PY");
  assert(convert_from_py != NULL &&
    "unable to retrieve convert_from_py function, type_support mustn't have been imported");

  Py_DECREF(pymetaclass);

  void * raw_ros_response = create_ros_message();
  if (!raw_ros_response) {
    return PyErr_NoMemory();
  }

  if (!convert_from_py(pyresponse, raw_ros_response)) {
    // the function has set the Python error
    destroy_ros_message(raw_ros_response);
    return NULL;
  }

  rcl_ret_t ret = rcl_send_response(service, header, raw_ros_response);
  destroy_ros_message(raw_ros_response);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to send request: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Check if a service server is available
/**
 * Raises ValueError if the arguments are not capsules
 *
 * \param[in] pynode Capsule pointing to the node the entity belongs to
 * \param[in] pyclient Capsule pointing to the client
 * \return True if the service server is available
 */
static PyObject *
rclpy_service_server_is_available(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pyclient;

  if (!PyArg_ParseTuple(args, "OO", &pynode, &pyclient)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }
  rcl_client_t * client = (rcl_client_t *)PyCapsule_GetPointer(pyclient, "rcl_client_t");
  if (!client) {
    return NULL;
  }

  bool is_ready;
  rcl_ret_t ret = rcl_service_server_is_available(node, client, &is_ready);

  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to check service availability: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  if (is_ready) {
    Py_RETURN_TRUE;
  }
  Py_RETURN_FALSE;
}

/// Destroy an entity attached to a node
/**
 * Entity type must be one of ["subscription", "publisher", "client", "service"].
 *
 * Raises RuntimeError on failure
 *
 * \param[in] pyentity Capsule pointing to the entity to destroy
 * \param[in] pynode Capsule pointing to the node the entity belongs to
 */
static PyObject *
rclpy_destroy_node_entity(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyentity;
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "OO", &pyentity, &pynode)) {
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");
  if (!node) {
    return NULL;
  }

  if (!PyCapsule_CheckExact(pyentity)) {
    PyErr_Format(PyExc_RuntimeError, "entity is not a capsule");
    return NULL;
  }

  rcl_ret_t ret;
  if (PyCapsule_IsValid(pyentity, "rcl_subscription_t")) {
    rcl_subscription_t * subscription = (rcl_subscription_t *)PyCapsule_GetPointer(
      pyentity, "rcl_subscription_t");
    ret = rcl_subscription_fini(subscription, node);
    PyMem_Free(subscription);
  } else if (PyCapsule_IsValid(pyentity, "rcl_publisher_t")) {
    rcl_publisher_t * publisher = (rcl_publisher_t *)PyCapsule_GetPointer(
      pyentity, "rcl_publisher_t");
    ret = rcl_publisher_fini(publisher, node);
    PyMem_Free(publisher);
  } else if (PyCapsule_IsValid(pyentity, "rcl_client_t")) {
    rcl_client_t * client = (rcl_client_t *)PyCapsule_GetPointer(pyentity, "rcl_client_t");
    ret = rcl_client_fini(client, node);
    PyMem_Free(client);
  } else if (PyCapsule_IsValid(pyentity, "rcl_service_t")) {
    rcl_service_t * service = (rcl_service_t *)PyCapsule_GetPointer(pyentity, "rcl_service_t");
    ret = rcl_service_fini(service, node);
    PyMem_Free(service);
  } else {
    ret = RCL_RET_ERROR;  // to avoid a linter warning
    PyErr_Format(PyExc_RuntimeError, "'%s' is not a known node entity",
      PyCapsule_GetName(pyentity));
    return NULL;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to fini '%s': %s", PyCapsule_GetName(pyentity), rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  if (PyCapsule_SetPointer(pyentity, Py_None)) {
    // exception set by PyCapsule_SetPointer
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Destroy an rcl entity
/**
 * Raises RuntimeError on failure
 *
 * \param[in] pyentity Capsule pointing to the entity to destroy
 */
static PyObject *
rclpy_destroy_entity(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyentity;

  if (!PyArg_ParseTuple(args, "O", &pyentity)) {
    return NULL;
  }

  if (!PyCapsule_CheckExact(pyentity)) {
    PyErr_Format(PyExc_ValueError, "Object is not a capsule");
    return NULL;
  }

  rcl_ret_t ret;
  if (PyCapsule_IsValid(pyentity, "rcl_node_t")) {
    rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pyentity, "rcl_node_t");
    ret = rcl_node_fini(node);
    PyMem_Free(node);
  } else if (PyCapsule_IsValid(pyentity, "rcl_timer_t")) {
    rcl_timer_t * timer = (rcl_timer_t *)PyCapsule_GetPointer(pyentity, "rcl_timer_t");
    ret = rcl_timer_fini(timer);
    PyMem_Free(timer);
  } else if (PyCapsule_IsValid(pyentity, "rcl_guard_condition_t")) {
    rcl_guard_condition_t * guard_condition = (rcl_guard_condition_t *)PyCapsule_GetPointer(
      pyentity, "rcl_guard_condition_t");
    ret = rcl_guard_condition_fini(guard_condition);
    PyMem_Free(guard_condition);
  } else {
    ret = RCL_RET_ERROR;  // to avoid a linter warning
    PyErr_Format(PyExc_RuntimeError, "'%s' is not a known entity", PyCapsule_GetName(pyentity));
    return NULL;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to fini '%s': %s", PyCapsule_GetName(pyentity), rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  if (PyCapsule_SetPointer(pyentity, Py_None)) {
    // exception set by PyCapsule_SetPointer
    return NULL;
  }

  Py_RETURN_NONE;
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

  if (!PyArg_ParseTuple(
      args, "OKKKKK", &pywait_set, &number_of_subscriptions,
      &number_of_guard_conditions, &number_of_timers,
      &number_of_clients, &number_of_services))
  {
    return NULL;
  }

  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, "rcl_wait_set_t");
  if (!wait_set) {
    return NULL;
  }
  rcl_ret_t ret = rcl_wait_set_init(
    wait_set, number_of_subscriptions, number_of_guard_conditions, number_of_timers,
    number_of_clients, number_of_services, rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to initialize wait set: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Clear all the pointers of a given wait set field
/**
 * Raises RuntimeError if the entity type is unknown or any rcl error occurs
 *
 * \param[in] entity_type string defining the entity ["subscription, client, service"]
 * \param[in] pywait_set Capsule pointing to the wait set structure
 * \return NULL
 */
static PyObject *
rclpy_wait_set_clear_entities(PyObject * Py_UNUSED(self), PyObject * args)
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
  rcl_ret_t ret;
  if (0 == strcmp(entity_type, "subscription")) {
    ret = rcl_wait_set_clear_subscriptions(wait_set);
  } else if (0 == strcmp(entity_type, "client")) {
    ret = rcl_wait_set_clear_clients(wait_set);
  } else if (0 == strcmp(entity_type, "service")) {
    ret = rcl_wait_set_clear_services(wait_set);
  } else if (0 == strcmp(entity_type, "timer")) {
    ret = rcl_wait_set_clear_timers(wait_set);
  } else if (0 == strcmp(entity_type, "guard_condition")) {
    ret = rcl_wait_set_clear_guard_conditions(wait_set);
  } else {
    ret = RCL_RET_ERROR;  // to avoid a linter warning
    PyErr_Format(PyExc_RuntimeError,
      "'%s' is not a known entity", entity_type);
    return NULL;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to clear '%s' from wait set: %s", entity_type, rcl_get_error_string_safe());
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
 * \return None
 */
static PyObject *
rclpy_wait_set_add_entity(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * entity_type;
  PyObject * pywait_set;
  PyObject * pyentity;

  if (!PyArg_ParseTuple(args, "zOO", &entity_type, &pywait_set, &pyentity)) {
    return NULL;
  }
  rcl_ret_t ret;
  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)PyCapsule_GetPointer(pywait_set, "rcl_wait_set_t");
  if (!wait_set) {
    return NULL;
  }
  if (0 == strcmp(entity_type, "subscription")) {
    rcl_subscription_t * subscription =
      (rcl_subscription_t *)PyCapsule_GetPointer(pyentity, "rcl_subscription_t");
    ret = rcl_wait_set_add_subscription(wait_set, subscription);
  } else if (0 == strcmp(entity_type, "client")) {
    rcl_client_t * client =
      (rcl_client_t *)PyCapsule_GetPointer(pyentity, "rcl_client_t");
    ret = rcl_wait_set_add_client(wait_set, client);
  } else if (0 == strcmp(entity_type, "service")) {
    rcl_service_t * service =
      (rcl_service_t *)PyCapsule_GetPointer(pyentity, "rcl_service_t");
    ret = rcl_wait_set_add_service(wait_set, service);
  } else if (0 == strcmp(entity_type, "timer")) {
    rcl_timer_t * timer =
      (rcl_timer_t *)PyCapsule_GetPointer(pyentity, "rcl_timer_t");
    ret = rcl_wait_set_add_timer(wait_set, timer);
  } else if (0 == strcmp(entity_type, "guard_condition")) {
    rcl_guard_condition_t * guard_condition =
      (rcl_guard_condition_t *)PyCapsule_GetPointer(pyentity, "rcl_guard_condition_t");
    ret = rcl_wait_set_add_guard_condition(wait_set, guard_condition);
  } else {
    ret = RCL_RET_ERROR;  // to avoid a linter warning
    PyErr_Format(PyExc_RuntimeError,
      "'%s' is not a known entity", entity_type);
    return NULL;
  }
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to add '%s' to wait set: %s", entity_type, rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to fini wait set: %s", rcl_get_error_string_safe());
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
      PyObject * obj = PyLong_FromUnsignedLongLong((uint64_t) & struct_ptr[idx]->impl); \
      if (obj) { \
        PyList_Append(entity_ready_list, obj); \
        Py_DECREF(obj); \
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
  PyErr_Format(PyExc_RuntimeError,
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to wait on wait set: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
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

  if (!PyArg_ParseTuple(args, "OO", &pysubscription, &pymsg_type)) {
    return NULL;
  }
  if (!PyCapsule_CheckExact(pysubscription)) {
    PyErr_Format(PyExc_TypeError, "Argument pysubscription is not a valid PyCapsule");
    return NULL;
  }
  rcl_subscription_t * subscription =
    (rcl_subscription_t *)PyCapsule_GetPointer(pysubscription, "rcl_subscription_t");

  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__");

  create_ros_message_signature * create_ros_message = get_capsule_pointer(
    pymetaclass, "_CREATE_ROS_MESSAGE");
  assert(create_ros_message != NULL &&
    "unable to retrieve create_ros_message function, type_support mustn't have been imported");

  destroy_ros_message_signature * destroy_ros_message = get_capsule_pointer(
    pymetaclass, "_DESTROY_ROS_MESSAGE");
  assert(destroy_ros_message != NULL &&
    "unable to retrieve destroy_ros_message function, type_support mustn't have been imported");

  void * taken_msg = create_ros_message();
  if (!taken_msg) {
    Py_DECREF(pymetaclass);
    return PyErr_NoMemory();
  }

  rcl_ret_t ret = rcl_take(subscription, taken_msg, NULL);

  if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to take from a subscription: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    destroy_ros_message(taken_msg);
    Py_DECREF(pymetaclass);
    return NULL;
  }

  if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    convert_to_py_signature * convert_to_py = get_capsule_pointer(pymetaclass, "_CONVERT_TO_PY");
    Py_DECREF(pymetaclass);

    PyObject * pytaken_msg = convert_to_py(taken_msg);
    destroy_ros_message(taken_msg);
    if (!pytaken_msg) {
      // the function has set the Python error
      return NULL;
    }

    return pytaken_msg;
  }

  // if take failed, just do nothing
  destroy_ros_message(taken_msg);
  Py_DECREF(pymetaclass);
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

  rcl_service_t * service =
    (rcl_service_t *)PyCapsule_GetPointer(pyservice, "rcl_service_t");
  if (!service) {
    return NULL;
  }

  PyObject * pymetaclass = PyObject_GetAttrString(pyrequest_type, "__class__");

  create_ros_message_signature * create_ros_message = get_capsule_pointer(
    pymetaclass, "_CREATE_ROS_MESSAGE");
  assert(create_ros_message != NULL &&
    "unable to retrieve create_ros_message function, type_support mustn't have been imported");

  destroy_ros_message_signature * destroy_ros_message = get_capsule_pointer(
    pymetaclass, "_DESTROY_ROS_MESSAGE");
  assert(destroy_ros_message != NULL &&
    "unable to retrieve destroy_ros_message function, type_support mustn't have been imported");

  void * taken_request = create_ros_message();

  if (!taken_request) {
    Py_DECREF(pymetaclass);
    return PyErr_NoMemory();
  }

  rmw_request_id_t * header = (rmw_request_id_t *)PyMem_Malloc(sizeof(rmw_request_id_t));
  rcl_ret_t ret = rcl_take_request(service, header, taken_request);

  if (ret != RCL_RET_OK && ret != RCL_RET_SERVICE_TAKE_FAILED) {
    PyErr_Format(PyExc_RuntimeError,
      "Service failed to take request: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    destroy_ros_message(taken_request);
    PyMem_Free(header);
    Py_DECREF(pymetaclass);
    return NULL;
  }

  if (ret != RCL_RET_SERVICE_TAKE_FAILED) {
    convert_to_py_signature * convert_to_py = get_capsule_pointer(pymetaclass, "_CONVERT_TO_PY");
    Py_DECREF(pymetaclass);

    PyObject * pytaken_request = convert_to_py(taken_request);
    destroy_ros_message(taken_request);
    if (!pytaken_request) {
      // the function has set the Python error
      PyMem_Free(header);
      return NULL;
    }

    PyObject * pylist = PyList_New(2);
    PyList_SET_ITEM(pylist, 0, pytaken_request);
    PyList_SET_ITEM(pylist, 1, PyCapsule_New(header, "rmw_request_id_t", NULL));

    return pylist;
  }
  // if take_request failed, just do nothing
  PyMem_Free(header);
  destroy_ros_message(taken_request);
  Py_DECREF(pymetaclass);
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
  rcl_client_t * client =
    (rcl_client_t *)PyCapsule_GetPointer(pyclient, "rcl_client_t");
  if (!client) {
    return NULL;
  }

  PyObject * pymetaclass = PyObject_GetAttrString(pyresponse_type, "__class__");

  create_ros_message_signature * create_ros_message = get_capsule_pointer(
    pymetaclass, "_CREATE_ROS_MESSAGE");
  assert(create_ros_message != NULL &&
    "unable to retrieve create_ros_message function, type_support mustn't have been imported");

  destroy_ros_message_signature * destroy_ros_message = get_capsule_pointer(
    pymetaclass, "_DESTROY_ROS_MESSAGE");
  assert(destroy_ros_message != NULL &&
    "unable to retrieve destroy_ros_message function, type_support mustn't have been imported");

  void * taken_response = create_ros_message();
  if (!taken_response) {
    // the function has set the Python error
    Py_DECREF(pymetaclass);
    return NULL;
  }
  rmw_request_id_t * header = (rmw_request_id_t *)PyMem_Malloc(sizeof(rmw_request_id_t));
  rcl_ret_t ret = rcl_take_response(client, header, taken_response);
  int64_t sequence = header->sequence_number;
  PyMem_Free(header);

  // Create the tuple to return
  PyObject * pytuple = PyTuple_New(2);
  if (!pytuple) {
    return NULL;
  }

  if (ret != RCL_RET_CLIENT_TAKE_FAILED) {
    convert_to_py_signature * convert_to_py = get_capsule_pointer(pymetaclass, "_CONVERT_TO_PY");
    Py_DECREF(pymetaclass);

    PyObject * pytaken_response = convert_to_py(taken_response);
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
  Py_DECREF(pymetaclass);
  destroy_ros_message(taken_response);
  return pytuple;
}

/// Status of the the client library
/**
 * \return True if rcl is running properly, False otherwise
 */
static PyObject *
rclpy_ok(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  bool ok = rcl_ok();
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
rclpy_shutdown(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcl_ret_t ret = rcl_shutdown();
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to shutdown: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  // Restore the original signal handler.
  signal(SIGINT, g_original_signal_handler);
  g_original_signal_handler = NULL;

  Py_RETURN_NONE;
}

/// Get the list of nodes discovered by the provided node
/**
 *  Raises ValueError if pynode is not a node capsule
 *  Raises RuntimeError  if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node
 * \return Python list of strings
 */
static PyObject *
rclpy_get_node_names(PyObject * Py_UNUSED(self), PyObject * args)
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
  rcl_ret_t ret = rcl_get_node_names(node, allocator, &node_names);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get_node_names: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  PyObject * pynode_names = PyList_New(node_names.size);
  size_t idx;
  for (idx = 0; idx < node_names.size; ++idx) {
    if (node_names.data[idx]) {
      PyList_SetItem(
        pynode_names, idx, PyUnicode_FromString(node_names.data[idx]));
    } else {
      Py_INCREF(Py_None);
      PyList_SetItem(pynode_names, idx, Py_None);
    }
  }

  ret = rcutils_string_array_fini(&node_names);
  if (ret != RCUTILS_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to destroy node_names: %s", rcl_get_error_string_safe());
    Py_DECREF(pynode_names);
    rcl_reset_error();
    return NULL;
  }

  return pynode_names;
}

/// Get the list of topics discovered by the provided node
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RuntimeError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node
 * \param[in] no_demangle if true topic names and types returned will not be demangled
 * \return Python list of tuples where each tuple contains the two strings:
 *   the topic name and topic type
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get_topic_names_and_types: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  PyObject * pytopic_names_and_types = PyList_New(topic_names_and_types.names.size);
  size_t i;
  for (i = 0; i < topic_names_and_types.names.size; ++i) {
    PyObject * pytuple = PyTuple_New(2);
    PyTuple_SetItem(
      pytuple, 0,
      PyUnicode_FromString(topic_names_and_types.names.data[i]));
    PyObject * types_list = PyList_New(topic_names_and_types.types[i].size);
    size_t j;
    for (j = 0; j < topic_names_and_types.types[i].size; ++j) {
      PyList_SetItem(
        types_list, j,
        PyUnicode_FromString(topic_names_and_types.types[i].data[j]));
    }
    PyTuple_SetItem(
      pytuple, 1,
      types_list);
    PyList_SetItem(
      pytopic_names_and_types, i,
      pytuple);
  }

  ret = rcl_names_and_types_fini(&topic_names_and_types);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to destroy topic_names_and_types: %s", rcl_get_error_string_safe());
    Py_DECREF(pytopic_names_and_types);
    rcl_reset_error();
    return NULL;
  }

  return pytopic_names_and_types;
}

/// Get the list of services discovered by the provided node
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RuntimeError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node
 * \return Python list of tuples where each tuple contains the two strings:
 *   the topic name and topic type
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
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get_service_names_and_types: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  PyObject * pyservice_names_and_types = PyList_New(service_names_and_types.names.size);
  size_t i;
  for (i = 0; i < service_names_and_types.names.size; ++i) {
    PyObject * pytuple = PyTuple_New(2);
    PyTuple_SetItem(
      pytuple, 0,
      PyUnicode_FromString(service_names_and_types.names.data[i]));
    PyObject * types_list = PyList_New(service_names_and_types.types[i].size);
    size_t j;
    for (j = 0; j < service_names_and_types.types[i].size; ++j) {
      PyList_SetItem(
        types_list, j,
        PyUnicode_FromString(service_names_and_types.types[i].data[j]));
    }
    PyTuple_SetItem(
      pytuple, 1,
      types_list);
    PyList_SetItem(
      pyservice_names_and_types, i,
      pytuple);
  }

  ret = rcl_names_and_types_fini(&service_names_and_types);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to destroy service_names_and_types: %s", rcl_get_error_string_safe());
    Py_DECREF(pyservice_names_and_types);
    rcl_reset_error();
    return NULL;
  }

  return pyservice_names_and_types;
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
  int avoid_ros_namespace_conventions;

  if (!PyArg_ParseTuple(
      args, "KKKKp",
      &pyqos_history,
      &pyqos_depth,
      &pyqos_reliability,
      &pyqos_durability,
      &avoid_ros_namespace_conventions))
  {
    return NULL;
  }

  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)PyMem_Malloc(sizeof(rmw_qos_profile_t));
  qos_profile->history = pyqos_history;
  qos_profile->depth = pyqos_depth;
  qos_profile->reliability = pyqos_reliability;
  qos_profile->durability = pyqos_durability;
  qos_profile->avoid_ros_namespace_conventions = avoid_ros_namespace_conventions;
  PyObject * pyqos_profile = PyCapsule_New(qos_profile, "rmw_qos_profile_t", NULL);
  return pyqos_profile;
}

/// Convert a C rmw_qos_profile_t into a Python QoSProfile object
/**
 * \param[in] void pointer to a rmw_qos_profile_t structure
 * \return QoSProfile object
 */
static PyObject *
rclpy_convert_to_py_qos_policy(void * profile)
{
  PyObject * pyqos_module = PyImport_ImportModule("rclpy.qos");
  PyObject * pyqos_policy_class = PyObject_GetAttrString(pyqos_module, "QoSProfile");
  PyObject * pyqos_profile = NULL;
  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)profile;
  pyqos_profile = PyObject_CallObject(pyqos_policy_class, NULL);
  assert(pyqos_profile != NULL);

  PyObject_SetAttrString(pyqos_profile, "depth", PyLong_FromSize_t(qos_profile->depth));
  PyObject_SetAttrString(pyqos_profile, "history", PyLong_FromUnsignedLong(qos_profile->history));
  PyObject_SetAttrString(pyqos_profile, "reliability",
    PyLong_FromUnsignedLong(qos_profile->reliability));
  PyObject_SetAttrString(pyqos_profile, "durability",
    PyLong_FromUnsignedLong(qos_profile->durability));
  PyObject_SetAttrString(pyqos_profile, "avoid_ros_namespace_conventions",
    PyBool_FromLong(qos_profile->avoid_ros_namespace_conventions));

  assert(pyqos_profile != NULL);
  return pyqos_profile;
}

/// Fetch a predefined qos_profile from rmw and convert it to a Python QoSProfile Object
/**
 * Raises RuntimeError if there is an rcl error
 *
 * This function takes a string defining a rmw_qos_profile_t and returns the
 * corresponding Python QoSProfile object.
 * \param[in] string with the name of the profile to load
 * \return QoSProfile object
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
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_sensor_data);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_default")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_default);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_system_default")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_system_default);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_services_default")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_services_default);
    // NOTE(mikaelarguedas) all conditions following this one are defined but not used
    // because parameters are not implemented in Python yet
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameters")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_parameters);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameter_events")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_parameter_events);
  } else {
    PyErr_Format(PyExc_RuntimeError,
      "Requested unknown rmw_qos_profile: '%s'", pyrmw_profile);
    return NULL;
  }
  return pyqos_profile;
}

/// Define the public methods of this module
static PyMethodDef rclpy_methods[] = {
  {
    "rclpy_init", rclpy_init, METH_VARARGS,
    "Initialize RCL."
  },
  {
    "rclpy_remove_ros_args", rclpy_remove_ros_args, METH_VARARGS,
    "Remove ROS-specific arguments from argument vector"
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
    "rclpy_get_sigint_guard_condition", rclpy_get_sigint_guard_condition, METH_NOARGS,
    "Create a guard_condition triggered when sigint is received."
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
    "rclpy_destroy_node_entity", rclpy_destroy_node_entity, METH_VARARGS,
    "Destroy a Node entity."
  },
  {
    "rclpy_destroy_entity", rclpy_destroy_entity, METH_VARARGS,
    "Destroy an rclpy entity."
  },

  {
    "rclpy_publish", rclpy_publish, METH_VARARGS,
    "Publish a message."
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
    "rclpy_ok", rclpy_ok, METH_NOARGS,
    "rclpy_ok."
  },

  {
    "rclpy_shutdown", rclpy_shutdown, METH_NOARGS,
    "rclpy_shutdown."
  },

  {
    "rclpy_get_node_names", rclpy_get_node_names, METH_VARARGS,
    "Get node names list from graph API."
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
    "Convert a QoSPolicy python object into a rmw_qos_profile_t."
  },

  {
    "rclpy_get_rmw_qos_profile", rclpy_get_rmw_qos_profile, METH_VARARGS,
    "Get QOS profile."
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
  return PyModule_Create(&_rclpymodule);
}
