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

#include <assert.h>

#include <pybind11/pybind11.h>

#include <rcl/error_handling.h>

#include <memory>
#include <string>

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"
#include "rclpy_common/exceptions.hpp"

#include "utils.hpp"

namespace rclpy
{

py::list
convert_to_py_names_and_types(const rcl_names_and_types_t * names_and_types)
{
  assert(names_and_types);

  py::list py_names_and_types(names_and_types->names.size);
  for (size_t i = 0u; i < names_and_types->names.size; ++i) {
    py::list py_types(names_and_types->types[i].size);
    for (size_t j = 0u; j < names_and_types->types[i].size; ++j) {
      py_types[j] = py::str(names_and_types->types[i].data[j]);
    }
    py_names_and_types[i] = py::make_tuple(
      py::str(names_and_types->names.data[i]), py_types);
  }
  return py_names_and_types;
}

std::unique_ptr<void, destroy_ros_message_function *>
create_from_py(py::object pymessage)
{
  typedef void * create_ros_message_function (void);

  py::object pymetaclass = pymessage.attr("__class__");

  py::object value = pymetaclass.attr("_CREATE_ROS_MESSAGE");
  auto capsule_ptr = static_cast<void *>(value.cast<py::capsule>());
  auto create_ros_message =
    reinterpret_cast<create_ros_message_function *>(capsule_ptr);
  if (!create_ros_message) {
    throw py::error_already_set();
  }

  value = pymetaclass.attr("_DESTROY_ROS_MESSAGE");
  capsule_ptr = static_cast<void *>(value.cast<py::capsule>());
  auto destroy_ros_message =
    reinterpret_cast<destroy_ros_message_function *>(capsule_ptr);
  if (!destroy_ros_message) {
    throw py::error_already_set();
  }

  void * message = create_ros_message();
  if (!message) {
    throw std::bad_alloc();
  }
  return std::unique_ptr<
    void, destroy_ros_message_function *>(message, destroy_ros_message);
}

py::object
convert_to_py(void * message, py::object pyclass)
{
  py::object pymetaclass = pyclass.attr("__class__");

  auto capsule_ptr = static_cast<void *>(
    pymetaclass.attr("_CONVERT_TO_PY").cast<py::capsule>());

  typedef PyObject * convert_to_py_function (void *);
  auto convert = reinterpret_cast<convert_to_py_function *>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }
  return py::reinterpret_steal<py::object>(convert(message));
}

const char *
get_rmw_implementation_identifier()
{
  return rmw_get_implementation_identifier();
}

void
assert_liveliness(py::object pyentity)
{
  if (PyCapsule_IsValid(pyentity.ptr(), "rclpy_publisher_t")) {
    auto publisher = static_cast<rclpy_publisher_t *>(
      rclpy_handle_get_pointer_from_capsule(
        pyentity.ptr(), "rclpy_publisher_t"));
    if (RCL_RET_OK != rcl_publisher_assert_liveliness(&publisher->publisher)) {
      throw RCLError(
              std::string("Failed to assert liveliness on the Publisher: ") +
              rcl_get_error_string().str);
    }
  } else {
    throw UnsupportedObjectTypeError("Passed capsule is not a valid Publisher.");
  }
}

/// Deallocate a list of allocated strings.
void
_arg_list_fini(int num_args, char ** argv)
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
_pyargs_to_list(PyObject * pyargs, int * num_args, char *** arg_values)
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
        _arg_list_fini(i, *arg_values);
        Py_DECREF(pyargs);
        // Exception raised
        return RCL_RET_ERROR;
      }
      const char * arg_str = PyUnicode_AsUTF8(pyarg);
      (*arg_values)[i] = rcutils_strdup(arg_str, allocator);
      if (!(*arg_values)[i]) {
        _arg_list_fini(i, *arg_values);
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
/**
 * \param[in] pyargs a sequence of string args
 * \param[in] unknown_ros_args_count the number of unknown ROS args
 * \param[in] unknown_ros_args_indices the indices to unknown ROS args
 */
void
_raise_unknown_ros_args(
  rclpy_module_state_t * module_state,
  PyObject * pyargs,
  const int * unknown_ros_args_indices,
  int unknown_ros_args_count)
{
  PyObject * unknown_ros_pyargs = NULL;
  if (!module_state) {
    PyErr_Format(PyExc_RuntimeError, "_raise_unknown_ros_args got NULL module state");
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
/**
 * Raises TypeError of pyargs is not a sequence
 * Raises OverflowError if len(pyargs) > INT_MAX
 * \param[in] pyargs a Python sequence of strings
 * \param[out] parsed_args a zero initialized pointer to rcl_arguments_t
 */
rcl_ret_t
_parse_args(
  rclpy_module_state_t * module_state, PyObject * pyargs, rcl_arguments_t * parsed_args)
{
  if (!module_state) {
    PyErr_Format(PyExc_RuntimeError, "_parse_args got NULL module state");
    return RCL_RET_ERROR;
  }
  rcl_ret_t ret;

  int num_args = 0;
  char ** arg_values = NULL;
  // Py_None is a singleton so comparing pointer value works
  if (Py_None != pyargs) {
    ret = _pyargs_to_list(pyargs, &num_args, &arg_values);
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
    _raise_unknown_ros_args(
      module_state, pyargs, unparsed_ros_args_indices, unparsed_ros_args_count);
    allocator.deallocate(unparsed_ros_args_indices, allocator.state);
    ret = RCL_RET_ERROR;
  }
cleanup:
  _arg_list_fini(num_args, arg_values);
  return ret;
}

py::list
remove_ros_args(py::list pyargs)
{
  rcl_ret_t ret;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  int num_args;
  char ** arg_values;
  ret = _pyargs_to_list(pyargs, &num_args, &arg_values);
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
    _arg_list_fini(num_args, arg_values);
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
    _arg_list_fini(num_args, arg_values);
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

  _arg_list_fini(num_args, arg_values);

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
}  // namespace rclpy
