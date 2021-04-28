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
#include <rcpputils/scope_exit.hpp>

#include <limits>
#include <memory>
#include <vector>

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"
#include "rclpy_common/exceptions.hpp"

#include "init.hpp"
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
assert_liveliness(py::capsule pyentity)
{
  if (0 == strcmp("rclpy_publisher_t", pyentity.name())) {
    auto publisher = static_cast<rclpy_publisher_t *>(
      rclpy_handle_get_pointer_from_capsule(
        pyentity.ptr(), "rclpy_publisher_t"));
    if (nullptr == publisher) {
      throw py::error_already_set();
    }
    if (RCL_RET_OK != rcl_publisher_assert_liveliness(&publisher->publisher)) {
      throw RCLError("Failed to assert liveliness on the Publisher");
    }
  } else {
    throw py::type_error("Passed capsule is not a valid Publisher.");
  }
}

py::list
remove_ros_args(py::object pycli_args)
{
  rcl_ret_t ret;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_arguments_t parsed_args = rcl_get_zero_initialized_arguments();

  std::vector<const char *> arg_values;
  const char ** const_arg_values = NULL;
  py::list pyargs;
  if (!pycli_args.is_none()) {
    pyargs = pycli_args;
    if (!pyargs.empty()) {
      arg_values.resize(pyargs.size());
      for (size_t i = 0; i < pyargs.size(); ++i) {
        // CPython owns const char * memory - no need to free it
        arg_values[i] = PyUnicode_AsUTF8(pyargs[i].ptr());
        if (!arg_values[i]) {
          throw py::error_already_set();
        }
      }
      const_arg_values = &(arg_values[0]);
    }
  }

  if (arg_values.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw py::value_error("too many cli arguments");
  }

  int num_args = static_cast<int>(arg_values.size());
  ret = rcl_parse_arguments(num_args, const_arg_values, allocator, &parsed_args);

  if (RCL_RET_INVALID_ROS_ARGS == ret) {
    throw RCLInvalidROSArgsError("Failed to parse ROS arguments");
  }
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to parse arguments");
  }

  RCPPUTILS_SCOPE_EXIT(
    {
      ret = rcl_arguments_fini(&parsed_args);
      if (RCL_RET_OK != ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "rcl_arguments_fini failed: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
    });

  throw_if_unparsed_ros_args(pyargs, parsed_args);

  int nonros_argc = 0;
  const char ** nonros_argv = NULL;

  ret = rcl_remove_ros_arguments(
    const_arg_values,
    &parsed_args,
    allocator,
    &nonros_argc,
    &nonros_argv);
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed rcl_remove_ros_arguments");
  }

/* it was determined that the following warning is likely a front-end parsing issue in MSVC.
 * See: https://github.com/ros2/rclpy/pull/180#issuecomment-375452757
 */
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4090)
#endif
  RCPPUTILS_SCOPE_EXIT(
    {
      allocator.deallocate(nonros_argv, allocator.state);
    });
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

  py::list result_args(nonros_argc);
  for (int i = 0; i < nonros_argc; ++i) {
    result_args[i] = nonros_argv[i];
  }

  return result_args;
}
}  // namespace rclpy
