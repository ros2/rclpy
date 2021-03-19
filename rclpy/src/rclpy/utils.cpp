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

#include <memory>

#include "rcl/error_handling.h"

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

}  // namespace rclpy
