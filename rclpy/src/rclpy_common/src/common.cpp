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

#include "rcl/error_handling.h"

#include "rclpy_common/common.hpp"

namespace rclpy
{

py::list
convert_to_py_names_and_types(rcl_names_and_types_t * names_and_types)
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

}  // namespace rclpy
