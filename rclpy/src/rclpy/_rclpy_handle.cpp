// Copyright 2020 Open Source Robotics Foundation, Inc.
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
#include <pybind11/pybind11.h>

namespace py = pybind11;

#include "rclpy_common/handle.h"

size_t
rclpy_handle_get_pointer(py::capsule handle_capsule)
{
  void * ptr = rclpy_handle_get_pointer_from_capsule(handle_capsule.ptr(), handle_capsule.name());

  if (!ptr) {
    throw py::error_already_set();
  }

  return reinterpret_cast<size_t>(ptr);
}

PYBIND11_MODULE(_rclpy_handle, m) {
  m.doc() = "rclpy module for working with Handle objects.";
  m.def("rclpy_handle_get_pointer", &rclpy_handle_get_pointer, "Get handle pointer.");
}
