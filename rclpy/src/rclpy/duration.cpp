// Copyright 2021 Open Source Robotics Foundation, Inc.
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

// Include pybind11 before rclpy_common/handle.h includes Python.h
#include <pybind11/pybind11.h>

#include <rcl/time.h>

#include <memory>

#include "duration.hpp"

namespace rclpy
{
void
_rclpy_destroy_duration(PyObject * pycapsule)
{
  PyMem_Free(PyCapsule_GetPointer(pycapsule, "rcl_duration_t"));
}

py::capsule
create_duration(int64_t nanoseconds)
{
  auto duration = static_cast<rcl_duration_t *>(PyMem_Malloc(sizeof(rcl_duration_t)));
  if (!duration) {
    throw std::bad_alloc();
  }

  duration->nanoseconds = nanoseconds;

  return py::capsule(duration, "rcl_duration_t", _rclpy_destroy_duration);
}

int64_t
duration_get_nanoseconds(py::capsule pyduration)
{
  if (0 != strcmp("rcl_duration_t", pyduration.name())) {
    throw py::value_error("capsule is not an rcl_duration_t");
  }
  auto duration = static_cast<rcl_duration_t *>(pyduration);

  return duration->nanoseconds;
}
}  // namespace rclpy
