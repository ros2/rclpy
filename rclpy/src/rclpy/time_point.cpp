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

#include <rcl/rcl.h>

#include <cstring>
#include <memory>

#include "time_point.hpp"

namespace rclpy
{
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
py::capsule
create_time_point(int64_t nanoseconds, rcl_clock_type_t clock_type)
{
  auto deleter = [](rcl_time_point_t * ptr) {PyMem_FREE(ptr);};
  auto time_point = std::unique_ptr<rcl_time_point_t, decltype(deleter)>(
    static_cast<rcl_time_point_t *>(PyMem_Malloc(sizeof(rcl_time_point_t))),
    deleter);

  time_point->nanoseconds = nanoseconds;
  time_point->clock_type = clock_type;

  return py::capsule(time_point.release(), "rcl_time_point_t", _rclpy_destroy_time_point);
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
int64_t
time_point_get_nanoseconds(py::capsule pytime_point)
{
  if (0 != std::strcmp("rcl_time_point_t", pytime_point.name())) {
    throw py::value_error("capsule is not an rcl_time_point_t");
  }
  auto time_point = static_cast<rcl_time_point_t *>(pytime_point);
  return time_point->nanoseconds;
}
}  // namespace rclpy
