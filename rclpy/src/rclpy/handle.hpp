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

#ifndef RCLPY__HANDLE_HPP_
#define RCLPY__HANDLE_HPP_

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rclpy
{
/// RAII wrapper around rclpy_handle_t that keeps it alive
class Handle
{
public:
  Handle() = default;

  /// Create a handle instance wrapping a pycapsule to a type that uses rclpy_handle_t
  explicit Handle(py::capsule pyhandle);

  ~Handle();

  /// assignment operator
  Handle & operator=(const Handle & other);

  /// Get the rcl pointer belonging to the handle or throw an exception
  template<typename T>
  T
  cast(const char * capsule_name)
  {
    void * ptr = rcl_ptr(capsule_name);
    if (!ptr) {
      throw py::error_already_set();
    }
    return static_cast<T>(ptr);
  }

  /// Get the rcl pointer belonging to the handle or print a warning
  template<typename T>
  T
  cast_or_warn(const char * capsule_name) noexcept
  {
    void * ptr = rcl_ptr(capsule_name);
    if (!ptr) {
      PyErr_Clear();
      int stack_level = 1;
      PyErr_WarnFormat(PyExc_RuntimeWarning, stack_level, "Failed to get rclpy_handle_t pointer");
    }
    return static_cast<T>(ptr);
  }

private:
  py::capsule pyhandle_;

  void inc_ref();
  void dec_ref();

  void *
  rcl_ptr(const char * capsule_name) noexcept;
};
}  // namespace rclpy

#endif  // RCLPY__HANDLE_HPP_
