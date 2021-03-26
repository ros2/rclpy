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

#include <pybind11/pybind11.h>

#include "rclpy_common/handle.h"

#include "handle.hpp"

namespace rclpy
{
Handle::Handle(py::capsule pyhandle)
: pyhandle_(pyhandle)
{
  inc_ref();
}

Handle::~Handle()
{
  dec_ref();
}

Handle &
Handle::operator=(const Handle & other)
{
  dec_ref();

  pyhandle_ = other.pyhandle_;

  inc_ref();

  return *this;
}

void Handle::inc_ref()
{
  if (pyhandle_.ptr()) {
    // Forced to assume this uses rclpy_handle_t because pycapsule name gives no clues
    auto c_handle = static_cast<rclpy_handle_t *>(pyhandle_);
    // Increment the reference count
    _rclpy_handle_inc_ref(c_handle);
  }
}

void Handle::dec_ref()
{
  if (pyhandle_.ptr()) {
    // Forced to assume this uses rclpy_handle_t because pycapsule name gives no clues
    auto c_handle = static_cast<rclpy_handle_t *>(pyhandle_);
    // Decrement the reference count
    _rclpy_handle_dec_ref(c_handle);
  }
}

void *
Handle::rcl_ptr(const char * capsule_name) noexcept
{
  return rclpy_handle_get_pointer_from_capsule(pyhandle_.ptr(), capsule_name);
}
}  // namespace rclpy
