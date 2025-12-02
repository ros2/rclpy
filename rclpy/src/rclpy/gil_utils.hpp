// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef RCLPY__GIL_UTILS_HPP_
#define RCLPY__GIL_UTILS_HPP_

#include <pybind11/pybind11.h>

#ifndef Py_PACK_FULL_VERSION
#define Py_PACK_FULL_VERSION(X, Y, Z, LEVEL, SERIAL) ( \
   (((X) & 0xff) << 24) |                              \
   (((Y) & 0xff) << 16) |                              \
   (((Z) & 0xff) << 8) |                               \
   (((LEVEL) & 0xf) << 4) |                            \
   (((SERIAL) & 0xf) << 0))
#endif

#if PY_VERSION_HEX < Py_PACK_FULL_VERSION(3, 13, 0, 0, 0)
#define rclpy_IsPythonFinalizing _Py_IsFinalizing
#else
#define rclpy_IsPythonFinalizing Py_IsFinalizing
#endif

namespace rclpy
{

class gil_scoped_release
{
public:
  gil_scoped_release() = default;

  ~gil_scoped_release() {
    // Avoid taking the GIL if runtime is finalizing
    if (rclpy_IsPythonFinalizing()) {
      gil_release_.disarm();
    }
  }

private:

  pybind11::gil_scoped_release gil_release_;
};

}  // namespace rclpy

#endif  // RCLPY__GIL_UTILS_HPP_
