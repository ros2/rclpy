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

#ifndef RCLPY__SIGNAL_HANDLER_HPP_
#define RCLPY__SIGNAL_HANDLER_HPP_

#include <pybind11/pybind11.h>

#include <functional>
#include <memory>

namespace py = pybind11;

namespace rclpy
{

/// Register a C++ callback to be invoked when a signal is caught.  These callbacks will be invoked
/// on an arbitrary thread.  The callback will be automatically unregistered if the object goes out
/// of scope.
class ScopedSignalCallback {
public:
  explicit ScopedSignalCallback(std::function<void()>);
  ~ScopedSignalCallback();

private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};

/// Define methods on a module for working with signal handlers
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_signal_handler_api(py::module module);

}  // namespace rclpy
#endif  // RCLPY__SIGNAL_HANDLER_HPP_
