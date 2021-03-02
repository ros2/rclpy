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

#include "context.hpp"
#include "rclpy_common/exceptions.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_rclpy_pybind11, m) {
  m.doc() = "ROS 2 Python client library.";

  auto rclerror = py::register_exception<rclpy::RCLError>(m, "RCLError", PyExc_RuntimeError);
  py::register_exception<rclpy::RCLInvalidROSArgsError>(
    m, "RCLInvalidROSArgsError", rclerror.ptr());
  py::register_exception<rclpy::UnknownROSArgsError>(m, "UnknownROSArgsError", rclerror.ptr());
  py::register_exception<rclpy::NodeNameNonExistentError>(
    m, "NodeNameNonExistentError", rclerror.ptr());
  py::register_exception<rclpy::UnsupportedEventTypeError>(
    m, "UnsupportedEventTypeError", rclerror.ptr());

  m.def(
    "rclpy_context_get_domain_id", &rclpy::context_get_domain_id,
    "Retrieves domain ID from init_options of context");
  m.def(
    "rclpy_create_context", &rclpy::create_context,
    "Create a capsule with an rcl_context_t instance");
  m.def(
    "rclpy_ok", &rclpy::context_is_valid,
    "Return true if the context is valid");
}
