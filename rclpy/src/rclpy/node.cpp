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

#include <rcl/context.h>
#include <rcl/error_handling.h>
#include <rcl/expand_topic_name.h>
#include <rcl/rcl.h>
#include <rcl/remap.h>
#include <rcl/types.h>
#include <rcl/validate_topic_name.h>
#include <rcutils/error_handling.h>
#include <rmw/error_handling.h>
#include <rmw/validate_full_topic_name.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>

#include <memory>
#include <stdexcept>
#include <string>

#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "names.hpp"


namespace rclpy
{
py::object
get_node_name(py::capsule pynode)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  const char * node_name = rcl_node_get_name(node);
  if (!node_name) {
    return py::none();
  }

  return py::str(node_name);
}

py::object
get_node_namespace(py::capsule pynode)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  const char * node_namespace = rcl_node_get_namespace(node);
  if (!node_namespace) {
    return py::none();
  }

  return py::str(node_namespace);
}
}  // namespace rclpy
