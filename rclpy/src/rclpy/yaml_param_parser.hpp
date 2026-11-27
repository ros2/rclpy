// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef RCLPY__YAML_PARAM_PARSER_HPP_
#define RCLPY__YAML_PARAM_PARSER_HPP_

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rclpy
{
/// Define a pybind11 wrapper for parsing a YAML parameter file with
/// rcl_parse_yaml_file().
/**
 * \\param[in] module a pybind11 module to add the definition to
 */
void define_yaml_param_parser(py::module module);
}  // namespace rclpy

#endif  // RCLPY__YAML_PARAM_PARSER_HPP_
