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

#ifndef RCLPY__NODE_HPP_
#define RCLPY__NODE_HPP_

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rclpy
{
/// Get the name of a node.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if name is not set
 *
 * \param[in] pynode Capsule pointing to the node to get the name from
 * \return name of the node
 */
const char *
get_node_name(py::capsule pynode);

/// Get the namespace of a node.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if namespace is not set
 *
 * \param[in] pynode Capsule pointing to the node to get the namespace from
 * \return namespace
 */
const char *
get_node_namespace(py::capsule pynode);

/// Get a list of parameters for the current node
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises ValueError if the argument is not a node handle.
 * Raises RCLError if the parameters file fails to parse
 *
 * \param[in] pyparameter_cls The rclpy.parameter.Parameter class object.
 * \param[in] node_capsule Capsule pointing to the node handle
 * \return A dict mapping parameter names to rclpy.parameter.Parameter (may be empty).
 */
py::dict
get_node_parameters(py::object parameter_cls, py::capsule pynode);
}  // namespace rclpy

#endif  // RCLPY__NODE_HPP_
