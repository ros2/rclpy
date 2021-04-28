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
/// Get the fully qualified name of the node.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if name is not set
 *
 * \param[in] pynode Capsule pointing to the node
 * \return String containing the fully qualified name of the node
 */
const char *
get_node_fully_qualified_name(py::capsule pynode);

/// Get the name of the logger associated with a node.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if logger name is not set
 *
 * \param[in] pynode Capsule pointing to the node to get the logger name of
 * \return logger_name
 */
const char *
get_node_logger_name(py::capsule pynode);

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

/// Returns the count of all the publishers known for that topic in the entire ROS graph.
/*
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if an error occurs in rcl
 *
 * \param[in] pynode pynode Capsule pointing to a node
 * \param[in] topic_name Name of the topic to count the number of publishers
 * \return the count of all the publishers known for that topic in the entire ROS graph.
 */
size_t
get_count_publishers(py::capsule pynode, const char * topic_name);

/// Returns the count of all the subscribers known for that topic in the entire ROS graph
/*
 *
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if an error occurs in rcl
 *
 * \param[in] pynode pynode Capsule pointing to a node
 * \param[in] topic_name Name of the topic to count the number of subscribers
 * \return the count of all the subscribers known for that topic in the entire ROS graph
 */
size_t
get_count_subscribers(py::capsule pynode, const char * topic_name);

/// Get the list of nodes discovered by the provided node
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if the names are unavailable.
 *
 * \param[in] module the Python module this function is part of
 * \param[in] pynode Capsule pointing to the node
 * \param[in] get_enclaves specifies if the output includes the enclaves names
 *            or not
 * \return Python list of tuples, containing:
 *  node name, node namespace, and
 *  enclave if `get_enclaves` is true.
 */
py::list
get_node_names_impl(py::capsule pynode, bool get_enclaves);

/// Get the list of nodes discovered by the provided node
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if the names are unavailable.
 *
 * \param[in] pynode Capsule pointing to the node
 * \return Python list of tuples where each tuple contains the two strings:
 *   the node name and node namespace
 */
py::list
get_node_names_and_namespaces(py::capsule pynode);

/// Get the list of nodes discovered by the provided node, with their respective enclaves.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if the names are unavailable.
 *
 * \param[in] pynode Capsule pointing to the node
 * \return Python list of tuples where each tuple contains three strings:
 *   node name, node namespace, and enclave.
 */
py::list
get_node_names_and_namespaces_with_enclaves(py::capsule pynode);

/// Get a list of parameters for the current node
/**
 * Raises ValueError if the argument is not a node handle
 * Raises RCLError if any rcl function call fails
 * Raises AttributeError if pyparameter_cls doesn't have a 'Type' attribute
 * Raises RuntimeError if assumptions about rcl structures are violated
 *
 * \param[in] pyparameter_cls The rclpy.parameter.Parameter class object.
 * \param[in] node_capsule Capsule pointing to the node handle
 * \return A dict mapping parameter names to rclpy.parameter.Parameter (may be empty).
 */
py::dict
get_node_parameters(py::object pyparameter_cls, py::capsule pynode);

/// Create a node
/**
 * Raises ValueError if the node name or namespace is invalid
 * Raises RCLError if the node could not be initialized for an unexpected reason
 * Raises RCLInvalidROSArgsError if the given CLI arguments could not be parsed
 * Raises UnknownROSArgsError if there are unknown CLI arguments given
 * Raises MemoryError if memory could not be allocated for the node
 *
 * \param[in] node_name name of the node to be created
 * \param[in] namespace namespace for the node
 * \param[in] pycontext Capsule for an rcl_context_t
 * \param[in] pycli_args a sequence of command line arguments for just this node, or None
 * \param[in] use_global_arguments if true then the node will also use cli arguments on pycontext
 * \param[in] enable rosout if true then enable rosout logging
 * \return Capsule of the pointer to the created rcl_node_t * structure
 */
py::capsule
create_node(
  const char * node_name,
  const char * namespace_,
  py::capsule pycontext,
  py::object pycli_args,
  bool use_global_arguments,
  bool enable_rosout);
}  // namespace rclpy

#endif  // RCLPY__NODE_HPP_
