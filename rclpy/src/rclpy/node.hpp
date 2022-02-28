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

#include <rcl/node.h>

#include <memory>

#include "context.hpp"
#include "destroyable.hpp"

namespace py = pybind11;

namespace rclpy
{
class Node : public Destroyable, public std::enable_shared_from_this<Node>
{
public:
  /// Create a node
  /**
   * Raises ValueError if the node name or namespace is invalid
   * Raises RCLError if the node could not be initialized for an unexpected reason
   * Raises RCLInvalidROSArgsError if the given CLI arguments could not be parsed
   * Raises UnknownROSArgsError if there are unknown CLI arguments given
   * Raises MemoryError if memory could not be allocated for the node
   *
   * \param[in] node_name name of the node to be created
   * \param[in] namespace_ namespace for the node
   * \param[in] context Context
   * \param[in] pycli_args a sequence of command line arguments for just this node, or None
   * \param[in] use_global_arguments if true then the node will also use cli arguments on context
   * \param[in] enable rosout if true then enable rosout logging
   */
  Node(
    const char * node_name,
    const char * namespace_,
    Context & context,
    py::object pycli_args,
    bool use_global_arguments,
    bool enable_rosout);

  /// Get the fully qualified name of the node.
  /**
   * Raises RCLError if name is not set
   *
   * \return String containing the fully qualified name of the node
   */
  const char *
  get_fully_qualified_name();

  /// Get the name of the logger associated with a node.
  /**
   * Raises RCLError if logger name is not set
   *
   * \return logger_name
   */
  const char *
  logger_name();

  /// Get the name of a node.
  /**
   * Raises RCLError if name is not set
   *
   * \return name of the node
   */
  const char *
  get_node_name();

  /// Get the namespace of a node.
  /**
   * Raises RCLError if namespace is not set
   *
   * \return namespace
   */
  const char *
  get_namespace();

  /// Returns the count of all the publishers known for that topic in the entire ROS graph.
  /**
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] topic_name Name of the topic to count the number of publishers
   * \return the count of all the publishers known for that topic in the entire ROS graph.
   */
  size_t
  get_count_publishers(const char * topic_name);

  /// Returns the count of all the subscribers known for that topic in the entire ROS graph
  /**
   * Raises RCLError if an error occurs in rcl
   *
   * \param[in] topic_name Name of the topic to count the number of subscribers
   * \return the count of all the subscribers known for that topic in the entire ROS graph
   */
  size_t
  get_count_subscribers(const char * topic_name);

  /// Get the list of nodes discovered by the provided node
  /**
   * Raises RCLError if the names are unavailable.
   *
   * \return Python list of tuples where each tuple contains the two strings:
   *   the node name and node namespace
   */
  py::list
  get_node_names_and_namespaces();

  /// Get the list of nodes discovered by the provided node, with their respective enclaves.
  /**
   * Raises RCLError if the names are unavailable.
   *
   * \return Python list of tuples where each tuple contains three strings:
   *   node name, node namespace, and enclave.
   */
  py::list
  get_node_names_and_namespaces_with_enclaves();

  /// Get a list of parameters for the current node
  /**
   * Raises RCLError if any rcl function call fails
   * Raises AttributeError if pyparameter_cls doesn't have a 'Type' attribute
   * Raises RuntimeError if assumptions about rcl structures are violated
   *
   * \param[in] pyparameter_cls The rclpy.parameter.Parameter class object.
   * \return A dict mapping parameter names to rclpy.parameter.Parameter (may be empty).
   */
  py::dict
  get_parameters(py::object pyparameter_cls);

  /// Get action client names and types by node.
  /**
   * \param[in] remote_node_name the node name of the actions to return
   * \param[in] remote_node_namespace the node namespace of the actions to return
   * \return list of action client names and their types
   */
  py::list
  get_action_client_names_and_types_by_node(
    const char * remote_node_name, const char * remote_node_namespace);

  /// Get action server names and types by node.
  /**
   * \param[in] remote_node_name the node name of the actions to return
   * \param[in] remote_node_namespace the node namespace of the actions to return
   * \return list of action server names and their types
   */
  py::list
  get_action_server_names_and_types_by_node(
    const char * remote_node_name, const char * remote_node_namespace);

  /// Get action names and types.
  /**
   * \return list of action names and types
   */
  py::list
  get_action_names_and_types();

  /// Get rcl_node_t pointer
  rcl_node_t *
  rcl_ptr() const
  {
    return rcl_node_.get();
  }

  /// Force an early destruction of this object
  void
  destroy() override;

private:
  /// Get the list of nodes discovered by the provided node
  /**
   * Raises RCLError if the names are unavailable.
   *
   * \param[in] get_enclaves specifies whether the output includes the enclaves names
   * \return Python list of tuples, containing:
   *  node name, node namespace, and
   *  enclave if `get_enclaves` is true.
   */
  py::list
  get_names_impl(bool get_enclaves);

  Context context_;
  std::shared_ptr<rcl_node_t> rcl_node_;
};

void
define_node(py::object module);
}  // namespace rclpy
#endif  // RCLPY__NODE_HPP_
