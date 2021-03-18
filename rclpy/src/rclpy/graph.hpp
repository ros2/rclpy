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

#ifndef RCLPY__GRAPH_HPP_
#define RCLPY__GRAPH_HPP_

#include <pybind11/pybind11.h>

#include <rcl/graph.h>

#include <memory>
#include <string>

namespace py = pybind11;

namespace rclpy
{

/// Get topic names and types for which a remote node has publishers.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises NodeNameNonExistentError if remote node was not found
 * Raises RCLError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node.
 * \param[in] no_demangle Whether to demangle topic names following ROS
 *   conventions or not.
 * \param[in] node_name Name of the remote node to query.
 * \param[in] node_namespace Namespace of the remote node to query.
 * \return List of tuples, where the first element of each tuple is the topic
 *   name (string) and the second element is a list of topic types (list of
 *   strings).
 * \see rcl_get_publisher_names_and_types_by_node
 */
py::list
graph_get_publisher_names_and_types_by_node(
  py::capsule pynode, bool no_demangle,
  std::string node_name, std::string node_namespace);

/// Get topic names and types for which a remote node has subscribers.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises NodeNameNonExistentError if the remote node was not found
 * Raises RCLError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node.
 * \param[in] no_demangle Whether to demangle topic names following ROS
 *   conventions or not.
 * \param[in] node_name Name of the remote node to query.
 * \param[in] node_namespace Namespace of the remote node to query.
 * \return List of tuples, where the first element of each tuple is the topic
 *   name (string) and the second element is a list of topic types (list of
 *   strings).
 * \see rcl_get_subscriber_names_and_types_by_node
 */
py::list
graph_get_subscriber_names_and_types_by_node(
  py::capsule pynode, bool no_demangle,
  std::string node_name, std::string node_namespace);

/// Get service names and types for which a remote node has servers.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises NodeNameNonExistentError if the remote node was not found
 * Raises RCLError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node.
 * \param[in] node_name Name of the remote node to query.
 * \param[in] node_namespace Namespace of the remote node to query.
 * \return List of tuples, where the first element of each tuple is the service
 *   name (string) and the second element is a list of service types (list of
 *   strings).
 * \see rcl_get_service_names_and_types_by_node
 */
py::list
graph_get_service_names_and_types_by_node(
  py::capsule pynode, std::string node_name, std::string node_namespace);

/// Get service names and types for which a remote node has clients.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises NodeNameNonExistentError if the remote node was not found
 * Raises RCLError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node.
 * \param[in] node_name Name of the remote node to query.
 * \param[in] node_namespace Namespace of the remote node to query.
 * \return List of tuples, where the first element of each tuple is the service
 *   name (string) and the second element is a list of service types (list of
 *   strings).
 * \see rcl_get_client_names_and_types_by_node
 */
py::list
graph_get_client_names_and_types_by_node(
  py::capsule pynode, std::string node_name, std::string node_namespace);

/// Get all topic names and types in the ROS graph.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node to query the ROS graph.
 * \param[in] no_demangle Whether to demangle topic names following ROS
 *   conventions or not.
 * \return List of tuples, where the first element of each tuple is the topic
 *   name (string) and the second element is a list of topic types (list of
 *   strings).
 * \see rcl_get_topic_names_and_types
 */
py::list
graph_get_topic_names_and_types(py::capsule pynode, bool no_demangle);

/// Get all service names and types in the ROS graph.
/**
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if there is an rcl error
 *
 * \param[in] pynode Capsule pointing to the node to query the ROS graph.
 * \return List of tuples, where the first element of each tuple is the service
 *   name (string) and the second element is a list of service types (list of
 *   strings).
 * \see rcl_get_service_names_and_types
 */
py::list
graph_get_service_names_and_types(py::capsule pynode);

}  // namespace rclpy

#endif  // RCLPY__GRAPH_HPP_
