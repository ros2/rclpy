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

#include <string>

#include "node.hpp"

namespace py = pybind11;

namespace rclpy
{

/// Get topic names and types for which a remote node has publishers.
/**
 * Raises NodeNameNonExistentError if remote node was not found
 * Raises RCLError if there is an rcl error
 *
 * \param[in] node Node to get publisher topic names and types.
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
  Node & node, bool no_demangle,
  std::string node_name, std::string node_namespace);

/// Get topic names and types for which a remote node has subscribers.
/**
 * Raises NodeNameNonExistentError if the remote node was not found
 * Raises RCLError if there is an rcl error
 *
 * \param[in] node node to get subscriber topic names and types
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
  Node & node, bool no_demangle,
  std::string node_name, std::string node_namespace);

/// Get service names and types for which a remote node has servers.
/**
 * Raises NodeNameNonExistentError if the remote node was not found
 * Raises RCLError if there is an rcl error
 *
 * \param[in] node Node to get service topic names and types
 * \param[in] node_name Name of the remote node to query.
 * \param[in] node_namespace Namespace of the remote node to query.
 * \return List of tuples, where the first element of each tuple is the service
 *   name (string) and the second element is a list of service types (list of
 *   strings).
 * \see rcl_get_service_names_and_types_by_node
 */
py::list
graph_get_service_names_and_types_by_node(
  Node & node, std::string node_name, std::string node_namespace);

/// Get service names and types for which a remote node has clients.
/**
 * Raises NodeNameNonExistentError if the remote node was not found
 * Raises RCLError if there is an rcl error
 *
 * \param[in] node node to get client topic names and types
 * \param[in] node_name Name of the remote node to query.
 * \param[in] node_namespace Namespace of the remote node to query.
 * \return List of tuples, where the first element of each tuple is the service
 *   name (string) and the second element is a list of service types (list of
 *   strings).
 * \see rcl_get_client_names_and_types_by_node
 */
py::list
graph_get_client_names_and_types_by_node(
  Node & node, std::string node_name, std::string node_namespace);

/// Get all topic names and types in the ROS graph.
/**
 * Raises RCLError if there is an rcl error
 *
 * \param[in] node node to get topic names and types
 * \param[in] no_demangle Whether to demangle topic names following ROS
 *   conventions or not.
 * \return List of tuples, where the first element of each tuple is the topic
 *   name (string) and the second element is a list of topic types (list of
 *   strings).
 * \see rcl_get_topic_names_and_types
 */
py::list
graph_get_topic_names_and_types(Node & node, bool no_demangle);

/// Get all service names and types in the ROS graph.
/**
 * Raises RCLError if there is an rcl error
 *
 * \param[in] node Node to get all topic service names and types
 * \return List of tuples, where the first element of each tuple is the service
 *   name (string) and the second element is a list of service types (list of
 *   strings).
 * \see rcl_get_service_names_and_types
 */
py::list
graph_get_service_names_and_types(Node & node);

/// Return a list of publishers on a given topic.
/**
 * The returned publisher information includes node name, node namespace, topic type, gid,
 * and qos profile.
 *
 * Raises NotImplementedError if the call is not supported by RMW
 * Raises RCLError if there is an rcl error
 *
 * \param[in] node Node to get topic publisher info
 * \param[in] topic_name the topic name to get the publishers for.
 * \param[in] no_mangle if `true`, `topic_name` needs to be a valid middleware topic name,
 *     otherwise it should be a valid ROS topic name.
 * \return list of publishers
 */
py::list
graph_get_publishers_info_by_topic(
  Node & node, const char * topic_name, bool no_mangle);

/// Return a list of subscriptions on a given topic.
/**
 * The returned subscription information includes node name, node namespace, topic type, gid,
 * and qos profile.
 *
 * Raises NotImplementedError if the call is not supported by RMW
 * Raises RCLError if there is an rcl error
 *
 * \param[in] node node to get topic subscriber info
 * \param[in] topic_name the topic name to get the subscriptions for.
 * \param[in] no_mangle if `true`, `topic_name` needs to be a valid middleware topic name,
 *     otherwise it should be a valid ROS topic name.
 * \return list of subscriptions.
 */
py::list
graph_get_subscriptions_info_by_topic(
  Node & node, const char * topic_name, bool no_mangle);

}  // namespace rclpy

#endif  // RCLPY__GRAPH_HPP_
