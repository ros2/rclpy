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

/// Count publishers for a topic.
/*
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if rcl_count_publishers doesn't work
 *
 * \param[in] pynode Capsule pointing to the node to get number of subscribers
 * for a topic
 * \param[in] topic_name Name of the topic to count the number of publishers
 * \return count of publishers
 */
unsigned int
get_count_publishers(py::capsule pynode, const char * topic_name);

/// Count subscribers for a topic.
/*
 * Raises ValueError if pynode is not a node capsule
 * Raises RCLError if rcl_count_subscribers doesn't work
 *
 * \param[in] pynode Capsule pointing to the node to get number of subscribers
 * for a topic
 * \param[in] topic_name Name of the topic to count the number of subscribers
 * \return count of subscribers
 */
unsigned int
get_count_subscribers(py::capsule pynode, const char * topic_name);

}  // namespace rclpy

#endif  // RCLPY__NODE_HPP_
