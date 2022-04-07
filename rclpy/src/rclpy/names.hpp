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

#ifndef RCLPY__NAMES_HPP_
#define RCLPY__NAMES_HPP_

#include <pybind11/pybind11.h>

#include <string>

#include "node.hpp"

namespace py = pybind11;

namespace rclpy
{
/// Validate a topic name and return error message and index of invalidation.
/**
 * Does not have to be a fully qualified topic name.
 * The topic name is not expanded.
 *
 * Raises MemoryError if memory could not be allocated
 * Raises RCLError if an unexpected error happened while validating the topic name
 *
 * \param[in] topic_name name of the topic to be validated
 * \return tuple of error message and invalid index if invalid, or
 * \return None if valid
 */
py::object
get_validation_error_for_topic_name(const char * topic_name);

/// Validate a full topic name and return error message and index of invalidation.
/**
 * Must be a fully qualified topic name.
 *
 * Raises MemoryError if memory could not be allocated
 * Raises RMWError if an unexpected error happened while validating the topic name
 *
 * \param[in] topic_name name of the topic to be validated
 * \return tuple of error message and invalid index if invalid, or
 * \return None if valid
 */
py::object
get_validation_error_for_full_topic_name(const char * topic_name);

/// Validate a namespace and return error message and index of invalidation.
/**
 * Raises MemoryError if memory could not be allocated
 * Raises RMWError if an unexpected error happened while validating the namespace
 *
 * \param[in] namespace_ namespace to be validated
 * \return tuple of error message and invalid index if invalid, or
 * \return None if valid
 */
py::object
get_validation_error_for_namespace(const char * namespace_);

/// Validate a node name and return error message and index of invalidation.
/**
 * Raises MemoryError if memory could not be allocated
 * Raises RMWError if an unexpected error happened while validating the node name
 *
 * \param[in] node_name name of the node to be validated
 * \return tuple of error message and invalid index if invalid, or
 * \return None if valid
 */
py::object
get_validation_error_for_node_name(const char * node_name);

/// Expand a topic name
/**
 * Raises ValueError if the topic name, node name, or namespace are not valid.
 * Raises RCLError or RCUtilsError if an unexpected error happens
 *
 * \param[in] topic topic string to be expanded
 * \param[in] node_name name of the node to be used during expansion
 * \param[in] node_namespace namespace of the node to be used during expansion
 * \return expanded topic name
 */
std::string
expand_topic_name(const char * topic, const char * node_name, const char * node_namespace);

/// Remap a topic name
/**
 * Raises ValueError if the capsule is not the correct type
 * Raises RCLError if an unexpected error happens
 *
 * \param[in] node node to remap the topic name
 * \param[in] topic_name topic string to be remapped
 * \return remapped topic name
 */
std::string
remap_topic_name(Node & node, const char * topic_name);

/// Expand and remap a topic name
/**
 * Raises ValueError if the capsule is not the correct type
 * Raises RCLError if an unexpected error happens
 *
 * \param[in] node node to expand and remap a topic name
 * \param[in] topic_name topic string to be remapped
 * \param[in] only_expand when `false`, remapping rules are ignored
 * \param[in] is_service `true` for service names, `false` for topic names
 * \return expanded and remapped topic name
 */
std::string
resolve_name(Node & node, const char * topic_name, bool only_expand, bool is_service);
}  // namespace rclpy

#endif  // RCLPY__NAMES_HPP_
