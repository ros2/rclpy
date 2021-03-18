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

#ifndef RCLPY__UTILS_HPP_
#define RCLPY__UTILS_HPP_

#include <pybind11/pybind11.h>

#include <rcl/graph.h>  // rcl_names_and_types_t

#include <memory>

namespace py = pybind11;

namespace rclpy
{

typedef void destroy_ros_message_function (void *);

/// Convert a C rcl_names_and_types_t into a Python list.
/**
 * \param[in] names_and_types The names and types struct to convert.
 * \return List of tuples, where the first element of each tuple is a string
 *   for the name and the second element is a list of strings for the types.
 */
py::list
convert_to_py_names_and_types(const rcl_names_and_types_t * topic_names_and_types);

/// Create the equivalent ROS message C type instance for a given Python type.
/**
* Raises AttributeError if \p pyclass is missing a required attribute.
*
* \param[in] pyclass ROS message Python type to extract typesupport data from.
* \return a ROS message C type instance.
*/
std::unique_ptr<void, destroy_ros_message_function *>
create_from_py(py::object pyclass);

/// Convert a ROS message from a C type to a Python type.
/**
 * Raises AttributeError if \p pyclass is missing a required attribute.
 *
 * \param[in] message ROS message C type instance to be converted.
 * \param[in] pyclass ROS message Python type to convert to.
 * \return an instance of \p pyclass.
 */
py::object
convert_to_py(void * message, py::object pyclass);

}  // namespace rclpy

#endif  // RCLPY__UTILS_HPP_
