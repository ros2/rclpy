// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#ifndef RCLPY_COMMON__COMMON_H_
#define RCLPY_COMMON__COMMON_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <Python.h>

#include <rcl/graph.h>  // rcl_names_and_types_t
#include <rcl/rcl.h>
#include <rmw/types.h>

#include "rclpy_common/visibility_control.h"

typedef void * create_ros_message_signature (void);
typedef void destroy_ros_message_signature (void *);
typedef bool convert_from_py_signature (PyObject *, void *);
typedef PyObject * convert_to_py_signature (void *);

/// Get the type support structure for a Python ROS message type.
/**
 * \param[in] pymsg_type The Python ROS message type.
 * \return The type support structure or NULL if an error occurred.
 */
RCLPY_COMMON_PUBLIC
void *
rclpy_common_get_type_support(PyObject * pymsg_type);

/// Convert a C rmw_qos_profile_t into a Python dictionary with qos profile args.
/**
 * \param[in] profile Pointer to a rmw_qos_profile_t to convert
 * \return Python dictionary
 */
RCLPY_COMMON_PUBLIC
PyObject *
rclpy_common_convert_to_qos_dict(const rmw_qos_profile_t * profile);

RCLPY_COMMON_PUBLIC
void *
get_capsule_pointer(PyObject * pymetaclass, const char * attr);

RCLPY_COMMON_PUBLIC
void *
rclpy_create_from_py(PyObject * pymessage, destroy_ros_message_signature ** destroy_ros_message);

/// Convert a ROS message from a Python type to a C type.
/**
 * Raises AttributeError if the Python message type is missing a required attribute.
 * Raises MemoryError on a memory allocation failure.
 *
 * \param[in] pymessage The Python message to convert from.
 * \param[out] destroy_ros_message The destructor function for finalizing the returned message.
 * \return The C version of the input ROS message.
 */
RCLPY_COMMON_PUBLIC
void *
rclpy_convert_from_py(PyObject * pymessage, destroy_ros_message_signature ** destroy_ros_message);

/// Convert a ROS message from a C type to a Python type.
/**
 * Raises AttributeError if the Python type is missing a required attribute.
 *
 * \param[in] message The C message to convert to a Python type
 * \param[in] pyclass An instance of the Python type to convert to.
 * \return The Python version of the input ROS message.
 */
RCLPY_COMMON_PUBLIC
PyObject *
rclpy_convert_to_py(void * message, PyObject * pyclass);

/// Convert a C rmw_topic_endpoint_info_array_t into a Python list.
/**
 * \param[in] info_array a pointer to a rmw_topic_endpoint_info_array_t
 * \return Python list
 */
RCLPY_COMMON_PUBLIC
PyObject *
rclpy_convert_to_py_topic_endpoint_info_list(const rmw_topic_endpoint_info_array_t * info_array);

#ifdef __cplusplus
}
#endif

#endif  // RCLPY_COMMON__COMMON_H_
