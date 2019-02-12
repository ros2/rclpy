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

#include <Python.h>

#include <rmw/types.h>

typedef void * create_ros_message_signature (void);
typedef void destroy_ros_message_signature (void *);
typedef bool convert_from_py_signature (PyObject *, void *);
typedef PyObject * convert_to_py_signature (void *);

/// Convert a C rmw_qos_profile_t into a Python QoSProfile object
/**
 * \param[in] void pointer to a rmw_qos_profile_t structure
 * \return QoSProfile object
 */
PyObject *
rclpy_convert_to_py_qos_policy(void * profile);

void *
get_capsule_pointer(PyObject * pymetaclass, const char * attr);

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
PyObject *
rclpy_convert_to_py(void * message, PyObject * pyclass);

#endif  // RCLPY_COMMON__COMMON_H_
