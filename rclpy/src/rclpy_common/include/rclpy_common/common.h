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

#include <rcl/graph.h>  // rcl_names_and_types_t
#include <rcl/rcl.h>
#include <rmw/types.h>

#include "rclpy_common/visibility_control.h"

typedef void * create_ros_message_signature (void);
typedef void destroy_ros_message_signature (void *);
typedef bool convert_from_py_signature (PyObject *, void *);
typedef PyObject * convert_to_py_signature (void *);

typedef struct
{
  // Important: a pointer to a structure is also a pointer to its first member.
  // The subscription must be first in the struct to compare sub.handle.pointer to an address
  // in a wait set.
  rcl_subscription_t subscription;
  rcl_node_t * node;
} rclpy_subscription_t;

typedef struct
{
  rcl_publisher_t publisher;
  rcl_node_t * node;
} rclpy_publisher_t;

typedef struct
{
  // Important: a pointer to a structure is also a pointer to its first member.
  // The client must be first in the struct to compare cli.handle.pointer to an address
  // in a wait set.
  rcl_client_t client;
  rcl_node_t * node;
} rclpy_client_t;

typedef struct
{
  // Important: a pointer to a structure is also a pointer to its first member.
  // The service must be first in the struct to compare srv.handle.pointer to an address
  // in a wait set.
  rcl_service_t service;
  rcl_node_t * node;
} rclpy_service_t;

/// Finalize names and types struct with error setting.
/**
 * \param[in] names_and_types The struct to finalize.
 * \return `true` if finalized successfully, `false` otherwise.
 *   If `false`, then a Python error is set.
 */
RCLPY_COMMON_PUBLIC
bool
rclpy_names_and_types_fini(rcl_names_and_types_t * names_and_types);

/// Convert a C rcl_names_and_types_t into a Python list.
/**
 * \param names_and_types The names and types struct to convert.
 * \return A PyList of PyTuples. The first element of each tuple is a string for the
 *   name and the second element is a list of strings for the types.
 * \return `NULL` if there is an error. No Python error is set.
 */
RCLPY_COMMON_PUBLIC
PyObject *
rclpy_convert_to_py_names_and_types(rcl_names_and_types_t * topic_names_and_types);

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

#endif  // RCLPY_COMMON__COMMON_H_
