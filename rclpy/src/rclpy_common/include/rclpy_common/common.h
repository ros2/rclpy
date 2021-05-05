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

/// Convert a C rmw_qos_profile_t into a Python dictionary with qos profile args.
/**
 * \param[in] profile Pointer to a rmw_qos_profile_t to convert
 * \return Python dictionary
 */
RCLPY_COMMON_PUBLIC
PyObject *
rclpy_common_convert_to_qos_dict(const rmw_qos_profile_t * profile);

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
