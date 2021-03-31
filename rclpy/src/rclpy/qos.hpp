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

#ifndef RCLPY__QOS_HPP_
#define RCLPY__QOS_HPP_

#include <rmw/qos_profiles.h>

#include <pybind11/pybind11.h>


namespace py = pybind11;

namespace rclpy
{

/// Result type for checking QoS compatibility
/**
 * \see rclpy::qos_check_compatible()
 */
struct QoSCheckCompatibleResult
{
  /// Compatibility result.
  rmw_qos_compatibility_type_t compatibility;

  /// Reason for a (possible) incompatibility.
  /**
   * Set if compatiblity is RMW_QOS_COMPATIBILITY_WARNING or RMW_QOS_COMPATIBILITY_ERROR.
   * Not set if the QoS profiles are compatible.
   */
  char reason[2048];
};

/// Check if two QoS profiles are compatible.
/**
 * Two QoS profiles are compatible if a publisher and subcription
 * using the QoS policies can communicate with each other.
 *
 * If any policies have value "system default" or "unknown" then it is possible that
 * compatiblity cannot be determined.
 * In this case, the value RMW_QOS_COMPATIBILITY_WARNING is set as part of
 * the returned structure.
 *
 * \param[in] publisher_qos_profile: The QoS profile for a publisher.
 * \param[in] subscription_qos_profile: The QoS profile for a subscription.
 * \return Struct with compatiblity set to RMW_QOS_COMPATIBILITY_OK if the QoS profiles are
 *   compatible, or
 * \return Struct with compatibility set to RMW_QOS_COMPATIBILITY_WARNING if there is a chance
 *   the QoS profiles are not compatible, or
 * \return Struct with compatibility set to RMW_QOS_COMPATIBILITY_ERROR if the QoS profiles are
 *   not compatible.
 * \throws RMWError if an unexpected error occurs.
 */
QoSCheckCompatibleResult
qos_check_compatible(
  const py::capsule & publisher_qos_profile,
  const py::capsule & subscription_qos_profile
);

/// Convert rclpy.qos.QoSProfile arguments to a C rmw_qos_profile_t capsule.
/**
 * Raises ValueError if a any capsule is not of the expected type.
 * Raises MemoryError if rmw_qos_profile_t allocation fails.
 *
 * \param[in] qos_history an rclpy.qos.QoSHistoryPolicy value.
 * \param[in] qos_depth depth of the message queue.
 * \param[in] qos_reliability an rclpy.qos.QoSReliabilityPolicy value.
 * \param[in] qos_durability an rclpy.qos.QoSDurabilityPolicy value.
 * \param[in] pyqos_lifespan lifespan QoS policy parameter
 *   as an rcl_duration_t capsule.
 * \param[in] pyqos_deadline deadline QoS policy parameter
 *   as an rcl_duration_t capsule.
 * \param[in] qos_liveliness an rclpy.qos.QoSLivelinessPolicy value.
 * \param[in] pyqos_liveliness_lease_duration livelines QoS policy
 *   lease duration as an rcl_duration_t capsule.
 * \param[in] avoid_ros_namespace_conventions Whether to use ROS
 *   namespace conventions or not.
 * \return Capsule pointing to a rmw_qos_profile_t C struct.
 */
py::capsule
convert_from_py_qos_policy(
  int qos_history,
  int qos_depth,
  int qos_reliability,
  int qos_durability,
  rcl_duration_t pyqos_lifespan,
  rcl_duration_t pyqos_deadline,
  int qos_liveliness,
  rcl_duration_t pyqos_liveliness_lease_duration,
  bool avoid_ros_namespace_conventions);

/// Convert a C rmw_qos_profile_t capsule to a dictionary.
/**
 * This function is exposed to facilitate testing profile type conversion.
 *
 * Raises ValueError if a \p pyqos_profile is not an rmw_qos_profile_t capsule.
 *
 * \param[in] pyqos_profile Capsule pointing to rmw_qos_profile_t
 * \return a dictionary suitable for rclpy.qos.QoSProfile instantiation.
 */
py::dict
convert_to_py_qos_policy(py::capsule pyqos_profile);

/// Fetch a predefined rclpy.qos.QoSProfile keyword arguments.
/**
 * Raises InvalidArgument if the given \p rmw_profile is unknown.
 *
 * \param[in] qos_profile_name Name of the profile to fetch.
 * \return a dictionary suitable for rclpy.qos.QoSProfile instantiation.
 */
py::dict
get_rmw_qos_profile(const char * qos_profile_name);

}  // namespace rclpy

#endif  // RCLPY__QOS_HPP_
