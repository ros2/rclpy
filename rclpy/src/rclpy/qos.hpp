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

#include <pybind11/pybind11.h>

#include <rmw/qos_profiles.h>

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
  const rmw_qos_profile_t & publisher_qos_profile,
  const rmw_qos_profile_t & subscription_qos_profile
);

/// Define a pybind11 wrapper for an rmw_qos_profile_t
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_rmw_qos_profile(py::object module);

}  // namespace rclpy

#endif  // RCLPY__QOS_HPP_
