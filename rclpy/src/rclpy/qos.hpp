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

#include "rclpy_common/exceptions.hpp"
#include "rmw/qos_profiles.h"
#include "rcl/logging_rosout.h"
#include "rmw/incompatible_qos_events_statuses.h"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include "rmw/error_handling.h"

namespace py = pybind11;

namespace rclpy
{
enum class QoSCompatibility
{
  Ok = RMW_QOS_COMPATIBILITY_OK,
  Warning = RMW_QOS_COMPATIBILITY_WARNING,
  Error = RMW_QOS_COMPATIBILITY_ERROR,
};

/// Result type for checking QoS compatibility
/**
 * \see rclcpp::qos_check_compatible()
 */
struct QoSCheckCompatibleResult
{
  /// Compatibility result.
  QoSCompatibility compatibility;

  /// Reason for a (possible) incompatibility.
  /**
   * Set if compatiblity is QoSCompatibility::Warning or QoSCompatiblity::Error.
   * Not set if the QoS profiles are compatible.
   */
  std::string reason;
};

/// Check if two QoS profiles are compatible.
/**
 * Two QoS profiles are compatible if a publisher and subcription
 * using the QoS policies can communicate with each other.
 *
 * If any policies have value "system default" or "unknown" then it is possible that
 * compatiblity cannot be determined.
 * In this case, the value QoSCompatility::Warning is set as part of
 * the returned structure.
 *
 * \param[in] publisher_qos_profile: The QoS profile for a publisher.
 * \param[in] subscription_qos_profile: The QoS profile for a subscription.
 * \return Struct with compatiblity set to QoSCompatibility::Ok if the QoS profiles are
 *   compatible, or
 * \return Struct with compatibility set to QoSCompatibility::Warning if there is a chance
 *   the QoS profiles are not compatible, or
 * \return Struct with compatibility set to QoSCompatibility::Error if the QoS profiles are
 *   not compatible.
 * \throws RCLError if an unexpected error occurs.
 */
QoSCheckCompatibleResult
qos_check_compatible(
  const py::capsule &publisher_qos_profile, 
  const py::capsule &subscription_qos_profile
);

}  // namespace rclpy

#endif  // RCLPY__QOS_HPP_
