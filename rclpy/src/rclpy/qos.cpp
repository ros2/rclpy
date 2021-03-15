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

// Include pybind11 before rclpy_common/handle.h includes Python.h
#include <pybind11/pybind11.h>
#include <iostream>

#include "rclpy_common/handle.h"

#include "qos.hpp"

namespace rclpy
{

QoSCheckCompatibleResult
qos_check_compatible(
  const py::capsule &publisher_qos_profile, 
  const py::capsule &subscription_qos_profile
)
{
  if (0 != strcmp("rmw_qos_profile_t", publisher_qos_profile.name())) {
    throw py::value_error("capsule is not an rmw_qos_profile_t");
  }
  auto publisher_qos_profile_ = static_cast<rmw_qos_profile_t *>(publisher_qos_profile);

  if (0 != strcmp("rmw_qos_profile_t", subscription_qos_profile.name())) {
    throw py::value_error("capsule is not an rmw_qos_profile_t");
  }
  auto subscription_qos_profile_ = static_cast<rmw_qos_profile_t *>(subscription_qos_profile);

  rmw_qos_compatibility_type_t compatible;
  const size_t reason_size = 2048u;
  char reason_c_str[reason_size] = "";
  rmw_ret_t ret = rmw_qos_profile_check_compatible(
    *publisher_qos_profile_,
    *subscription_qos_profile_,
    &compatible,
    reason_c_str,
    reason_size);

  if (RMW_RET_OK != ret) {
    auto error_str = rmw_get_error_string().str;
    rmw_reset_error();
    throw RCLError(error_str);
  }

  QoSCheckCompatibleResult result;
  result.reason = std::string(reason_c_str);

  switch (compatible) {
  case RMW_QOS_COMPATIBILITY_OK:
    result.compatibility = QoSCompatibility::Ok;
    break;
  case RMW_QOS_COMPATIBILITY_WARNING:
    result.compatibility = QoSCompatibility::Warning;
    break;
  case RMW_QOS_COMPATIBILITY_ERROR:
    result.compatibility = QoSCompatibility::Error;
    break;
  default:
    auto error_str = "Unexpected compatibility value returned by rmw '" + 
      std::to_string(compatible) + "'";
    throw RCLError(error_str);
  }
  return result;
}

}  // namespace rclpy
