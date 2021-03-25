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

#include <rcl/logging_rosout.h>
#include <rmw/error_handling.h>
#include <rmw/incompatible_qos_events_statuses.h>
#include <rmw/qos_profiles.h>
#include <rmw/types.h>

#include <pybind11/pybind11.h>

#include <string>

#include "qos.hpp"
#include "rclpy_common/exceptions.hpp"
#include "rclpy_common/handle.h"

namespace rclpy
{

QoSCheckCompatibleResult
qos_check_compatible(
  const py::capsule & publisher_qos_profile,
  const py::capsule & subscription_qos_profile
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

  QoSCheckCompatibleResult result;
  const size_t reason_size = 2048u;
  char reason_c_str[reason_size] = "";
  rmw_ret_t ret = rmw_qos_profile_check_compatible(
    *publisher_qos_profile_,
    *subscription_qos_profile_,
    &result.compatibility,
    reason_c_str,
    reason_size);

  if (RMW_RET_OK != ret) {
    std::string error_str(rmw_get_error_string().str);
    rmw_reset_error();
    throw RMWError(error_str);
  }

  result.reason = std::string(reason_c_str);
  return result;
}

}  // namespace rclpy
