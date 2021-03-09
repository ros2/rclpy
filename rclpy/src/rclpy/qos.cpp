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
void
run_my_fn(py::capsule publisher_qos_profile)
{
  std::cout << "it runs!\n";
  if (0 != strcmp("rmw_qos_profile_t", publisher_qos_profile.name())) {
    throw py::value_error("capsule is not an rmw_qos_profile_t");
  }
  auto qos_profile = static_cast<rmw_qos_profile_t *>(
    rclpy_handle_get_pointer_from_capsule(
      publisher_qos_profile.ptr(),
      "rmw_qos_profile_t"
    )
  );
  if (!qos_profile) {
    throw py::error_already_set();
  }

  rmw_qos_compatibility_type_t compatible;
  const size_t reason_size = 2048u;
  char reason_c_str[reason_size];
  rmw_ret_t ret = rmw_qos_profile_check_compatible(
    *qos_profile,
    *qos_profile,
    //*publisher_qos_profile,
    //*publisher_qos_profile,
    //*subscription_qos_profile,
    &compatible,
    reason_c_str,
    reason_size);
  if (RMW_RET_OK != ret) {
    //throw RCLError("Failed to check QoS compatibility: %s", rcl_get_error_string().str);
    throw RCLError("Failed to check QoS compatibility");
  }
}

/*
QoSCheckCompatibleResult
qos_check_compatible(const py::object & publisher_qos, const py::object & subscription_qos)
{
  QoSCheckCompatibleResult result;
  return result;
}
*/
}  // namespace rclpy
