// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <pybind11/pybind11.h>

#include <rcl/error_handling.h>
#include <rcl_action/rcl_action.h>

#include <cstring>
#include <memory>
#include <string>

#include "rclpy_common/exceptions.hpp"

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"

#include "action_client.hpp"
#include "action_goal_handle.hpp"
#include "action_server.hpp"
#include "clock.hpp"
#include "node.hpp"

namespace py = pybind11;

/// Fetch a predefined qos_profile from rcl_action and convert it to a Python QoSProfile object.
/**
 * Raises RuntimeError if the QoS profile is unknown.
 *
 * This function takes a string defining a rmw_qos_profile_t and returns the
 * corresponding Python QoSProfile object.
 * \param[in] rmw_profile String with the name of the profile to load.
 * \return QoSProfile object.
 */
py::dict
rclpy_action_get_rmw_qos_profile(const char * rmw_profile)
{
  PyObject * pyqos_profile = NULL;
  if (0 == strcmp(rmw_profile, "rcl_action_qos_profile_status_default")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rcl_action_qos_profile_status_default);
  } else {
    std::string error_text = "Requested unknown rmw_qos_profile: ";
    error_text += rmw_profile;
    throw std::runtime_error(error_text);
  }
  return py::reinterpret_steal<py::dict>(pyqos_profile);
}

namespace rclpy
{
void
define_action_api(py::module m)
{
  m.def(
    "rclpy_action_get_rmw_qos_profile", &rclpy_action_get_rmw_qos_profile,
    "Get an action RMW QoS profile.");
  define_action_goal_handle(m);
  define_action_server(m);
  rclpy::define_action_client(m);
}
}  // namespace rclpy
