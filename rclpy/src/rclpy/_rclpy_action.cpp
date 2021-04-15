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


template<typename T>
T
get_pointer(py::capsule & capsule, const char * name)
{
  if (strcmp(name, capsule.name())) {
    std::string error_text{"Expected capusle with name '"};
    error_text += name;
    error_text += "' but got '";
    error_text += capsule.name();
    error_text += "'";
    throw py::value_error(error_text);
  }
  // TODO(sloretz) use get_pointer() in pybind11 2.6+
  return static_cast<T>(capsule);
}

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

py::object
rclpy_action_get_client_names_and_types_by_node(
  rclpy::Node & node, const char * remote_node_name, const char * remote_node_namespace)
{
  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_client_names_and_types_by_node(
    node.rcl_ptr(),
    &allocator,
    remote_node_name,
    remote_node_namespace,
    &names_and_types);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get action client names and type");
  }

  py::object pynames_and_types = py::reinterpret_steal<py::object>(
    rclpy_convert_to_py_names_and_types(&names_and_types));
  if (!rclpy_names_and_types_fini(&names_and_types)) {
    throw py::error_already_set();
  }
  return pynames_and_types;
}

py::object
rclpy_action_get_server_names_and_types_by_node(
  rclpy::Node & node, const char * remote_node_name, const char * remote_node_namespace)
{
  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_server_names_and_types_by_node(
    node.rcl_ptr(),
    &allocator,
    remote_node_name,
    remote_node_namespace,
    &names_and_types);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get action server names and type");
  }

  py::object pynames_and_types = py::reinterpret_steal<py::object>(
    rclpy_convert_to_py_names_and_types(&names_and_types));
  if (!rclpy_names_and_types_fini(&names_and_types)) {
    throw py::error_already_set();
  }
  return pynames_and_types;
}

py::object
rclpy_action_get_names_and_types(rclpy::Node & node)
{
  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_names_and_types(node.rcl_ptr(), &allocator, &names_and_types);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get action names and type");
  }

  py::object pynames_and_types = py::reinterpret_steal<py::object>(
    rclpy_convert_to_py_names_and_types(&names_and_types));
  if (!rclpy_names_and_types_fini(&names_and_types)) {
    throw py::error_already_set();
  }
  return pynames_and_types;
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

  m.def(
    "rclpy_action_get_client_names_and_types_by_node",
    &rclpy_action_get_client_names_and_types_by_node,
    "Get action client names and types by node.");
  m.def(
    "rclpy_action_get_server_names_and_types_by_node",
    &rclpy_action_get_server_names_and_types_by_node,
    "Get action server names and types by node.");
  m.def(
    "rclpy_action_get_names_and_types", &rclpy_action_get_names_and_types,
    "Get action names and types.");

  rclpy::define_action_client(m);
}
}  // namespace rclpy
