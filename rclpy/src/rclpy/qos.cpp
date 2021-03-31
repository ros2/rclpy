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

#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rmw/error_handling.h>
#include <rmw/incompatible_qos_events_statuses.h>
#include <rmw/qos_profiles.h>
#include <rmw/types.h>

#include <cstring>
#include <memory>
#include <string>

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "qos.hpp"

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
  rmw_ret_t ret = rmw_qos_profile_check_compatible(
    *publisher_qos_profile_,
    *subscription_qos_profile_,
    &result.compatibility,
    result.reason,
    sizeof(QoSCheckCompatibleResult::reason));

  if (RMW_RET_OK != ret) {
    throw RMWError("failed to check if qos profiles are compatible");
  }

  return result;
}

namespace
{

// Fill a given rmw_time_t from an rclpy.duration.Duration instance.
void
_convert_py_duration_to_rmw_time(const rcl_duration_t duration, rmw_time_t * out_time)
{
  out_time->sec = RCL_NS_TO_S(duration.nanoseconds);
  out_time->nsec = duration.nanoseconds % (1000LL * 1000LL * 1000LL);
}

}  // namespace

py::capsule
convert_from_py_qos_policy(
  int qos_history,
  int qos_depth,
  int qos_reliability,
  int qos_durability,
  const rcl_duration_t pyqos_lifespan,
  const rcl_duration_t pyqos_deadline,
  int qos_liveliness,
  const rcl_duration_t pyqos_liveliness_lease_duration,
  bool avoid_ros_namespace_conventions)
{
  auto qos_profile =
    std::unique_ptr<rmw_qos_profile_t, decltype(&PyMem_Free)>(
    static_cast<rmw_qos_profile_t *>(
      PyMem_Malloc(sizeof(rmw_qos_profile_t))), PyMem_Free);
  if (!qos_profile) {
    throw std::bad_alloc();
  }
  // Set to default so that we don't use uninitialized data if new fields are added
  *qos_profile = rmw_qos_profile_default;

  // Overwrite defaults with passed values
  qos_profile->history = static_cast<rmw_qos_history_policy_t>(qos_history);
  qos_profile->depth = qos_depth;
  qos_profile->reliability = static_cast<rmw_qos_reliability_policy_t>(qos_reliability);
  qos_profile->durability = static_cast<rmw_qos_durability_policy_t>(qos_durability);

  _convert_py_duration_to_rmw_time(pyqos_lifespan, &(qos_profile->lifespan));
  _convert_py_duration_to_rmw_time(pyqos_deadline, &(qos_profile->deadline));

  qos_profile->liveliness = static_cast<rmw_qos_liveliness_policy_t>(qos_liveliness);

  _convert_py_duration_to_rmw_time(
    pyqos_liveliness_lease_duration,
    &(qos_profile->liveliness_lease_duration));

  qos_profile->avoid_ros_namespace_conventions = avoid_ros_namespace_conventions;

  return py::capsule(
    qos_profile.release(), "rmw_qos_profile_t",
    [](PyObject * pycapsule_c) {
      PyMem_Free(PyCapsule_GetPointer(pycapsule_c, "rmw_qos_profile_t"));
    });
}

py::dict
convert_to_py_qos_policy(py::capsule pyqos_profile)
{
  if (0 != strcmp("rmw_qos_profile_t", pyqos_profile.name())) {
    throw py::value_error("capsule is not an rmw_qos_profile_t");
  }
  auto qos_profile = static_cast<rmw_qos_profile_t *>(pyqos_profile);
  PyObject * pydict = rclpy_common_convert_to_qos_dict(qos_profile);
  if (!pydict) {
    throw py::error_already_set();
  }
  return py::reinterpret_steal<py::dict>(pydict);
}

py::dict
get_rmw_qos_profile(const char * qos_profile_name)
{
  PyObject * pyqos_profile = nullptr;
  if (0 == strcmp(qos_profile_name, "qos_profile_sensor_data")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_sensor_data);
  } else if (0 == strcmp(qos_profile_name, "qos_profile_default")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_default);
  } else if (0 == strcmp(qos_profile_name, "qos_profile_system_default")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_system_default);
  } else if (0 == strcmp(qos_profile_name, "qos_profile_services_default")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_services_default);
  } else if (0 == strcmp(qos_profile_name, "qos_profile_unknown")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_unknown);
  } else if (0 == strcmp(qos_profile_name, "qos_profile_parameters")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_parameters);
  } else if (0 == strcmp(qos_profile_name, "qos_profile_parameter_events")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_parameter_events);
  } else {
    std::string error_text = "Requested unknown rmw_qos_profile: ";
    error_text += qos_profile_name;
    throw std::invalid_argument(error_text);
  }
  if (!pyqos_profile) {
    throw py::error_already_set();
  }
  return py::reinterpret_steal<py::dict>(pyqos_profile);
}

}  // namespace rclpy
