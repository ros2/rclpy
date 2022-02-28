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

#include <pybind11/pybind11.h>

#include <rcl/time.h>
#include <rmw/error_handling.h>
#include <rmw/incompatible_qos_events_statuses.h>
#include <rmw/qos_profiles.h>
#include <rmw/time.h>
#include <rmw/types.h>

#include <cstring>
#include <stdexcept>
#include <string>

#include "exceptions.hpp"
#include "qos.hpp"
#include "utils.hpp"

namespace rclpy
{

QoSCheckCompatibleResult
qos_check_compatible(
  const rmw_qos_profile_t & publisher_qos_profile,
  const rmw_qos_profile_t & subscription_qos_profile)
{
  QoSCheckCompatibleResult result;
  rmw_ret_t ret = rmw_qos_profile_check_compatible(
    publisher_qos_profile,
    subscription_qos_profile,
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
_convert_py_duration_to_rmw_time(const rcl_duration_t & duration, rmw_time_t * out_time)
{
  out_time->sec = RCL_NS_TO_S(duration.nanoseconds);
  out_time->nsec = duration.nanoseconds % (1000LL * 1000LL * 1000LL);
}

/// Create an rmw_qos_profile_t instance.
/**
 * Raises ValueError if a any capsule is not of the expected type.
 * Raises MemoryError if rmw_qos_profile_t allocation fails.
 *
 * \param[in] qos_history an rclpy.qos.QoSHistoryPolicy value.
 * \param[in] qos_depth depth of the message queue.
 * \param[in] qos_reliability an rclpy.qos.QoSReliabilityPolicy value.
 * \param[in] qos_durability an rclpy.qos.QoSDurabilityPolicy value.
 * \param[in] pyqos_lifespan lifespan QoS policy parameter of
 *   rcl_duration_t type.
 * \param[in] pyqos_deadline deadline QoS policy parameter of
 *   rcl_duration_t type.
 * \param[in] qos_liveliness an rclpy.qos.QoSLivelinessPolicy value.
 * \param[in] pyqos_liveliness_lease_duration livelines QoS policy
 *   lease duration of rcl_duration_t type.
 * \param[in] avoid_ros_namespace_conventions Whether to use ROS
 *   namespace conventions or not.
 * \return an rmw_qos_profile_t instance.
 */
rmw_qos_profile_t
create_qos_profile(
  int qos_history,
  int qos_depth,
  int qos_reliability,
  int qos_durability,
  const rcl_duration_t & pyqos_lifespan,
  const rcl_duration_t & pyqos_deadline,
  int qos_liveliness,
  const rcl_duration_t & pyqos_liveliness_lease_duration,
  bool avoid_ros_namespace_conventions)
{
  // Set to default so that we don't use uninitialized data if new fields are added
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

  // Overwrite defaults with passed values
  qos_profile.history = static_cast<rmw_qos_history_policy_t>(qos_history);
  qos_profile.depth = qos_depth;
  qos_profile.reliability = static_cast<rmw_qos_reliability_policy_t>(qos_reliability);
  qos_profile.durability = static_cast<rmw_qos_durability_policy_t>(qos_durability);

  _convert_py_duration_to_rmw_time(pyqos_lifespan, &(qos_profile.lifespan));
  _convert_py_duration_to_rmw_time(pyqos_deadline, &(qos_profile.deadline));

  qos_profile.liveliness = static_cast<rmw_qos_liveliness_policy_t>(qos_liveliness);

  _convert_py_duration_to_rmw_time(
    pyqos_liveliness_lease_duration,
    &(qos_profile.liveliness_lease_duration));

  qos_profile.avoid_ros_namespace_conventions = avoid_ros_namespace_conventions;

  return qos_profile;
}

/// Fetch a predefined rmw_qos_profile_t instance.
/**
 * Raises InvalidArgument if the given \p qos_profile_name is unknown.
 *
 * \param[in] qos_profile_name Name of the profile to fetch.
 * \return an rmw_qos_profile_t instance
 */
rmw_qos_profile_t
predefined_qos_profile_from_name(const char * qos_profile_name)
{
  if (0 == strcmp(qos_profile_name, "qos_profile_sensor_data")) {
    return rmw_qos_profile_sensor_data;
  }
  if (0 == strcmp(qos_profile_name, "qos_profile_default")) {
    return rmw_qos_profile_default;
  }
  if (0 == strcmp(qos_profile_name, "qos_profile_system_default")) {
    return rmw_qos_profile_system_default;
  }
  if (0 == strcmp(qos_profile_name, "qos_profile_services_default")) {
    return rmw_qos_profile_services_default;
  }
  if (0 == strcmp(qos_profile_name, "qos_profile_unknown")) {
    return rmw_qos_profile_unknown;
  }
  if (0 == strcmp(qos_profile_name, "qos_profile_parameters")) {
    return rmw_qos_profile_parameters;
  }
  if (0 == strcmp(qos_profile_name, "qos_profile_parameter_events")) {
    return rmw_qos_profile_parameter_events;
  }

  std::string error_text = "Requested unknown rmw_qos_profile: ";
  error_text += qos_profile_name;
  throw std::invalid_argument(error_text);
}

}  // namespace

void
define_rmw_qos_profile(py::object module)
{
  py::class_<rmw_qos_profile_t>(module, "rmw_qos_profile_t")
  .def(py::init<>(&create_qos_profile))
  .def("to_dict", &convert_to_qos_dict)
  .def_static("predefined", &predefined_qos_profile_from_name);
}

}  // namespace rclpy
