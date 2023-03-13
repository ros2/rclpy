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

#include <rcl/domain_id.h>
#include <rcl/service_introspection.h>
#include <rcl/time.h>
#include <rcl_action/types.h>

#include <rmw/qos_profiles.h>
#include <rmw/time.h>

#include "action_client.hpp"
#include "action_goal_handle.hpp"
#include "action_server.hpp"
#include "client.hpp"
#include "clock.hpp"
#include "context.hpp"
#include "destroyable.hpp"
#include "duration.hpp"
#include "clock_event.hpp"
#include "event_handle.hpp"
#include "exceptions.hpp"
#include "graph.hpp"
#include "guard_condition.hpp"
#include "lifecycle.hpp"
#include "logging.hpp"
#include "logging_api.hpp"
#include "names.hpp"
#include "node.hpp"
#include "publisher.hpp"
#include "qos.hpp"
#include "serialization.hpp"
#include "service.hpp"
#include "service_info.hpp"
#include "service_introspection.hpp"
#include "signal_handler.hpp"
#include "subscription.hpp"
#include "time_point.hpp"
#include "timer.hpp"
#include "utils.hpp"
#include "wait_set.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_rclpy_pybind11, m) {
  m.doc() = "ROS 2 Python client library.";

  rclpy::define_destroyable(m);

  py::enum_<rcl_clock_type_t>(m, "ClockType")
  .value("UNINITIALIZED", RCL_CLOCK_UNINITIALIZED)
  .value("ROS_TIME", RCL_ROS_TIME)
  .value("SYSTEM_TIME", RCL_SYSTEM_TIME)
  .value("STEADY_TIME", RCL_STEADY_TIME);

  py::enum_<rcl_action_goal_event_t>(m, "GoalEvent")
  .value("EXECUTE", GOAL_EVENT_EXECUTE)
  .value("CANCEL_GOAL", GOAL_EVENT_CANCEL_GOAL)
  .value("SUCCEED", GOAL_EVENT_SUCCEED)
  .value("ABORT", GOAL_EVENT_ABORT)
  .value("CANCELED", GOAL_EVENT_CANCELED);

  m.attr("RCL_DEFAULT_DOMAIN_ID") = py::int_(RCL_DEFAULT_DOMAIN_ID);
  m.attr("RMW_DURATION_INFINITE") = py::int_(rmw_time_total_nsec(RMW_DURATION_INFINITE));
  m.attr("RMW_QOS_DEADLINE_BEST_AVAILABLE") = py::int_(
    rmw_time_total_nsec(RMW_QOS_DEADLINE_BEST_AVAILABLE));
  m.attr("RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE") = py::int_(
    rmw_time_total_nsec(RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE));

  py::enum_<rcl_clock_change_t>(m, "ClockChange")
  .value(
    "ROS_TIME_NO_CHANGE", RCL_ROS_TIME_NO_CHANGE,
    "ROS time is active and will continue to be active")
  .value(
    "ROS_TIME_ACTIVATED", RCL_ROS_TIME_ACTIVATED,
    "ROS time is being activated")
  .value(
    "ROS_TIME_DEACTIVATED", RCL_ROS_TIME_DEACTIVATED,
    "ROS TIME is being deactivated, the clock will report system time after the jump")
  .value(
    "SYSTEM_TIME_NO_CHANGE", RCL_SYSTEM_TIME_NO_CHANGE,
    "ROS time is inactive and the clock will keep reporting system time");

  py::enum_<rmw_qos_compatibility_type_t>(m, "QoSCompatibility")
  .value("OK", RMW_QOS_COMPATIBILITY_OK)
  .value("WARNING", RMW_QOS_COMPATIBILITY_WARNING)
  .value("ERROR", RMW_QOS_COMPATIBILITY_ERROR);

  py::class_<rclpy::QoSCheckCompatibleResult>(
    m, "QoSCheckCompatibleResult",
    "Result type for checking QoS compatibility with result")
  .def(py::init<>())
  .def_readonly("compatibility", &rclpy::QoSCheckCompatibleResult::compatibility)
  .def_readonly("reason", &rclpy::QoSCheckCompatibleResult::reason);

  py::register_exception<rclpy::RCUtilsError>(m, "RCUtilsError", PyExc_RuntimeError);
  py::register_exception<rclpy::RMWError>(m, "RMWError", PyExc_RuntimeError);
  auto rclerror = py::register_exception<rclpy::RCLError>(m, "RCLError", PyExc_RuntimeError);
  py::register_exception<rclpy::RCLInvalidROSArgsError>(
    m, "RCLInvalidROSArgsError", rclerror.ptr());
  py::register_exception<rclpy::UnknownROSArgsError>(m, "UnknownROSArgsError", PyExc_RuntimeError);
  py::register_exception<rclpy::NodeNameNonExistentError>(
    m, "NodeNameNonExistentError", rclerror.ptr());
  py::register_exception<rclpy::UnsupportedEventTypeError>(
    m, "UnsupportedEventTypeError", rclerror.ptr());
  py::register_exception<rclpy::NotImplementedError>(
    m, "NotImplementedError", PyExc_NotImplementedError);
  py::register_exception<rclpy::InvalidHandle>(
    m, "InvalidHandle", PyExc_RuntimeError);

  rclpy::define_service_introspection(m);

  rclpy::define_client(m);

  rclpy::define_context(m);

  rclpy::define_duration(m);

  rclpy::define_publisher(m);

  rclpy::define_service(m);

  rclpy::define_service_info(m);

  m.def(
    "rclpy_qos_check_compatible", &rclpy::qos_check_compatible,
    "Check if two QoS profiles are compatible.");

  rclpy::define_action_client(m);
  rclpy::define_action_goal_handle(m);
  rclpy::define_action_server(m);
  m.def(
    "rclpy_action_get_rmw_qos_profile", &rclpy::rclpy_action_get_rmw_qos_profile,
    "Get an action RMW QoS profile.");
  rclpy::define_guard_condition(m);
  rclpy::define_timer(m);
  rclpy::define_subscription(m);
  rclpy::define_time_point(m);
  rclpy::define_clock(m);
  rclpy::define_waitset(m);

  m.def(
    "rclpy_expand_topic_name", &rclpy::expand_topic_name,
    "Expand a topic name.");
  m.def(
    "rclpy_remap_topic_name", &rclpy::remap_topic_name,
    "Remap a topic name.");
  m.def(
    "rclpy_get_validation_error_for_topic_name", &rclpy::get_validation_error_for_topic_name,
    "Get the error message and invalid index of a topic name or None if valid.");
  m.def(
    "rclpy_get_validation_error_for_full_topic_name",
    &rclpy::get_validation_error_for_full_topic_name,
    "Get the error message and invalid index of a full topic name or None if valid.");
  m.def(
    "rclpy_get_validation_error_for_namespace", &rclpy::get_validation_error_for_namespace,
    "Get the error message and invalid index of a namespace or None if valid.");
  m.def(
    "rclpy_get_validation_error_for_node_name", &rclpy::get_validation_error_for_node_name,
    "Get the error message and invalid index of a node name or None if valid.");
  m.def(
    "rclpy_resolve_name", &rclpy::resolve_name,
    "Expand and remap a topic or service name.");

  m.def(
    "rclpy_get_topic_names_and_types",
    &rclpy::graph_get_topic_names_and_types,
    "Get all topic names and types in the ROS graph.");
  m.def(
    "rclpy_get_publisher_names_and_types_by_node",
    &rclpy::graph_get_publisher_names_and_types_by_node,
    "Get topic names and types for which a remote node has publishers.");
  m.def(
    "rclpy_get_subscriber_names_and_types_by_node",
    &rclpy::graph_get_subscriber_names_and_types_by_node,
    "Get topic names and types for which a remote node has subscribers.");
  m.def(
    "rclpy_get_publishers_info_by_topic",
    &rclpy::graph_get_publishers_info_by_topic,
    "Get publishers info for a topic.");
  m.def(
    "rclpy_get_subscriptions_info_by_topic",
    &rclpy::graph_get_subscriptions_info_by_topic,
    "Get subscriptions info for a topic.");
  m.def(
    "rclpy_get_service_names_and_types",
    &rclpy::graph_get_service_names_and_types,
    "Get all service names and types in the ROS graph.");
  m.def(
    "rclpy_get_service_names_and_types_by_node",
    &rclpy::graph_get_service_names_and_types_by_node,
    "Get service names and types for which a remote node has servers.");
  m.def(
    "rclpy_get_client_names_and_types_by_node",
    &rclpy::graph_get_client_names_and_types_by_node,
    "Get service names and types for which a remote node has clients.");

  m.def(
    "rclpy_serialize", &rclpy::serialize,
    "Serialize a ROS message.");
  m.def(
    "rclpy_deserialize", &rclpy::deserialize,
    "Deserialize a ROS message.");

  rclpy::define_node(m);
  rclpy::define_event_handle(m);

  m.def(
    "rclpy_get_rmw_implementation_identifier",
    &rclpy::get_rmw_implementation_identifier,
    "Retrieve the identifier for the active RMW implementation.");

  m.def(
    "rclpy_assert_liveliness", &rclpy::assert_liveliness,
    "Assert the liveliness of an entity.");

  m.def(
    "rclpy_remove_ros_args", &rclpy::remove_ros_args,
    "Remove ROS-specific arguments from argument vector.");

  rclpy::define_rmw_qos_profile(m);

  m.def(
    "rclpy_logging_fini", rclpy::logging_fini,
    "Finalize RCL logging.");
  m.def(
    "rclpy_logging_configure", rclpy::logging_configure,
    "Initialize RCL logging.");

  rclpy::define_logging_api(m);
  rclpy::define_signal_handler_api(m);
  rclpy::define_clock_event(m);
  rclpy::define_lifecycle_api(m);
}
