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

#include "action_api.hpp"
#include "client.hpp"
#include "clock.hpp"
#include "context.hpp"
#include "destroyable.hpp"
#include "duration.hpp"
#include "graph.hpp"
#include "guard_condition.hpp"
#include "handle_api.hpp"
#include "init.hpp"
#include "logging.hpp"
#include "logging_api.hpp"
#include "names.hpp"
#include "node.hpp"
#include "publisher.hpp"
#include "pycapsule_api.hpp"
#include "qos.hpp"
#include "qos_events.hpp"
#include "rclpy_common/exceptions.hpp"
#include "serialization.hpp"
#include "service.hpp"
#include "service_info.hpp"
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

  m.attr("RCL_DEFAULT_DOMAIN_ID") = py::int_(RCL_DEFAULT_DOMAIN_ID);

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

  py::enum_<rcl_subscription_event_type_t>(m, "QoSSubscriptionEventType")
  .value("RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED", RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED)
  .value("RCL_SUBSCRIPTION_LIVELINESS_CHANGED", RCL_SUBSCRIPTION_LIVELINESS_CHANGED)
  .value("RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS", RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS)
  .value("RCL_SUBSCRIPTION_MESSAGE_LOST", RCL_SUBSCRIPTION_MESSAGE_LOST);

  py::enum_<rcl_publisher_event_type_t>(m, "QoSPublisherEventType")
  .value("RCL_PUBLISHER_OFFERED_DEADLINE_MISSED", RCL_PUBLISHER_OFFERED_DEADLINE_MISSED)
  .value("RCL_PUBLISHER_LIVELINESS_LOST", RCL_PUBLISHER_LIVELINESS_LOST)
  .value("RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS", RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS);

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

  m.def(
    "rclpy_init", &rclpy::init,
    "Initialize RCL.");
  m.def(
    "rclpy_shutdown", &rclpy::shutdown,
    "rclpy_shutdown.");

  rclpy::define_client(m);

  m.def(
    "rclpy_context_get_domain_id", &rclpy::context_get_domain_id,
    "Retrieves domain ID from init_options of context");
  m.def(
    "rclpy_create_context", &rclpy::create_context,
    "Create a capsule with an rcl_context_t instance");
  m.def(
    "rclpy_ok", &rclpy::context_is_valid,
    "Return true if the context is valid");

  rclpy::define_duration(m);

  m.def(
    "rclpy_create_publisher", &rclpy::publisher_create,
    "Create a Publisher");
  m.def(
    "rclpy_get_publisher_logger_name", &rclpy::publisher_get_logger_name,
    "Get the logger name associated with the node of a publisher.");
  m.def(
    "rclpy_publisher_get_subscription_count", &rclpy::publisher_get_subscription_count,
    "Count subscribers from a publisher");
  m.def(
    "rclpy_publisher_get_topic_name", &rclpy::publisher_get_topic_name,
    "Get the resolved name(topic) of publisher");
  m.def(
    "rclpy_publish", &rclpy::publisher_publish_message,
    "Publish a message");
  m.def(
    "rclpy_publish_raw", &rclpy::publisher_publish_raw,
    "Publish a serialized message");

  rclpy::define_service(m);

  rclpy::define_service_info(m);

  m.def(
    "rclpy_qos_check_compatible", &rclpy::qos_check_compatible,
    "Check if two QoS profiles are compatible.");

  m.def(
    "rclpy_create_guard_condition", &rclpy::guard_condition_create,
    "Create a general purpose guard condition");
  m.def(
    "rclpy_trigger_guard_condition", &rclpy::guard_condition_trigger,
    "Trigger a general purpose guard condition");

  rclpy::define_timer(m);

  m.def(
    "rclpy_create_subscription", &rclpy::subscription_create,
    "Create a Subscription");
  m.def(
    "rclpy_get_subscription_logger_name", &rclpy::subscription_get_logger_name,
    "Get the logger name associated with the node of a subscription");
  m.def(
    "rclpy_get_subscription_topic_name", &rclpy::subscription_get_topic_name,
    "Get the topic name of a subscription");
  m.def(
    "rclpy_take", &rclpy::subscription_take_message,
    "Take a message and its metadata from a subscription");

  rclpy::define_time_point(m);

  m.def(
    "rclpy_create_clock", &rclpy::create_clock,
    "Create a clock.");
  m.def(
    "rclpy_clock_get_now", &rclpy::clock_get_now,
    "Get the current value of a clock.");
  m.def(
    "rclpy_clock_get_ros_time_override_is_enabled", &rclpy::clock_get_ros_time_override_is_enabled,
    "Get if a clock using ROS time has the ROS time override enabled.");
  m.def(
    "rclpy_clock_set_ros_time_override_is_enabled", &rclpy::clock_set_ros_time_override_is_enabled,
    "Set if a clock using ROS time has the ROS time override enabled.");
  m.def(
    "rclpy_clock_set_ros_time_override", &rclpy::clock_set_ros_time_override,
    "Set the current time of a clock using ROS time.");
  m.def(
    "rclpy_add_clock_callback", &rclpy::add_jump_callback,
    "Add a time jump callback to a clock.");
  m.def(
    "rclpy_remove_clock_callback", &rclpy::remove_jump_callback,
    "Remove a time jump callback from a clock.");

  m.def(
    "rclpy_get_zero_initialized_wait_set", &rclpy::get_zero_initialized_wait_set,
    "rclpy_get_zero_initialized_wait_set.");
  m.def(
    "rclpy_wait_set_init", &rclpy::wait_set_init,
    "rclpy_wait_set_init.");
  m.def(
    "rclpy_wait_set_clear_entities", &rclpy::wait_set_clear_entities,
    "rclpy_wait_set_clear_entities.");
  m.def(
    "rclpy_wait_set_add_entity", &rclpy::wait_set_add_entity,
    "rclpy_wait_set_add_entity.");
  m.def(
    "rclpy_wait_set_add_client", &rclpy::wait_set_add_client,
    "Add a client to the wait set.");
  m.def(
    "rclpy_wait_set_add_service", &rclpy::wait_set_add_service,
    "Add a service to the wait set.");
  m.def(
    "rclpy_wait_set_add_timer", &rclpy::wait_set_add_timer,
    "Add a timer to the wait set.");
  m.def(
    "rclpy_wait_set_is_ready", &rclpy::wait_set_is_ready,
    "rclpy_wait_set_is_ready.");
  m.def(
    "rclpy_get_ready_entities", &rclpy::get_ready_entities,
    "List non null entities in wait set.");
  m.def(
    "rclpy_wait", &rclpy::wait,
    "rclpy_wait.");

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

  m.def(
    "rclpy_create_node", &rclpy::create_node,
    "Create a Node.");
  m.def(
    "rclpy_node_get_fully_qualified_name", &rclpy::get_node_fully_qualified_name,
    "Get the fully qualified name of node.");
  m.def(
    "rclpy_get_node_logger_name", &rclpy::get_node_logger_name,
    "Get the logger name associated with a node.");
  m.def(
    "rclpy_get_node_name", &rclpy::get_node_name,
    "Get the name of a node.");
  m.def(
    "rclpy_get_node_namespace", &rclpy::get_node_namespace,
    "Get the namespace of a node.");
  m.def(
    "rclpy_get_node_names_and_namespaces", &rclpy::get_node_names_and_namespaces,
    "Get node names and namespaces list from graph API.");
  m.def(
    "rclpy_get_node_names_and_namespaces_with_enclaves",
    &rclpy::get_node_names_and_namespaces_with_enclaves,
    "Get node names, namespaces, and enclaves list from graph API.");

  m.def(
    "rclpy_count_publishers", &rclpy::get_count_publishers,
    "Count publishers for a topic.");

  m.def(
    "rclpy_count_subscribers", &rclpy::get_count_subscribers,
    "Count subscribers for a topic.");

  m.def(
    "rclpy_create_event", &rclpy::create_event,
    "Create an event for QoS event handling.");
  m.def(
    "rclpy_take_event", &rclpy::take_event,
    "Get pending data from a ready QoS event.");

  m.def(
    "rclpy_get_node_parameters", &rclpy::get_node_parameters,
    "Get the initial parameters for a node from the command line.");

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

  rclpy::define_pycapsule_api(m);
  rclpy::define_handle_api(m);
  rclpy::define_logging_api(m);
  rclpy::define_action_api(m);
}
