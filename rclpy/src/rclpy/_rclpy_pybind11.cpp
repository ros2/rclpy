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

#include "context.hpp"
#include "guard_condition.hpp"
#include "rclpy_common/exceptions.hpp"
#include "service_info.hpp"
#include "subscription.hpp"
#include "timer.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_rclpy_pybind11, m) {
  m.doc() = "ROS 2 Python client library.";

  auto rclerror = py::register_exception<rclpy::RCLError>(m, "RCLError", PyExc_RuntimeError);
  py::register_exception<rclpy::RCLInvalidROSArgsError>(
    m, "RCLInvalidROSArgsError", rclerror.ptr());
  py::register_exception<rclpy::UnknownROSArgsError>(m, "UnknownROSArgsError", rclerror.ptr());
  py::register_exception<rclpy::NodeNameNonExistentError>(
    m, "NodeNameNonExistentError", rclerror.ptr());
  py::register_exception<rclpy::UnsupportedEventTypeError>(
    m, "UnsupportedEventTypeError", rclerror.ptr());

  m.def(
    "rclpy_context_get_domain_id", &rclpy::context_get_domain_id,
    "Retrieves domain ID from init_options of context");
  m.def(
    "rclpy_create_context", &rclpy::create_context,
    "Create a capsule with an rcl_context_t instance");
  m.def(
    "rclpy_ok", &rclpy::context_is_valid,
    "Return true if the context is valid");

  m.def(
    "rclpy_service_info_get_sequence_number", &rclpy::service_info_get_sequence_number,
    "Retrieve sequence number from service_info");
  m.def(
    "rclpy_service_info_get_source_timestamp", &rclpy::service_info_get_source_timestamp,
    "Retrieve source timestamp from service_info");
  m.def(
    "rclpy_service_info_get_received_timestamp", &rclpy::service_info_get_received_timestamp,
    "Retrieve received timestamp from service_info");

  m.def(
    "rclpy_create_guard_condition", &rclpy::guard_condition_create,
    "Create a general purpose guard condition");
  m.def(
    "rclpy_trigger_guard_condition", &rclpy::guard_condition_trigger,
    "Trigger a general purpose guard condition");

  m.def(
    "rclpy_reset_timer", &rclpy::reset_timer,
    "Reset a timer.");
  m.def(
    "rclpy_call_timer", &rclpy::call_timer,
    "Call a timer and starts counting again.");
  m.def(
    "rclpy_change_timer_period", &rclpy::change_timer_period,
    "Set the period of a timer.");
  m.def(
    "rclpy_is_timer_ready", &rclpy::is_timer_ready,
    "Check if a timer as reached timeout.");
  m.def(
    "rclpy_cancel_timer", &rclpy::cancel_timer,
    "Cancel a timer.");
  m.def(
    "rclpy_is_timer_canceled", &rclpy::is_timer_canceled,
    "Check if a timer is canceled.");
  m.def(
    "rclpy_time_until_next_call", &rclpy::time_until_next_call,
    "Get the remaining time before timer is ready.");
  m.def(
    "rclpy_time_since_last_call", &rclpy::time_since_last_call,
    "Get the elapsed time since last timer call.");
  m.def(
    "rclpy_get_timer_period", &rclpy::get_timer_period,
    "Get the period of a timer.");
  m.def(
    "rclpy_create_timer", &rclpy::create_timer,
    "Create a Timer.");

  m.def(
    "rclpy_create_subscription", &rclpy::subscription_create,
    "Create a Subscription");
  m.def(
    "rclpy_get_subscription_topic_name", &rclpy::subscription_get_topic_name,
    "Get the topic name of a subscription");
}
