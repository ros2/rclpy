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

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/types.h>

#include <memory>
#include <string>

#include "rclpy_common/common.h"

#include "rclpy_common/exceptions.hpp"

#include "serialization.hpp"
#include "subscription.hpp"
#include "utils.hpp"

using pybind11::literals::operator""_a;

namespace rclpy
{
Subscription::Subscription(
  Node & node, py::object pymsg_type, std::string topic,
  py::object pyqos_profile)
: node_(node)
{
  auto msg_type = static_cast<rosidl_message_type_support_t *>(
    rclpy_common_get_type_support(pymsg_type.ptr()));
  if (!msg_type) {
    throw py::error_already_set();
  }

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();

  if (!pyqos_profile.is_none()) {
    subscription_ops.qos = pyqos_profile.cast<rmw_qos_profile_t>();
  }

  rcl_subscription_ = std::shared_ptr<rcl_subscription_t>(
    new rcl_subscription_t,
    [node](rcl_subscription_t * subscription)
    {
      // Intentionally capture node by copy so shared_ptr can be transfered to copies
      rcl_ret_t ret = rcl_subscription_fini(subscription, node.rcl_ptr());
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini subscription: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete subscription;
    });

  *rcl_subscription_ = rcl_get_zero_initialized_subscription();

  rcl_ret_t ret = rcl_subscription_init(
    rcl_subscription_.get(), node_.rcl_ptr(), msg_type,
    topic.c_str(), &subscription_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      std::string error_text{"Failed to create subscription due to invalid topic name '"};
      error_text += topic;
      error_text += "'";
      throw py::value_error(error_text);
    }
    throw RCLError("Failed to create subscription");
  }
}

void Subscription::destroy()
{
  rcl_subscription_.reset();
  node_.destroy();
}

py::object
Subscription::take_message(py::object pymsg_type, bool raw)
{
  py::object pytaken_msg;
  rmw_message_info_t message_info;
  if (raw) {
    SerializedMessage taken{rcutils_get_default_allocator()};
    rcl_ret_t ret = rcl_take_serialized_message(
      rcl_subscription_.get(), &taken.rcl_msg, &message_info, NULL);
    if (RCL_RET_OK != ret) {
      if (RCL_RET_BAD_ALLOC == ret) {
        rcl_reset_error();
        throw std::bad_alloc();
      }
      if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
        return py::none();
      }
      throw RCLError("failed to take raw message from subscription");
    }
    pytaken_msg = py::bytes(
      reinterpret_cast<const char *>(taken.rcl_msg.buffer),
      taken.rcl_msg.buffer_length);
  } else {
    auto taken_msg = create_from_py(pymsg_type);

    rcl_ret_t ret = rcl_take(
      rcl_subscription_.get(), taken_msg.get(), &message_info, NULL);
    if (RCL_RET_OK != ret) {
      if (RCL_RET_BAD_ALLOC == ret) {
        rcl_reset_error();
        throw std::bad_alloc();
      }
      if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
        return py::none();
      }
      throw RCLError("failed to take message from subscription");
    }

    pytaken_msg = convert_to_py(taken_msg.get(), pymsg_type);
  }

  return py::make_tuple(
    pytaken_msg, py::dict(
      "source_timestamp"_a = message_info.source_timestamp,
      "received_timestamp"_a = message_info.received_timestamp));
}

const char *
Subscription::get_logger_name()
{
  const char * node_logger_name = rcl_node_get_logger_name(node_.rcl_ptr());
  if (!node_logger_name) {
    throw RCLError("Node logger name not set");
  }

  return node_logger_name;
}

std::string
Subscription::get_topic_name()
{
  const char * subscription_name = rcl_subscription_get_topic_name(rcl_subscription_.get());
  if (nullptr == subscription_name) {
    throw RCLError("failed to get subscription topic name");
  }

  return std::string(subscription_name);
}
void
define_subscription(py::object module)
{
  py::class_<Subscription, Destroyable, std::shared_ptr<Subscription>>(module, "Subscription")
  .def(py::init<Node &, py::object, std::string, py::object>())
  .def_property_readonly(
    "pointer", [](const Subscription & subscription) {
      return reinterpret_cast<size_t>(subscription.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "take_message", &Subscription::take_message,
    "Take a message and its metadata from a subscription")
  .def(
    "get_logger_name", &Subscription::get_logger_name,
    "Get the name of the logger associated with the node of the subscription.")
  .def(
    "get_topic_name", &Subscription::get_topic_name,
    "Return the resolved topic name of a subscription.");
}
}  // namespace rclpy
