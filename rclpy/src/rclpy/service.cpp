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

#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <rcl/error_handling.h>
#include <rcl/service.h>
#include <rcl/service_introspection.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <rmw/types.h>

#include <memory>
#include <string>
#include <utility>

#include "clock.hpp"
#include "exceptions.hpp"
#include "node.hpp"
#include "service.hpp"
#include "utils.hpp"
#include "events_executor/rcl_support.hpp"

using nb::literals::operator""_a;

namespace rclpy
{
using events_executor::RclEventCallbackTrampoline;

void
Service::destroy()
{
  try {
    clear_on_new_request_callback();
  } catch (const rclpy::RCLError &) {
  }
  rcl_service_.reset();
  node_.destroy();
}

Service::Service(
  Node & node, nb::object pysrv_type, const std::string & service_name,
  std::optional<rmw_qos_profile_t> pyqos_profile)
: node_(node)
{
  srv_type_ = static_cast<rosidl_service_type_support_t *>(
    common_get_type_support(pysrv_type));
  if (nullptr == srv_type_) {
    throw nb::python_error();
  }

  rcl_service_options_t service_ops = rcl_service_get_default_options();

  if (pyqos_profile) {
    service_ops.qos = *pyqos_profile;
  }

  // Create a service
  rcl_service_ = std::shared_ptr<rcl_service_t>(
    new rcl_service_t,
    [node](rcl_service_t * service)
    {
      // Intentionally capture node by copy so shared_ptr can be transferred to copies
      rcl_ret_t ret = rcl_service_fini(service, node.rcl_ptr());
      if (RCL_RET_OK != ret) {
        warn_fini_failure("service");
      }
      delete service;
    });

  *rcl_service_ = rcl_get_zero_initialized_service();

  rcl_ret_t ret = rcl_service_init(
    rcl_service_.get(), node_.rcl_ptr(), srv_type_,
    service_name.c_str(), &service_ops);
  if (RCL_RET_OK != ret) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      std::string error_text{"failed to create service due to invalid topic name '"};
      error_text += service_name;
      error_text += "': ";
      error_text += rcl_get_error_string().str;
      rcl_reset_error();
      throw nb::value_error(error_text.c_str());
    }
    throw RCLError("failed to create service");
  }
}

Service::Service(
  Node & node, std::shared_ptr<rcl_service_t> rcl_service)
: node_(node), rcl_service_(rcl_service)
{}

void
Service::service_send_response(nb::object pyresponse, rmw_request_id_t * header)
{
  auto raw_ros_response = convert_from_py(pyresponse);
  if (!raw_ros_response) {
    throw nb::python_error();
  }

  rcl_ret_t ret = rcl_send_response(rcl_service_.get(), header, raw_ros_response.get());
  if (RCL_RET_OK != ret) {
    if (RCL_RET_TIMEOUT == ret) {
      // Warning should use line number of the current stack frame
      int stack_level = 1;
      PyErr_WarnFormat(
        PyExc_RuntimeWarning, stack_level, "failed to send response (timeout): %s",
        rcl_get_error_string().str);
      rcl_reset_error();
    } else {
      throw RCLError("failed to send response");
    }
  }
}

nb::tuple
Service::service_take_request(nb::object pyrequest_type)
{
  auto taken_request = create_from_py(pyrequest_type);
  rmw_service_info_t header;

  rcl_ret_t ret = rcl_take_request_with_info(rcl_service_.get(), &header, taken_request.get());
  if (ret == RCL_RET_SERVICE_TAKE_FAILED) {
    return nb::make_tuple(nb::none(), nb::none());
  } else if (RCL_RET_OK != ret) {
    throw RCLError("service failed to take request");
  }

  return nb::make_tuple(convert_to_py(taken_request.get(), pyrequest_type), header);
}

const char *
Service::get_service_name()
{
  return rcl_service_get_service_name(rcl_service_.get());
}

nb::dict
Service::get_qos_profile()
{
  const auto * options = rcl_service_get_options(rcl_service_.get());
  return rclpy::convert_to_qos_dict(&options->qos);
}

const char *
Service::get_logger_name() const
{
  const char * node_logger_name = rcl_node_get_logger_name(node_.rcl_ptr());
  if (!node_logger_name) {
    throw RCLError("Node logger name not set");
  }

  return node_logger_name;
}

void
Service::configure_introspection(
  Clock & clock, std::optional<rmw_qos_profile_t> pyqos_service_event_pub,
  rcl_service_introspection_state_t introspection_state)
{
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  if (pyqos_service_event_pub) {
    pub_opts.qos = *pyqos_service_event_pub;
  }

  rcl_ret_t ret = rcl_service_configure_service_introspection(
    rcl_service_.get(), node_.rcl_ptr(), clock.rcl_ptr(), srv_type_, pub_opts, introspection_state);

  if (RCL_RET_OK != ret) {
    throw RCLError("failed to configure service introspection");
  }
}

void
Service::set_callback(
  rcl_event_callback_t callback,
  const void * user_data)
{
  rcl_ret_t ret = rcl_service_set_on_new_request_callback(
    rcl_service_.get(),
    callback,
    user_data);

  if (RCL_RET_OK != ret) {
    throw RCLError(std::string("Failed to set the on new request callback for service: ") +
      rcl_get_error_string().str);
  }
}

void
Service::set_on_new_request_callback(std::function<void(size_t)> callback)
{
  clear_on_new_request_callback();
  on_new_request_callback_ = std::move(callback);
  set_callback(
    RclEventCallbackTrampoline,
    static_cast<const void *>(&on_new_request_callback_));
}

void
Service::clear_on_new_request_callback()
{
  if (on_new_request_callback_) {
    set_callback(nullptr, nullptr);
    on_new_request_callback_ = nullptr;
  }
}

void
define_service(nb::object module)
{
  nb::class_<Service, Destroyable>(
    module, "Service", nb::is_generic(),
    nb::sig("class Service(Destroyable, typing.Generic[SrvRequestT, SrvResponseT])"))
  .def(
    nb::init<Node &, nb::object, const std::string &, std::optional<rmw_qos_profile_t>>(),
    nb::sig(
      "def __init__(self, node: Node, pysrv_type: type[Srv[SrvRequestT, SrvResponseT]], "
      "name: str, pyqos_profile: rmw_qos_profile_t | None, /) -> None"))
  .def_prop_ro(
    "pointer", [](const Service & service) {
      return reinterpret_cast<size_t>(service.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def_prop_ro(
    "name", &Service::get_service_name,
    "Get the name of the service")
  .def_prop_ro(
    "qos", &Service::get_qos_profile,
    "Get the qos profile of the service")
  .def(
    "service_send_response", &Service::service_send_response,
    "Send a response",
    nb::sig(
      "def service_send_response(self, pyresponse: SrvResponseT, "
      "header: rmw_request_id_t, /) -> None"))
  .def(
    "service_take_request", &Service::service_take_request,
    "Take a request from a given service",
    nb::sig(
      "def service_take_request(self, pyrequest_type: type[SrvRequestT], /)"
      " -> tuple[SrvRequestT, rmw_service_info_t] | tuple[None, None]"))
  .def(
    "configure_introspection", &Service::configure_introspection,
    "Configure whether introspection is enabled")
  .def(
    "get_logger_name", &Service::get_logger_name,
    "Get the name of the logger associated with the node of the service.")
  .def(
    "set_on_new_request_callback", &Service::set_on_new_request_callback,
    "callback"_a)
  .def("clear_on_new_request_callback", &Service::clear_on_new_request_callback);
}
}  // namespace rclpy
