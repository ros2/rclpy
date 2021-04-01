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

#include <memory>
#include <string>
#include <utility>

#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "service.hpp"

namespace rclpy
{

void
Service::destroy()
{
  rcl_service_.reset();
  node_handle_.reset();
}

Service::Service(
  py::capsule pynode, py::object pysrv_type, std::string service_name,
  py::object pyqos_profile)
: node_handle_(std::make_shared<Handle>(pynode))
{
  auto node = node_handle_->cast<rcl_node_t *>("rcl_node_t");

  auto srv_type = static_cast<rosidl_service_type_support_t *>(
    rclpy_common_get_type_support(pysrv_type.ptr()));
  if (!srv_type) {
    throw py::error_already_set();
  }

  rcl_service_options_t service_ops = rcl_service_get_default_options();

  if (!pyqos_profile.is_none()) {
    service_ops.qos = pyqos_profile.cast<rmw_qos_profile_t>();
  }

  // Create a client
  rcl_service_ = std::shared_ptr<rcl_service_t>(
    new rcl_service_t,
    [this](rcl_service_t * service)
    {
      auto node = node_handle_->cast_or_warn<rcl_node_t *>("rcl_node_t");

      rcl_ret_t ret = rcl_service_fini(service, node);
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini client: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete service;
    });

  *rcl_service_ = rcl_get_zero_initialized_service();

  rcl_ret_t ret = rcl_service_init(
    rcl_service_.get(), node, srv_type,
    service_name.c_str(), &service_ops);
  if (RCL_RET_OK != ret) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      std::string error_text{"failed to create service due to invalid topic name '"};
      error_text += service_name;
      error_text += "': ";
      error_text += rcl_get_error_string().str;
      rcl_reset_error();
      throw py::value_error(error_text);
    }
    throw RCLError("failed to create service");
  }
}

void
Service::service_send_response(py::object pyresponse, py::capsule pyheader)
{
  destroy_ros_message_signature * destroy_ros_message = nullptr;
  void * raw_ros_response = rclpy_convert_from_py(pyresponse.ptr(), &destroy_ros_message);
  if (!raw_ros_response) {
    throw py::error_already_set();
  }
  auto message_deleter = [destroy_ros_message](void * ptr) {destroy_ros_message(ptr);};
  auto ros_response = std::unique_ptr<void, decltype(message_deleter)>(
    raw_ros_response, message_deleter);

  rmw_request_id_t * header;
  if (0 == strcmp("rmw_request_id_t", pyheader.name())) {
    header = static_cast<rmw_request_id_t *>(pyheader);
  } else if (0 == strcmp("rmw_service_info_t", pyheader.name())) {
    auto info_header = static_cast<rmw_service_info_t *>(pyheader);
    header = &(info_header->request_id);
  } else {
    throw py::value_error("capsule is not an rmw_request_id_t or rmw_service_info_t");
  }

  rcl_ret_t ret = rcl_send_response(rcl_service_.get(), header, ros_response.get());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to send response");
  }
}

py::tuple
Service::service_take_request(py::object pyrequest_type)
{
  destroy_ros_message_signature * destroy_ros_message = nullptr;
  void * taken_request_ptr = rclpy_create_from_py(pyrequest_type.ptr(), &destroy_ros_message);
  if (!taken_request_ptr) {
    throw py::error_already_set();
  }

  auto taken_request = create_from_py(pyrequest_type);

  auto deleter = [](rmw_service_info_t * ptr) {PyMem_Free(ptr);};
  auto header = std::unique_ptr<rmw_service_info_t, decltype(deleter)>(
    static_cast<rmw_service_info_t *>(PyMem_Malloc(sizeof(rmw_service_info_t))),
    deleter);
  if (!header) {
    throw std::bad_alloc();
  }

  py::tuple result_tuple(2);
  rcl_ret_t ret = rcl_take_request_with_info(
    rcl_service_.get(), header.get(), taken_request.get());
  if (ret == RCL_RET_SERVICE_TAKE_FAILED) {
    result_tuple[0] = py::none();
    result_tuple[1] = py::none();
    return result_tuple;
  } else if (RCL_RET_OK != ret) {
    throw RCLError("service failed to take request");
  }

  py::list result_list(2);
  result_list[1] = py::capsule(header.release(), "rmw_service_info_t");

  result_list[0] = convert_to_py(taken_request.get(), pyrequest_type);
  // result_list now owns the message
  taken_request.release();

  return result_list;
}
void
define_service(py::object module)
{
  py::class_<Service, Destroyable>(module, "Service")
  .def(py::init<py::capsule, py::object, std::string, py::object>())
  .def_property_readonly(
    "pointer", [](const Service & service) {
      return reinterpret_cast<size_t>(service.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "service_send_response", &Service::service_send_response,
    "Send a response")
  .def(
    "service_take_request", &Service::service_take_request,
    "Take a request from a given service");
}
}  // namespace rclpy
