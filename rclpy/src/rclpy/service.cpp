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

extern "C"
{
#include "rclpy_common/common.h"
}
#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "service.hpp"

namespace rclpy
{
static void
_rclpy_destroy_service(void * p)
{
  auto srv = static_cast<rclpy_service_t *>(p);
  if (!srv) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_service got NULL pointer");
    return;
  }

  rcl_ret_t ret = rcl_service_fini(&(srv->service), srv->node);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini service: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
  }
  PyMem_Free(srv);
}

py::capsule
service_create(
  py::capsule pynode, py::object pysrv_type, std::string service_name,
  py::capsule pyqos_profile)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  auto srv_type = static_cast<rosidl_service_type_support_t *>(
    rclpy_common_get_type_support(pysrv_type.ptr()));
  if (!srv_type) {
    throw py::error_already_set();
  }

  rcl_service_options_t service_ops = rcl_service_get_default_options();

  if (!pyqos_profile.is_none()) {
    if (0 != strcmp("rmw_qos_profile_t", pyqos_profile.name())) {
      throw py::value_error("capsule is not an rmw_qos_profile_t");
    }
    auto qos_profile = static_cast<rmw_qos_profile_t *>(pyqos_profile);
    service_ops.qos = *qos_profile;
  }

  // Use smart pointer to make sure memory is free'd on error
  auto deleter = [](rclpy_service_t * ptr) {_rclpy_destroy_service(ptr);};
  auto srv = std::unique_ptr<rclpy_service_t, decltype(deleter)>(
    static_cast<rclpy_service_t *>(PyMem_Malloc(sizeof(rclpy_service_t))),
    deleter);
  if (!srv) {
    throw std::bad_alloc();
  }
  srv->service = rcl_get_zero_initialized_service();
  srv->node = node;

  rcl_ret_t ret = rcl_service_init(
    &(srv->service), node, srv_type,
    service_name.c_str(), &service_ops);
  if (ret != RCL_RET_OK) {
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

  PyObject * pysrv_c =
    rclpy_create_handle_capsule(srv.get(), "rclpy_service_t", _rclpy_destroy_service);
  if (!pysrv_c) {
    throw py::error_already_set();
  }
  auto pysrv = py::reinterpret_steal<py::capsule>(pysrv_c);
  // pysrv now owns the rclpy_service_t
  srv.release();

  auto srv_handle = static_cast<rclpy_handle_t *>(pysrv);
  auto node_handle = static_cast<rclpy_handle_t *>(pynode);
  _rclpy_handle_add_dependency(srv_handle, node_handle);
  if (PyErr_Occurred()) {
    throw py::error_already_set();
  }

  return pysrv;
}

void
service_send_response(py::capsule pyservice, py::object pyresponse, py::capsule pyheader)
{
  auto srv = static_cast<rclpy_service_t *>(
    rclpy_handle_get_pointer_from_capsule(pyservice.ptr(), "rclpy_service_t"));
  if (!srv) {
    throw py::error_already_set();
  }

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

  rcl_ret_t ret = rcl_send_response(&(srv->service), header, ros_response.get());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to send response");
  }
}

py::object
service_take_request(py::capsule pyservice, py::object pyrequest_type)
{
  auto srv = static_cast<rclpy_service_t *>(
    rclpy_handle_get_pointer_from_capsule(pyservice.ptr(), "rclpy_service_t"));
  if (!srv) {
    throw py::error_already_set();
  }

  destroy_ros_message_signature * destroy_ros_message = nullptr;
  void * taken_request_ptr = rclpy_create_from_py(pyrequest_type.ptr(), &destroy_ros_message);
  if (!taken_request_ptr) {
    throw py::error_already_set();
  }
  auto message_deleter = [destroy_ros_message](void * ptr) {destroy_ros_message(ptr);};
  auto taken_request = std::unique_ptr<void, decltype(message_deleter)>(
    taken_request_ptr, message_deleter);

  auto deleter = [](rmw_service_info_t * ptr) {PyMem_Free(ptr);};
  auto header = std::unique_ptr<rmw_service_info_t, decltype(deleter)>(
    static_cast<rmw_service_info_t *>(PyMem_Malloc(sizeof(rmw_service_info_t))),
    deleter);
  if (!header) {
    throw std::bad_alloc();
  }

  rcl_ret_t ret = rcl_take_request_with_info(
    &(srv->service), header.get(), taken_request.get());
  if (ret == RCL_RET_SERVICE_TAKE_FAILED) {
    return py::none();
  } else if (ret != RCL_RET_OK) {
    throw RCLError("service failed to take request");
  }

  py::list result_list(2);
  result_list[1] = py::capsule(header.release(), "rmw_service_info_t");

  PyObject * pytaken_request_c = rclpy_convert_to_py(taken_request.get(), pyrequest_type.ptr());
  if (!pytaken_request_c) {
    throw py::error_already_set();
  }
  result_list[0] = py::reinterpret_steal<py::object>(pytaken_request_c);
  // result_list now owns the message
  taken_request.release();

  return std::move(result_list);
}
}  // namespace rclpy
