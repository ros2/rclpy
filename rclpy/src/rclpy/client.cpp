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

extern "C"
{
#include "rclpy_common/common.h"
}
#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "client.hpp"

namespace rclpy
{
static void
_rclpy_destroy_client(void * p)
{
  auto cli = static_cast<rclpy_client_t *>(p);
  if (!cli) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_client got NULL pointer");
    return;
  }

  rcl_ret_t ret = rcl_client_fini(&(cli->client), cli->node);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini client: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
  }
  PyMem_Free(cli);
}

py::capsule
client_create(
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

  rcl_client_options_t client_ops = rcl_client_get_default_options();

  if (!pyqos_profile.is_none()) {
    if (0 != strcmp("rmw_qos_profile_t", pyqos_profile.name())) {
      throw py::value_error("capsule is not an rmw_qos_profile_t");
    }
    auto qos_profile = static_cast<rmw_qos_profile_t *>(pyqos_profile);
    client_ops.qos = *qos_profile;
  }

  // Use smart pointer to make sure memory is free'd on error
  auto deleter = [](rclpy_client_t * ptr) {_rclpy_destroy_client(ptr);};
  auto cli = std::unique_ptr<rclpy_client_t, decltype(deleter)>(
    static_cast<rclpy_client_t *>(PyMem_Malloc(sizeof(rclpy_client_t))),
    deleter);
  if (!cli) {
    throw std::bad_alloc();
  }
  cli->client = rcl_get_zero_initialized_client();
  cli->node = node;

  rcl_ret_t ret = rcl_client_init(
    &(cli->client), node, srv_type,
    service_name.c_str(), &client_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      std::string error_text{"failed to create client due to invalid service name '"};
      error_text += service_name;
      error_text += "': ";
      error_text += rcl_get_error_string().str;
      rcl_reset_error();
      throw py::value_error(error_text);
    }
    throw RCLError("failed to create client");
  }

  PyObject * pycli_c =
    rclpy_create_handle_capsule(cli.get(), "rclpy_client_t", _rclpy_destroy_client);
  if (!pycli_c) {
    throw py::error_already_set();
  }
  auto pycli = py::reinterpret_steal<py::capsule>(pycli_c);
  // pycli now owns the rclpy_client_t
  cli.release();

  auto cli_handle = static_cast<rclpy_handle_t *>(pycli);
  auto node_handle = static_cast<rclpy_handle_t *>(pynode);
  _rclpy_handle_add_dependency(cli_handle, node_handle);
  if (PyErr_Occurred()) {
    _rclpy_handle_dec_ref(cli_handle);
    throw py::error_already_set();
  }

  return pycli;
}

uint64_t
client_send_request(py::capsule pyclient, py::object pyrequest)
{
  auto client = static_cast<rclpy_client_t *>(
    rclpy_handle_get_pointer_from_capsule(pyclient.ptr(), "rclpy_client_t"));
  if (!client) {
    throw py::error_already_set();
  }

  destroy_ros_message_signature * destroy_ros_message = NULL;
  void * raw_ros_request = rclpy_convert_from_py(pyrequest.ptr(), &destroy_ros_message);
  if (!raw_ros_request) {
    throw py::error_already_set();
  }

  int64_t sequence_number;
  rcl_ret_t ret = rcl_send_request(&(client->client), raw_ros_request, &sequence_number);
  destroy_ros_message(raw_ros_request);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to send request");
  }

  return sequence_number;
}

bool
client_service_server_is_available(py::capsule pyclient)
{
  auto client = static_cast<rclpy_client_t *>(
    rclpy_handle_get_pointer_from_capsule(pyclient.ptr(), "rclpy_client_t"));
  if (!client) {
    throw py::error_already_set();
  }

  bool is_ready;
  rcl_ret_t ret = rcl_service_server_is_available(client->node, &(client->client), &is_ready);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to check service availability");
  }

  return is_ready;
}

py::tuple
client_take_response(py::capsule pyclient, py::object pyresponse_type)
{
  auto client = static_cast<rclpy_client_t *>(
    rclpy_handle_get_pointer_from_capsule(pyclient.ptr(), "rclpy_client_t"));
  if (!client) {
    throw py::error_already_set();
  }

  destroy_ros_message_signature * destroy_ros_message = NULL;
  void * taken_response_ptr = rclpy_create_from_py(pyresponse_type.ptr(), &destroy_ros_message);
  if (!taken_response_ptr) {
    throw py::error_already_set();
  }
  auto message_deleter = [destroy_ros_message](void * ptr) {destroy_ros_message(ptr);};
  auto taken_response = std::unique_ptr<void, decltype(message_deleter)>(
    taken_response_ptr, message_deleter);

  auto deleter = [](rmw_service_info_t * ptr) {PyMem_Free(ptr);};
  auto header = std::unique_ptr<rmw_service_info_t, decltype(deleter)>(
    static_cast<rmw_service_info_t *>(PyMem_Malloc(sizeof(rmw_service_info_t))),
    deleter);
  if (!header) {
    throw std::bad_alloc();
  }

  rcl_ret_t ret = rcl_take_response_with_info(
    &(client->client), header.get(), taken_response.get());
  if (ret == RCL_RET_CLIENT_TAKE_FAILED) {
    return py::tuple(2);
  }

  PyObject * pytaken_response = rclpy_convert_to_py(taken_response.get(), pyresponse_type.ptr());
  if (!pytaken_response) {
    throw py::error_already_set();
  }
  // pytaken_response now owns the message
  taken_response.release();

  py::tuple result_tuple(2);
  result_tuple[0] = py::capsule(header.release(), "rmw_service_info_t");
  result_tuple[1] = py::reinterpret_steal<py::object>(pytaken_response);
  return result_tuple;
}
}  // namespace rclpy
