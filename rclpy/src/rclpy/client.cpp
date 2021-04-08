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

#include "rclpy_common/common.h"

#include "client.hpp"
#include "python_allocator.hpp"
#include "rclpy_common/exceptions.hpp"
#include "utils.hpp"

namespace rclpy
{

void
Client::destroy()
{
  rcl_client_.reset();
  node_handle_.reset();
}

Client::Client(
  py::capsule pynode, py::object pysrv_type, const char * service_name, py::object pyqos_profile)
: node_handle_(std::make_shared<Handle>(pynode))
{
  auto node = node_handle_->cast<rcl_node_t *>("rcl_node_t");

  auto srv_type = static_cast<rosidl_service_type_support_t *>(
    rclpy_common_get_type_support(pysrv_type.ptr()));
  if (!srv_type) {
    throw py::error_already_set();
  }

  rcl_client_options_t client_ops = rcl_client_get_default_options();

  if (!pyqos_profile.is_none()) {
    client_ops.qos = pyqos_profile.cast<rmw_qos_profile_t>();
  }


  // Create a client
  rcl_client_ = std::shared_ptr<rcl_client_t>(
    PythonAllocator<rcl_client_t>().allocate(1),
    [this](rcl_client_t * client)
    {
      auto node = node_handle_->cast_or_warn<rcl_node_t *>("rcl_node_t");

      rcl_ret_t ret = rcl_client_fini(client, node);
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini client: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      PythonAllocator<rcl_client_t>().deallocate(client, 1);
    });

  *rcl_client_ = rcl_get_zero_initialized_client();

  rcl_ret_t ret = rcl_client_init(
    rcl_client_.get(), node, srv_type, service_name, &client_ops);
  if (RCL_RET_OK != ret) {
    if (RCL_RET_SERVICE_NAME_INVALID == ret) {
      std::string error_text{"failed to create client due to invalid service name '"};
      error_text += service_name;
      error_text += "': ";
      error_text += rcl_get_error_string().str;
      rcl_reset_error();
      throw py::value_error(error_text);
    }
    throw RCLError("failed to create client");
  }
}

int64_t
Client::send_request(py::object pyrequest)
{
  destroy_ros_message_signature * destroy_ros_message = nullptr;
  void * raw_ros_request = rclpy_convert_from_py(pyrequest.ptr(), &destroy_ros_message);
  if (!raw_ros_request) {
    throw py::error_already_set();
  }

  int64_t sequence_number;
  rcl_ret_t ret = rcl_send_request(rcl_client_.get(), raw_ros_request, &sequence_number);
  destroy_ros_message(raw_ros_request);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to send request");
  }

  return sequence_number;
}

bool
Client::service_server_is_available()
{
  auto node = node_handle_->cast<rcl_node_t *>("rcl_node_t");
  bool is_ready;
  rcl_ret_t ret = rcl_service_server_is_available(node, rcl_client_.get(), &is_ready);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to check service availability");
  }

  return is_ready;
}

py::tuple
Client::take_response(py::object pyresponse_type)
{
  auto taken_response = create_from_py(pyresponse_type);

  rmw_service_info_t header;

  py::tuple result_tuple(2);
  rcl_ret_t ret = rcl_take_response_with_info(
    rcl_client_.get(), &header, taken_response.get());
  if (ret == RCL_RET_CLIENT_TAKE_FAILED) {
    result_tuple[0] = py::none();
    result_tuple[1] = py::none();
    return result_tuple;
  }
  if (RCL_RET_OK != ret) {
    throw RCLError("encountered error when taking client response");
  }

  result_tuple[0] = header;

  result_tuple[1] = convert_to_py(taken_response.get(), pyresponse_type);
  // result_tuple now owns the message
  taken_response.release();

  return result_tuple;
}

void
define_client(py::object module)
{
  py::class_<Client, Destroyable>(module, "Client")
  .def(py::init<py::capsule, py::object, const char *, py::object>())
  .def_property_readonly(
    "pointer", [](const Client & client) {
      return reinterpret_cast<size_t>(client.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "send_request", &Client::send_request,
    "Send a request")
  .def(
    "service_server_is_available", &Client::service_server_is_available,
    "Return true if the service server is available")
  .def(
    "take_response", &Client::take_response,
    "Take a received response from an earlier request");
}
}  // namespace rclpy
