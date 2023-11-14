// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "type_description_service.hpp"
#include "utils.hpp"

namespace rclpy
{

TypeDescriptionService::TypeDescriptionService(Node & node)
{
  auto srv_ptr = std::make_shared<rcl_service_t>();
  rcl_ret_t ret = rcl_node_type_description_service_init(srv_ptr.get(), node.rcl_ptr());
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to initialize type description service");
  }
  service_ = std::make_shared<Service>(node, srv_ptr);
}

Service TypeDescriptionService::get_impl()
{
  return *service_;
}

py::object TypeDescriptionService::handle_request(
  py::object pyrequest,
  py::object pyresponse_type,
  Node & node)
{
  // Header not used by handler, just needed as part of signature.
  rmw_request_id_t header;
  type_description_interfaces__srv__GetTypeDescription_Response response;
  auto request = convert_from_py(pyrequest);
  rcl_node_type_description_service_handle_request(
    node.rcl_ptr(),
    &header,
    static_cast<type_description_interfaces__srv__GetTypeDescription_Request *>(request.get()),
    &response);
  return convert_to_py(&response, pyresponse_type);
}

void TypeDescriptionService::destroy()
{
  service_.reset();
}

void
define_type_description_service(py::object module)
{
  py::class_<TypeDescriptionService, Destroyable, std::shared_ptr<TypeDescriptionService>
  >(module, "TypeDescriptionService")
  .def(py::init<Node &>())
  .def_property_readonly(
    "impl", &TypeDescriptionService::get_impl, "Get the rcl service wrapper capsule.")
  .def(
    "handle_request", &TypeDescriptionService::handle_request,
    "Handle an incoming request by calling RCL implementation");
}
}  // namespace rclpy
