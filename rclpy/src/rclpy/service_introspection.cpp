// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include "service_introspection.hpp"
#include "rcl/introspection.h"

namespace rclpy {

void
define_service_introspection(py::module_ module)
{
  py::module_ m2 = module.def_submodule("service_introspection",
      "utilities for introspecting services");

  m2.def("configure_service_events", &rcl_service_introspection_configure_service_events);
  m2.def("configure_client_events", &rcl_service_introspection_configure_client_events);
  m2.def("configure_service_message_payload", &rcl_service_introspection_configure_service_content);
  m2.def("configure_client_message_payload", &rcl_service_introspection_configure_client_content);
  m2.attr("RCL_SERVICE_INTROSPECTION_PUBLISH_CLIENT_PARAMETER") =
    RCL_SERVICE_INTROSPECTION_PUBLISH_CLIENT_PARAMETER;
  m2.attr("RCL_SERVICE_INTROSPECTION_PUBLISH_SERVICE_PARAMETER") =
    RCL_SERVICE_INTROSPECTION_PUBLISH_SERVICE_PARAMETER;
  m2.attr("RCL_SERVICE_INTROSPECTION_PUBLISH_CLIENT_EVENT_CONTENT_PARAMETER") =
    RCL_SERVICE_INTROSPECTION_PUBLISH_CLIENT_EVENT_CONTENT_PARAMETER;
  m2.attr("RCL_SERVICE_INTROSPECTION_PUBLISH_SERVICE_EVENT_CONTENT_PARAMETER") =
    RCL_SERVICE_INTROSPECTION_PUBLISH_SERVICE_EVENT_CONTENT_PARAMETER;

} 

} // namespace rclpy
