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

#include <cstddef>

#include "service_introspection.hpp"
#include "rcl/service_introspection.h"
#include "rcl/client.h"
#include "rcl/node.h"
#include "rcl/service.h"

namespace rclpy
{

void
define_service_introspection(py::module_ module)
{
  py::module_ m2 = module.def_submodule(
    "service_introspection",
    "utilities for introspecting services");
  m2.def(
    "configure_service_events",
    [](size_t srv, size_t node, bool opt) {
      return rcl_service_introspection_configure_server_service_events(
        reinterpret_cast<
          rcl_service_t *>(srv), reinterpret_cast<rcl_node_t *>(node), opt);
    });
  m2.def(
    "configure_client_events",
    [](size_t clt, size_t node, bool opt) {
      return rcl_service_introspection_configure_client_service_events(
        reinterpret_cast<rcl_client_t
        *>(clt), reinterpret_cast<rcl_node_t *>(node), opt);
    });
  m2.def(
    "configure_service_message_payload", [](size_t srv, bool opt) {
      return rcl_service_introspection_configure_server_service_event_message_payload(
        reinterpret_cast<rcl_service_t *>(srv), opt);
    });
  m2.def(
    "configure_client_message_payload", [](size_t clt, bool opt) {
      return rcl_service_introspection_configure_client_service_event_message_payload(
        reinterpret_cast<rcl_client_t *>(clt), opt);
    });
  m2.attr("RCL_SERVICE_INTROSPECTION_PUBLISH_CLIENT_PARAMETER") = "publish_client_events";
  m2.attr("RCL_SERVICE_INTROSPECTION_PUBLISH_SERVICE_PARAMETER") = "publish_service_events";
  m2.attr("RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX") = RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX;
}
}  // namespace rclpy
