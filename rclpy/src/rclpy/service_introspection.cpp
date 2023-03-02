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
#include "rcl/service_introspection.h"

namespace rclpy
{

void
define_service_introspection(py::module module)
{
  py::module m2 = module.def_submodule(
    "service_introspection",
    "utilities for introspecting services");

  py::enum_<rcl_service_introspection_state_t>(m2, "ServiceIntrospectionState")
  .value("OFF", RCL_SERVICE_INTROSPECTION_OFF)
  .value("METADATA", RCL_SERVICE_INTROSPECTION_METADATA)
  .value("CONTENTS", RCL_SERVICE_INTROSPECTION_CONTENTS);
}
}  // namespace rclpy
