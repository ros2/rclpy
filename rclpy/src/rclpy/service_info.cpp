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

#include <pybind11/pybind11.h>

#include <rmw/types.h>

#include "service_info.hpp"

namespace rclpy
{

void
define_service_info(py::object module)
{
  py::class_<rmw_service_info_t>(module, "rmw_service_info_t")
  .def_readonly("source_timestamp", &rmw_service_info_t::source_timestamp)
  .def_readonly("received_timestamp", &rmw_service_info_t::received_timestamp)
  .def_readonly("request_id", &rmw_service_info_t::request_id);

  py::class_<rmw_request_id_t>(module, "rmw_request_id_t")
  .def_readonly("sequence_number", &rmw_request_id_t::sequence_number);
  // "writer_guid" is not included because it's not used by rclpy
}
}  // namespace rclpy
