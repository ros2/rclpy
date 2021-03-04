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

#include <rmw/types.h>

#include "service_info.hpp"

namespace rclpy
{
int64_t
service_info_get_sequence_number(py::capsule pyservice_info)
{
  if (0 != strcmp("rmw_service_info_t", pyservice_info.name())) {
    throw py::value_error("capsule is not an rmw_service_info_t");
  }
  auto service_info = static_cast<rmw_service_info_t *>(pyservice_info);
  return service_info->request_id.sequence_number;
}

int64_t
service_info_get_source_timestamp(py::capsule pyservice_info)
{
  if (0 != strcmp("rmw_service_info_t", pyservice_info.name())) {
    throw py::value_error("capsule is not an rmw_service_info_t");
  }
  auto service_info = static_cast<rmw_service_info_t *>(pyservice_info);
  return service_info->source_timestamp;
}

int64_t
service_info_get_received_timestamp(py::capsule pyservice_info)
{
  if (0 != strcmp("rmw_service_info_t", pyservice_info.name())) {
    throw py::value_error("capsule is not an rmw_service_info_t");
  }
  auto service_info = static_cast<rmw_service_info_t *>(pyservice_info);
  return service_info->received_timestamp;
}
}  // namespace rclpy
