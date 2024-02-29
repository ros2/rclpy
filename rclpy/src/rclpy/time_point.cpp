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

#include <rcl/time.h>

#include "time_point.hpp"

namespace rclpy
{
/// Create a time point
rcl_time_point_t
create_time_point(int64_t nanoseconds, rcl_clock_type_t clock_type)
{
  rcl_time_point_t time_point;
  time_point.nanoseconds = nanoseconds;
  time_point.clock_type = clock_type;
  return time_point;
}

void
define_time_point(py::object module)
{
  py::class_<rcl_time_point_t>(module, "rcl_time_point_t")
  .def(py::init<>(&create_time_point))
  .def_readonly("nanoseconds", &rcl_time_point_t::nanoseconds)
  .def_readonly("clock_type", &rcl_time_point_t::clock_type);
}
}  // namespace rclpy
