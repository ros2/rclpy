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

#include "duration.hpp"

namespace rclpy
{
rcl_duration_t
create_duration(int64_t nanoseconds)
{
  rcl_duration_t duration;
  duration.nanoseconds = nanoseconds;
  return duration;
}
void
define_duration(py::object module)
{
  py::class_<rcl_duration_t>(module, "rcl_duration_t")
  .def(py::init<>(&create_duration))
  .def_readonly("nanoseconds", &rcl_duration_t::nanoseconds);
}
}  // namespace rclpy
