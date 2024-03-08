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

#include <rmw/types.h>
#include <rmw/subscription_options.h>

#include "subscription_options.hpp"

using pybind11::literals::operator""_a;

namespace rclpy
{
void
define_subscription_options(py::object module)
{
  py::class_<rmw_subscription_options_t>(module, "rmw_subscription_options_t")
  .def(py::init<>(&rmw_get_default_subscription_options))
  .def_readwrite(
    "ignore_local_publications",
    &rmw_subscription_options_t::ignore_local_publications);
}
}  // namespace rclpy
