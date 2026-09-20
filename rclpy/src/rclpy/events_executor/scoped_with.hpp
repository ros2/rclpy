// Copyright 2024-2025 Brad Martin
// Copyright 2024 Merlin Labs, Inc.
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

#ifndef RCLPY__EVENTS_EXECUTOR__SCOPED_WITH_HPP_
#define RCLPY__EVENTS_EXECUTOR__SCOPED_WITH_HPP_

#include <pybind11/pybind11.h>

namespace rclpy
{
namespace events_executor
{

/// Enters a python context manager for the scope of this object instance.
class ScopedWith
{
public:
  explicit ScopedWith(pybind11::handle object)
  : object_(pybind11::cast<pybind11::object>(object))
  {
    object_.attr("__enter__")();
  }

  ~ScopedWith() {object_.attr("__exit__")(pybind11::none(), pybind11::none(), pybind11::none());}

private:
  pybind11::object object_;
};

}  // namespace events_executor
}  // namespace rclpy

#endif  // RCLPY__EVENTS_EXECUTOR__SCOPED_WITH_HPP_
