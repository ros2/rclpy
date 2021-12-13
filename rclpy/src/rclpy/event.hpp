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

#ifndef RCLPY__EVENT_HPP_
#define RCLPY__EVENT_HPP_

#include <pybind11/pybind11.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rcl/types.h>

#include <condition_variable>
#include <cstring>
#include <memory>
#include <stdexcept>

#include "clock.hpp"

namespace py = pybind11;

namespace rclpy
{
class Event
{
public:
  template<typename ClockType>
  bool wait_until(std::shared_ptr<Clock> clock, rcl_time_point_t until);

  bool wait_until_ros(std::shared_ptr<Clock> clock, rcl_time_point_t until);

  bool is_set();

  void set();

  void clear();

private:
  bool state_ = false;
  std::mutex mutex_;
  std::condition_variable cv_;
};

/// Define a pybind11 wrapper for an rclpy::Event
void define_event(py::object module);
}  // namespace rclpy

#endif  // RCLPY__EVENT_HPP_
