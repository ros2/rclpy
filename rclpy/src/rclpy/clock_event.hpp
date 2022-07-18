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

#ifndef RCLPY__CLOCK_EVENT_HPP_
#define RCLPY__CLOCK_EVENT_HPP_

#include <pybind11/pybind11.h>

#include <rcl/time.h>

#include <condition_variable>
#include <memory>
#include <mutex>

#include "clock.hpp"

namespace py = pybind11;

namespace rclpy
{
class ClockEvent
{
public:
  /// Wait until a time specified by a system or steady clock.
  /// \param clock the clock to use for time synchronization with until
  /// \param until this method will block until this time is reached.
  template<typename ClockType>
  void wait_until(std::shared_ptr<Clock> clock, rcl_time_point_t until);

  /// Wait until a time specified by a ROS clock.
  /// \warning the caller is responsible for creating a time jump callback to set this event when
  /// the target ROS time is reached.
  /// when a given ROS time is reached.
  /// \param clock the clock to use for time synchronization.
  /// \param until this method will block until this time is reached.
  void wait_until_ros(std::shared_ptr<Clock> clock, rcl_time_point_t until);

  /// Indicate if the ClockEvent is set.
  /// \return True if the ClockEvent is set.
  bool is_set();

  /// Set the event.
  void set();

  /// Clear the event.
  void clear();

private:
  bool state_ = false;
  std::mutex mutex_;
  std::condition_variable cv_;
};

/// Define a pybind11 wrapper for an rclpy::ClockEvent
void define_clock_event(py::object module);
}  // namespace rclpy

#endif  // RCLPY__CLOCK_EVENT_HPP_
