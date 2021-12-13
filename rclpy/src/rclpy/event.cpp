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

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rcl/types.h>

#include <condition_variable>
#include <cstring>
#include <memory>
#include <stdexcept>

#include "event.hpp"

namespace py = pybind11;

namespace rclpy
{
template<typename ClockType>
void Event::wait_until(std::shared_ptr<Clock> clock, rcl_time_point_t until)
{
  // Synchronize because clock epochs might differ
  const rcl_time_point_t rcl_entry = clock->get_now();
  const typename ClockType::time_point chrono_entry = ClockType::now();

  rcl_duration_t delta_t;
  rcl_ret_t ret = rcl_difference_times(&rcl_entry, &until, &delta_t);

  if (RCL_RET_OK != ret) {
    throw RCLError("failed to subtract times");
  }

  const typename ClockType::time_point chrono_until =
    chrono_entry + std::chrono::nanoseconds(delta_t.nanoseconds);

  // Could be a long wait, release the gil
  py::gil_scoped_release release;
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait_until(lock, chrono_until, [this]() {return state_;});
}

void Event::wait_until_ros(std::shared_ptr<Clock> clock, rcl_time_point_t until)
{
  // Check if ROS time is enabled in C++ to avoid TOCTTOU with TimeSource by holding GIL
  if (clock->get_ros_time_override_is_enabled()) {
    // Could be a long wait, release the gil
    py::gil_scoped_release release;
    std::unique_lock<std::mutex> lock(mutex_);
    // Caller must have setup a time jump callback to wake this event
    cv_.wait(lock, [this]() {return state_;});
  } else {
    // ROS time not enabled is system time
    wait_until<std::chrono::system_clock>(clock, until);
  }
}

bool Event::is_set()
{
  std::unique_lock<std::mutex> lock(mutex_);
  return state_;
}

void Event::set()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    state_ = true;
  }
  cv_.notify_all();
}

void Event::clear()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    state_ = false;
  }
  cv_.notify_all();
}

void define_event(py::object module)
{
  py::class_<Event>(module, "Event")
  .def(py::init())
  .def(
    "wait_until_steady", &Event::wait_until<std::chrono::steady_clock>,
    "Wait for the event to be set (monotonic wait)")
  .def(
    "wait_until_system", &Event::wait_until<std::chrono::system_clock>,
    "Wait for the event to be set (system timed wait)")
  .def(
    "wait_until_ros", &Event::wait_until_ros,
    "Wait for the event to be set (ROS timed wait)")
  .def(
    "is_set", &Event::is_set,
    "Return True if the event is set, False otherwise.")
  .def(
    "set", &Event::set,
    "Set the event, waking all those who wait on it.")
  .def(
    "clear", &Event::clear,
    "Unset the event.");
}
}  // namespace rclpy
