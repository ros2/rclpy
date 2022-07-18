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

#include <rcl/allocator.h>
#include <rcl/error_handling.h>
#include <rcl/time.h>
#include <rcl/types.h>

#include <memory>

#include "clock.hpp"

using pybind11::literals::operator""_a;

namespace rclpy
{
Clock::Clock(rcl_clock_type_t clock_type)
{
  // Create a client
  rcl_clock_ = std::shared_ptr<rcl_clock_t>(
    new rcl_clock_t,
    [](rcl_clock_t * clock)
    {
      rcl_ret_t ret = rcl_clock_fini(clock);
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini client: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete clock;
    });

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_clock_init(clock_type, rcl_clock_.get(), &allocator);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to initialize clock");
  }
}

void
Clock::destroy()
{
  rcl_clock_.reset();
}

rcl_time_point_t Clock::get_now()
{
  rcl_time_point_t time_point;
  time_point.clock_type = rcl_clock_->type;
  rcl_ret_t ret = rcl_clock_get_now(rcl_clock_.get(), &time_point.nanoseconds);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get current clock value");
  }

  return time_point;
}

bool Clock::get_ros_time_override_is_enabled()
{
  bool is_enabled;
  rcl_ret_t ret = rcl_is_enabled_ros_time_override(rcl_clock_.get(), &is_enabled);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get ROS time override status");
  }

  return is_enabled;
}

void Clock::set_ros_time_override_is_enabled(bool enabled)
{
  rcl_ret_t ret;
  if (enabled) {
    ret = rcl_enable_ros_time_override(rcl_clock_.get());
  } else {
    ret = rcl_disable_ros_time_override(rcl_clock_.get());
  }
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to set ROS time override");
  }
  if (PyErr_Occurred()) {
    // Time jump callbacks raised
    throw py::error_already_set();
  }
}

void Clock::set_ros_time_override(rcl_time_point_t time_point)
{
  rcl_ret_t ret = rcl_set_ros_time_override(rcl_clock_.get(), time_point.nanoseconds);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to set ROS time override");
  }

  if (PyErr_Occurred()) {
    // Time jump callbacks raised
    throw py::error_already_set();
  }
}

/// Called when a time jump occurs.
void
_rclpy_on_time_jump(
  const rcl_time_jump_t * time_jump,
  bool before_jump,
  void * user_data)
{
  if (PyErr_Occurred()) {
    // An earlier time jump callback already raised an exception
    return;
  }
  auto pyjump_handle_c = static_cast<PyObject *>(user_data);
  auto pyjump_handle = py::reinterpret_borrow<py::object>(pyjump_handle_c);

  if (before_jump) {
    // Call pre jump callback with no arguments
    py::object pre_callback = pyjump_handle.attr("_pre_callback");
    if (pre_callback.is_none()) {
      return;
    }
    pre_callback();
  } else {
    // Call post jump callback with JumpInfo as an argument
    py::object post_callback = pyjump_handle.attr("_post_callback");
    if (post_callback.is_none()) {
      return;
    }
    py::object clock_change = py::cast(time_jump->clock_change);
    post_callback(
      py::dict("clock_change"_a = clock_change, "delta"_a = time_jump->delta.nanoseconds));
  }
}

void Clock::add_clock_callback(
  py::object pyjump_handle,
  bool on_clock_change,
  int64_t min_forward,
  int64_t min_backward)
{
  rcl_jump_threshold_t threshold;
  threshold.on_clock_change = on_clock_change;
  threshold.min_forward.nanoseconds = min_forward;
  threshold.min_backward.nanoseconds = min_backward;

  rcl_ret_t ret = rcl_clock_add_jump_callback(
    rcl_clock_.get(), threshold, _rclpy_on_time_jump, pyjump_handle.ptr());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to add time jump callback");
  }
}

void Clock::remove_clock_callback(py::object pyjump_handle)
{
  rcl_ret_t ret = rcl_clock_remove_jump_callback(
    rcl_clock_.get(), _rclpy_on_time_jump, pyjump_handle.ptr());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to remove time jump callback");
  }
}

void define_clock(py::object module)
{
  py::class_<Clock, Destroyable, std::shared_ptr<Clock>>(module, "Clock")
  .def(py::init<rcl_clock_type_t>())
  .def_property_readonly(
    "pointer", [](const Clock & clock) {
      return reinterpret_cast<size_t>(clock.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "get_now", &Clock::get_now,
    "Current value of the clock")
  .def(
    "get_ros_time_override_is_enabled",
    &Clock::get_ros_time_override_is_enabled,
    "Returns if a clock using ROS time has the ROS time override enabled.")
  .def(
    "set_ros_time_override_is_enabled",
    &Clock::set_ros_time_override_is_enabled,
    "Set if a clock using ROS time has the ROS time override enabled.")
  .def(
    "set_ros_time_override",
    &Clock::set_ros_time_override,
    "Set the ROS time override for a clock using ROS time.")
  .def(
    "add_clock_callback",
    &Clock::add_clock_callback,
    "Add a time jump callback to a clock.")
  .def(
    "remove_clock_callback",
    &Clock::remove_clock_callback,
    "Remove a time jump callback from a clock.");
}
}  // namespace rclpy
