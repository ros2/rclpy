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

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rcl/types.h>

#include <cstring>
#include <memory>
#include <stdexcept>

#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "time_point.hpp"

using pybind11::literals::operator""_a;

namespace rclpy
{
/// Destructor for a clock
static void
_rclpy_destroy_clock(void * p)
{
  auto clock = static_cast<rcl_clock_t *>(p);
  if (!clock) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to get clock pointer in destructor");
    rcl_reset_error();
    return;
  }

  rcl_ret_t ret_clock = rcl_clock_fini(clock);
  PyMem_Free(clock);
  if (ret_clock != RCL_RET_OK) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini clock: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
  }
}

py::capsule
create_clock(rcl_clock_type_t clock_type)
{
  auto deleter = [](rcl_clock_t * ptr) {_rclpy_destroy_clock(ptr);};
  auto clock = std::unique_ptr<rcl_clock_t, decltype(deleter)>(
    static_cast<rcl_clock_t *>(PyMem_Malloc(sizeof(rcl_clock_t))),
    deleter);
  if (!clock) {
    throw std::bad_alloc();
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_clock_init(clock_type, clock.get(), &allocator);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to initialize clock");
  }

  PyObject * pyclock_c =
    rclpy_create_handle_capsule(clock.get(), "rcl_clock_t", _rclpy_destroy_clock);
  auto pyclock = py::reinterpret_steal<py::capsule>(pyclock_c);
  // pyclock now owns the rcl_clock_t
  clock.release();
  return pyclock;
}

py::capsule
clock_get_now(py::capsule pyclock)
{
  auto clock = static_cast<rcl_clock_t *>(
    rclpy_handle_get_pointer_from_capsule(pyclock.ptr(), "rcl_clock_t"));
  if (!clock) {
    throw py::error_already_set();
  }

  int64_t nanoseconds;
  rcl_ret_t ret = rcl_clock_get_now(clock, &nanoseconds);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get current clock value");
  }

  return create_time_point(nanoseconds, clock->type);
}

bool
clock_get_ros_time_override_is_enabled(py::capsule pyclock)
{
  auto clock = static_cast<rcl_clock_t *>(
    rclpy_handle_get_pointer_from_capsule(pyclock.ptr(), "rcl_clock_t"));
  if (!clock) {
    throw py::error_already_set();
  }

  bool is_enabled;
  rcl_ret_t ret = rcl_is_enabled_ros_time_override(clock, &is_enabled);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get ROS time override status");
  }

  return is_enabled;
}

void
clock_set_ros_time_override_is_enabled(py::capsule pyclock, bool enabled)
{
  auto clock = static_cast<rcl_clock_t *>(
    rclpy_handle_get_pointer_from_capsule(pyclock.ptr(), "rcl_clock_t"));
  if (!clock) {
    throw py::error_already_set();
  }

  rcl_ret_t ret;
  if (enabled) {
    ret = rcl_enable_ros_time_override(clock);
  } else {
    ret = rcl_disable_ros_time_override(clock);
  }
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to set ROS time override");
  }
  if (PyErr_Occurred()) {
    // Time jump callbacks raised
    throw py::error_already_set();
  }
}

void
clock_set_ros_time_override(py::capsule pyclock, py::capsule pytime_point)
{
  auto clock = static_cast<rcl_clock_t *>(
    rclpy_handle_get_pointer_from_capsule(pyclock.ptr(), "rcl_clock_t"));
  if (!clock) {
    throw py::error_already_set();
  }

  int64_t nanoseconds = time_point_get_nanoseconds(pytime_point);

  rcl_ret_t ret = rcl_set_ros_time_override(clock, nanoseconds);
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
  const struct rcl_time_jump_t * time_jump,
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

void
add_jump_callback(
  py::capsule pyclock,
  py::object pyjump_handle,
  bool on_clock_change,
  int64_t min_forward,
  int64_t min_backward)
{
  auto clock = static_cast<rcl_clock_t *>(
    rclpy_handle_get_pointer_from_capsule(pyclock.ptr(), "rcl_clock_t"));
  if (!clock) {
    throw py::error_already_set();
  }

  rcl_jump_threshold_t threshold;
  threshold.on_clock_change = on_clock_change;
  threshold.min_forward.nanoseconds = min_forward;
  threshold.min_backward.nanoseconds = min_backward;

  rcl_ret_t ret = rcl_clock_add_jump_callback(
    clock, threshold, _rclpy_on_time_jump, pyjump_handle.ptr());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to add time jump callback");
  }
}

void
remove_jump_callback(py::capsule pyclock, py::object pyjump_handle)
{
  auto clock = static_cast<rcl_clock_t *>(
    rclpy_handle_get_pointer_from_capsule(pyclock.ptr(), "rcl_clock_t"));
  if (!clock) {
    throw py::error_already_set();
  }

  rcl_ret_t ret = rcl_clock_remove_jump_callback(
    clock, _rclpy_on_time_jump, pyjump_handle.ptr());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to remove time jump callback");
  }
}
}  // namespace rclpy
