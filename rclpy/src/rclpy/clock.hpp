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

#ifndef RCLPY__CLOCK_HPP_
#define RCLPY__CLOCK_HPP_

#include <pybind11/pybind11.h>

#include <rcl/time.h>

namespace py = pybind11;

namespace rclpy
{
/// Create a clock
/**
 * Raises RCLError on initialization failure
 * Raises TypeError if argument of invalid type
 *
 * This function creates a Clock object of the specified type
 * \param[in] clock_type enum of type ClockType
 * \return  Capsule to an rcl_clock_t object
 */
py::capsule
create_clock(rcl_clock_type_t clock_type);

/// Returns the current value of the clock
/**
 * Raises ValueError if pyclock is not a clock capsule
 * Raises RCLError if the clock's value cannot be retrieved
 *
 * \param[in] pyclock Capsule pointing to the clock
 * \return capsule to a time point with the current time
 */
py::capsule
clock_get_now(py::capsule pyclock);

/// Returns if a clock using ROS time has the ROS time override enabled.
/**
 * Raises ValueError if pyclock is not a clock capsule
 * Raises RCLError if the clock's ROS time override state cannot be retrieved
 *
 * \param[in] pyclock Capsule pointing to the clock
 * \return true if ROS time is enabled
 */
bool
clock_get_ros_time_override_is_enabled(py::capsule pyclock);

/// Set if a clock using ROS time has the ROS time override enabled.
/**
 * Raises ValueError if pyclock is not a clock capsule
 * Raises RCLError if the clock's ROS time override cannot be set
 *
 * \param[in] pyclock Capsule pointing to the clock to set
 * \param[in] enabled Override state to set
 */
void
clock_set_ros_time_override_is_enabled(py::capsule pyclock, bool enabled);

/// Set the ROS time override for a clock using ROS time.
/**
 * Raises ValueError if pyclock is not a clock capsule, or
 * pytime_point is not a time point capsule
 * Raises RRCLError if the clock's ROS time override cannot be set
 *
 * \param[in] pyclock Capsule pointing to the clock to set
 * \param[in] pytime_point Capsule pointing to the time point
 */
void
clock_set_ros_time_override(py::capsule py_clock, py::capsule pytime_point);

/// Add a time jump callback to a clock.
/**
 * Raises ValueError if pyclock is not a clock capsule, or
 * any argument is invalid
 * Raises RCLError if the callback cannot be added
 *
 * \param[in] pyclock Capsule pointing to the clock to set
 * \param[in] pyjump_handle Instance of rclpy.clock.JumpHandle
 * \param[in] on_clock_change True if callback should be called when ROS time is toggled
 * \param[in] min_forward minimum nanoseconds to trigger forward jump callback
 * \param[in] min_backward minimum negative nanoseconds to trigger backward jump callback
 */
void
add_jump_callback(
  py::capsule pyclock,
  py::object pyjump_handle,
  bool on_clock_change,
  int64_t min_forward,
  int64_t min_backward);

/// Remove a time jump callback from a clock.
/**
 * Raises ValueError if pyclock is not a clock capsule, or
 * any argument is invalid
 * Raises RCLError if the callback cannot be added
 *
 * \param[in] pyclock Capsule pointing to the clock to set
 * \param[in] pyjump_handle Instance of rclpy.clock.JumpHandle
 */
void
remove_jump_callback(py::capsule pyclock, py::object pyjump_handle);
}  // namespace rclpy

#endif  // RCLPY__CLOCK_HPP_
