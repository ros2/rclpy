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

#ifndef RCLPY__TIMER_HPP_
#define RCLPY__TIMER_HPP_

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rclpy
{
/// Reset the timer
/**
 * Raise ValueError if capsule is not a timer
 * Raises RCLError if the timer cannot be reset
 *
 * \param[in] pytimer Capsule pointing to the timer
 */
void
reset_timer(py::capsule pytimer);

/// Checks if timer reached its timeout
/**
 *  Raises ValueError if pytimer is not a timer capsule
 *  Raises RCLError if there is an rcl error
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return True if the timer is ready
 */
bool
is_timer_ready(py::capsule pytimer);

/// Set the last call time and start counting again
/**
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RCLError if there is an rcl error
 *
 * \param[in] pytimer Capsule pointing to the timer
 */
void
call_timer(py::capsule pytimer);

/// Update the timer period
/**
 * The change in period will take effect after the next timer call
 *
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RCLError if the timer period could not be changed
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \param[in] period_nsec the new period in nanoseconds
 */
void
change_timer_period(py::capsule pytimer, int64_t period_nsec);

/// Get the time before the timer will be ready
/**
 * the returned time can be negative, this means that the timer is ready and hasn't been called yet
 *
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RCLError there is an rcl error
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return the time until next call in nanoseconds
 */
int64_t
time_until_next_call(py::capsule pytimer);

/// Get the time since the timer has been called
/**
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RCLError if there is an rcl error
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return the time since last call in nanoseconds
 */
int64_t
time_since_last_call(py::capsule pytimer);

/// Returns the period of the timer in nanoseconds
/**
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RCLError if the timer period cannot be retrieved
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return the time since the last call in nanoseconds
 */
int64_t
get_timer_period(py::capsule pytimer);

/// Cancel the timer
/**
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RCLError if the timmer cannot be canceled
 *
 * \param[in] pytimer Capsule pointing to the timer
 */
void
cancel_timer(py::capsule pytimer);

/// Checks if timer is cancelled
/**
 * Raises ValueError if pytimer is not a timer capsule
 * Raises RCLError if there is an rcl error
 *
 * \param[in] pytimer Capsule pointing to the timer
 * \return True if the timer is canceled
 */
bool
is_timer_canceled(py::capsule pytimer);

/// Create a timer
/**
 * When successful a Capsule pointing to the pointer of the created rcl_timer_t * structure
 * is returned
 *
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises RCLError on initialization failure
 * Raises TypeError if argument of invalid type
 * Raises ValueError if argument cannot be converted
 *
 * \param[in] clock pycapsule containing an rcl_clock_t
 * \param[in] period_nsec the period of the timer in nanoseconds
 * \return a timer capsule
 */
py::capsule
create_timer(py::capsule pyclock, py::capsule pycontext, int64_t period_nsec);
}  // namespace rclpy

#endif  // RCLPY__TIMER_HPP_
