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
#include <pybind11/stl.h>

#include <rcl/timer.h>

#include <memory>
#include <optional>

#include "clock.hpp"
#include "context.hpp"
#include "destroyable.hpp"

namespace py = pybind11;

namespace rclpy
{

class Timer : public Destroyable, public std::enable_shared_from_this<Timer>
{
public:
  /// Create a timer
  /**
   * This class will create a timer
   *
   * Raises RCLError on initialization failure
   * Raises TypeError if argument of invalid type
   * Raises ValueError if argument cannot be converted
   *
   * \param[in] clock pycapsule containing an rcl_clock_t
   * \param[in] context Capsule for an rcl_timer_t
   * \param[in] period_nsec The period of the timer in nanoseconds
   * \return a timer capsule
   */
  Timer(Clock & clock, Context & context, int64_t period_nsec);

  ~Timer() = default;

  /// Reset the timer
  /**
   * Raises RCLError if the timer cannot be reset
   */
  void reset_timer();

  /// Checks if timer reached its timeout
  /**
   *  Raises RCLError if there is an rcl error
   *
   * \return True if the timer is ready
   */
  bool is_timer_ready();

  /// Set the last call time and start counting again
  /**
   * Raises RCLError if there is an rcl error
   *
   */
  void call_timer();

  /// Update the timer period
  /**
   * The change in period will take effect after the next timer call
   *
   * Raises RCLError if the timer period could not be changed
   *
   * \param[in] period_nsec the new period in nanoseconds
   */
  void change_timer_period(int64_t period_nsec);

  /// Get the time before the timer will be ready
  /**
   * the returned time can be negative, this means that the timer is ready and hasn't been called yet
   *
   * Raises RCLError there is an rcl error
   *
   * \return the time until next call in nanoseconds.
   *   std::nullopt if the timer is canceled.
   */
  std::optional<int64_t> time_until_next_call();

  /// Get the time since the timer has been called
  /**
   * Raises RCLError if there is an rcl error
   *
   * \return the time since last call in nanoseconds
   */
  int64_t time_since_last_call();

  /// Returns the period of the timer in nanoseconds
  /**
   * Raises RCLError if the timer period cannot be retrieved
   *
   * \return the time since the last call in nanoseconds
   */
  int64_t get_timer_period();

  /// Cancel the timer
  /**
   * Raises RCLError if the timmer cannot be canceled
   *
   */
  void cancel_timer();

  /// Checks if timer is cancelled
  /**
   * Raises RCLError if there is an rcl error
   *
   * \return True if the timer is canceled
   */
  bool is_timer_canceled();

  /// Get rcl_timer_t pointer
  rcl_timer_t *
  rcl_ptr() const
  {
    return rcl_timer_.get();
  }

  /// Force an early destruction of this object
  void destroy() override;

private:
  Context context_;
  Clock clock_;
  std::shared_ptr<rcl_timer_t> rcl_timer_;
};

/// Define a pybind11 wrapper for an rcl_timer_t
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_timer(py::object module);
}  // namespace rclpy

#endif  // RCLPY__TIMER_HPP_
