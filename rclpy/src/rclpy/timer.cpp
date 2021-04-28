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

#include <rcl/context.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/timer.h>
#include <rcl/types.h>

#include <memory>
#include <stdexcept>

#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "context.hpp"
#include "timer.hpp"

namespace rclpy
{
void
Timer::destroy()
{
  rcl_timer_.reset();
  clock_handle_.reset();
}

Timer::Timer(
  Clock & rclcy_clock, py::capsule pycontext, int64_t period_nsec)
{
  clock_handle_ = rclcy_clock.get_shared_ptr();

  auto context = static_cast<rcl_context_t *>(
    rclpy_handle_get_pointer_from_capsule(pycontext.ptr(), "rcl_context_t"));
  if (!context) {
    throw py::error_already_set();
  }

  // Create a client
  rcl_timer_ = std::shared_ptr<rcl_timer_t>(
    new rcl_timer_t,
    [](rcl_timer_t * timer)
    {
      rcl_ret_t ret = rcl_timer_fini(timer);
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini timer: %s",
          rcl_get_error_string().str);
      }
      delete timer;
    });

  *rcl_timer_ = rcl_get_zero_initialized_timer();

  rcl_allocator_t allocator = rcl_get_default_allocator();

  rcl_ret_t ret = rcl_timer_init(
    rcl_timer_.get(), clock_handle_.get(), context,
    period_nsec, NULL, allocator);

  if (RCL_RET_OK != ret) {
    throw RCLError("failed to create timer");
  }
}

void Timer::reset_timer()
{
  rcl_ret_t ret = rcl_timer_reset(rcl_timer_.get());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to reset timer");
  }
}

bool Timer::is_timer_ready()
{
  bool is_ready;
  rcl_ret_t ret = rcl_timer_is_ready(rcl_timer_.get(), &is_ready);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to check timer ready");
  }
  return is_ready;
}

void Timer::call_timer()
{
  rcl_ret_t ret = rcl_timer_call(rcl_timer_.get());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to call timer");
  }
}

void Timer::change_timer_period(int64_t period_nsec)
{
  int64_t old_period;
  rcl_ret_t ret = rcl_timer_exchange_period(rcl_timer_.get(), period_nsec, &old_period);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to exchange timer period");
  }
}

int64_t Timer::time_until_next_call()
{
  int64_t remaining_time;
  rcl_ret_t ret = rcl_timer_get_time_until_next_call(rcl_timer_.get(), &remaining_time);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get time until next timer call");
  }

  return remaining_time;
}

int64_t Timer::time_since_last_call()
{
  int64_t elapsed_time;
  rcl_ret_t ret = rcl_timer_get_time_since_last_call(rcl_timer_.get(), &elapsed_time);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get time since last timer call");
  }

  return elapsed_time;
}

int64_t Timer::get_timer_period()
{
  int64_t timer_period;
  rcl_ret_t ret = rcl_timer_get_period(rcl_timer_.get(), &timer_period);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get timer period");
  }
  return timer_period;
}

void Timer::cancel_timer()
{
  rcl_ret_t ret = rcl_timer_cancel(rcl_timer_.get());
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to cancel timer");
  }
}

bool Timer::is_timer_canceled()
{
  bool is_canceled;
  rcl_ret_t ret = rcl_timer_is_canceled(rcl_timer_.get(), &is_canceled);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to check if timer is canceled");
  }
  return is_canceled;
}

void
define_timer(py::object module)
{
  py::class_<Timer, Destroyable>(module, "Timer")
  .def(py::init<Clock &, py::capsule, int64_t>())
  .def_property_readonly(
    "pointer", [](const Timer & timer) {
      return reinterpret_cast<size_t>(timer.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def("reset_timer", &Timer::reset_timer, "Reset a timer.")
  .def(
    "is_timer_ready", &Timer::is_timer_ready,
    "Check if a timer as reached timeout.")
  .def(
    "call_timer", &Timer::call_timer,
    "Call a timer and starts counting again.")
  .def(
    "change_timer_period", &Timer::change_timer_period,
    "Set the period of a timer.")
  .def(
    "time_until_next_call", &Timer::time_until_next_call,
    "Get the remaining time before timer is ready.")
  .def(
    "time_since_last_call", &Timer::time_since_last_call,
    "Get the elapsed time since last timer call.")
  .def(
    "get_timer_period", &Timer::get_timer_period,
    "Get the period of a timer.")
  .def(
    "cancel_timer", &Timer::cancel_timer,
    "Cancel a timer.")
  .def(
    "is_timer_canceled", &Timer::is_timer_canceled,
    "Check if a timer is canceled.");
}

}  // namespace rclpy
