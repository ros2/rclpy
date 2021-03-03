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
reset_timer(py::capsule pytimer)
{
  auto timer = static_cast<rcl_timer_t *>(
    rclpy_handle_get_pointer_from_capsule(pytimer.ptr(), "rcl_timer_t"));
  if (!timer) {
    throw py::error_already_set();
  }
  rcl_ret_t ret = rcl_timer_reset(timer);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to reset timer");
  }
}

bool
is_timer_ready(py::capsule pytimer)
{
  auto timer = static_cast<rcl_timer_t *>(
    rclpy_handle_get_pointer_from_capsule(pytimer.ptr(), "rcl_timer_t"));
  if (!timer) {
    throw py::error_already_set();
  }
  bool is_ready;
  rcl_ret_t ret = rcl_timer_is_ready(timer, &is_ready);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to check timer ready");
  }
  return is_ready;
}

void
call_timer(py::capsule pytimer)
{
  auto timer = static_cast<rcl_timer_t *>(
    rclpy_handle_get_pointer_from_capsule(pytimer.ptr(), "rcl_timer_t"));
  if (!timer) {
    throw py::error_already_set();
  }
  rcl_ret_t ret = rcl_timer_call(timer);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to call timer");
  }
}

void
change_timer_period(py::capsule pytimer, int64_t period_nsec)
{
  auto timer = static_cast<rcl_timer_t *>(
    rclpy_handle_get_pointer_from_capsule(pytimer.ptr(), "rcl_timer_t"));
  if (!timer) {
    throw py::error_already_set();
  }
  int64_t old_period;
  rcl_ret_t ret = rcl_timer_exchange_period(timer, period_nsec, &old_period);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to exchange timer period");
  }
}

int64_t
time_until_next_call(py::capsule pytimer)
{
  auto timer = static_cast<rcl_timer_t *>(
    rclpy_handle_get_pointer_from_capsule(pytimer.ptr(), "rcl_timer_t"));
  if (!timer) {
    throw py::error_already_set();
  }
  int64_t remaining_time;
  rcl_ret_t ret = rcl_timer_get_time_until_next_call(timer, &remaining_time);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get time until next timer call");
  }

  return remaining_time;
}

int64_t
time_since_last_call(py::capsule pytimer)
{
  auto timer = static_cast<rcl_timer_t *>(
    rclpy_handle_get_pointer_from_capsule(pytimer.ptr(), "rcl_timer_t"));
  if (!timer) {
    throw py::error_already_set();
  }
  int64_t elapsed_time;
  rcl_ret_t ret = rcl_timer_get_time_since_last_call(timer, &elapsed_time);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get time since last timer call");
  }

  return elapsed_time;
}

int64_t
get_timer_period(py::capsule pytimer)
{
  auto timer = static_cast<rcl_timer_t *>(
    rclpy_handle_get_pointer_from_capsule(pytimer.ptr(), "rcl_timer_t"));
  if (!timer) {
    throw py::error_already_set();
  }
  int64_t timer_period;
  rcl_ret_t ret = rcl_timer_get_period(timer, &timer_period);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to get timer period");
  }
  return timer_period;
}

void
cancel_timer(py::capsule pytimer)
{
  auto timer = static_cast<rcl_timer_t *>(
    rclpy_handle_get_pointer_from_capsule(pytimer.ptr(), "rcl_timer_t"));
  if (!timer) {
    throw py::error_already_set();
  }
  rcl_ret_t ret = rcl_timer_cancel(timer);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to cancel timer");
  }
}

bool
is_timer_canceled(py::capsule pytimer)
{
  auto timer = static_cast<rcl_timer_t *>(
    rclpy_handle_get_pointer_from_capsule(pytimer.ptr(), "rcl_timer_t"));
  if (!timer) {
    throw py::error_already_set();
  }
  bool is_canceled;
  rcl_ret_t ret = rcl_timer_is_canceled(timer, &is_canceled);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to check if timer is canceled");
  }
  return is_canceled;
}

/// Handle destructor for timer
static void
_rclpy_destroy_timer(void * p)
{
  auto tmr = static_cast<rcl_timer_t *>(p);
  if (!tmr) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_timer got NULL pointer");
    return;
  }

  rcl_ret_t ret = rcl_timer_fini(tmr);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini timer: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(tmr);
}

py::capsule
create_timer(py::capsule pyclock, py::capsule pycontext, int64_t period_nsec)
{
  auto clock = static_cast<rcl_clock_t *>(
    rclpy_handle_get_pointer_from_capsule(pyclock.ptr(), "rcl_clock_t"));
  if (!clock) {
    throw py::error_already_set();
  }

  auto context = static_cast<rcl_context_t *>(
    rclpy_handle_get_pointer_from_capsule(pycontext.ptr(), "rcl_context_t"));
  if (!context) {
    throw py::error_already_set();
  }

  auto deleter = [](rcl_timer_t * ptr) {_rclpy_destroy_timer(ptr);};
  auto timer = std::unique_ptr<rcl_timer_t, decltype(deleter)>(
    static_cast<rcl_timer_t *>(PyMem_Malloc(sizeof(rcl_timer_t))),
    deleter);
  if (!timer) {
    throw std::bad_alloc();
  }

  *timer = rcl_get_zero_initialized_timer();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_timer_init(timer.get(), clock, context, period_nsec, NULL, allocator);
  if (ret != RCL_RET_OK) {
    throw RCLError("failed to create timer");
  }

  PyObject * pytimer_c =
    rclpy_create_handle_capsule(timer.get(), "rcl_timer_t", _rclpy_destroy_timer);
  if (!pytimer_c) {
    throw py::error_already_set();
  }
  auto pytimer = py::reinterpret_steal<py::capsule>(pytimer_c);
  // pytimer now owns the rcl_timer_t
  timer.release();

  auto timer_handle = static_cast<rclpy_handle_t *>(pytimer);
  auto clock_handle = static_cast<rclpy_handle_t *>(pyclock);
  auto context_handle = static_cast<rclpy_handle_t *>(pycontext);

  _rclpy_handle_add_dependency(timer_handle, context_handle);
  if (PyErr_Occurred()) {
    _rclpy_handle_dec_ref(timer_handle);
    throw py::error_already_set();
  }
  _rclpy_handle_add_dependency(timer_handle, clock_handle);
  if (PyErr_Occurred()) {
    _rclpy_handle_dec_ref(timer_handle);
    throw py::error_already_set();
  }
  return pytimer;
}

}  // namespace rclpy
