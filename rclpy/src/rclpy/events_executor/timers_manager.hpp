// Copyright 2024-2025 Brad Martin
// Copyright 2024 Merlin Labs, Inc.
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

#ifndef RCLPY__EVENTS_EXECUTOR__TIMERS_MANAGER_HPP_
#define RCLPY__EVENTS_EXECUTOR__TIMERS_MANAGER_HPP_

#include <pybind11/pybind11.h>

#include <rcl/time.h>
#include <rcl/timer.h>

#include <functional>
#include <memory>
#include <unordered_map>

#include "events_executor/events_queue.hpp"
#include "events_executor/python_hasher.hpp"
#include "events_executor/scoped_with.hpp"

namespace rclpy
{
namespace events_executor
{

/// This class manages low-level rcl timers in the system on behalf of EventsExecutor.
class RclTimersManager
{
public:
  /// The given pointer is aliased and must live for the lifetime of this object.
  explicit RclTimersManager(EventsQueue *);
  ~RclTimersManager();

  void AddTimer(rcl_timer_t *, std::function<void()> ready_callback);
  void RemoveTimer(rcl_timer_t *);

private:
  EventsQueue * const events_queue_;

  class ClockManager;
  /// Handlers for each distinct clock source in the system.
  std::unordered_map<const rcl_clock_t *, std::shared_ptr<ClockManager>> clock_managers_;
};

/// This class manages rclpy.Timer Python objects on behalf of EventsExecutor.
class TimersManager
{
public:
  /// @param events_queue is aliased and must live for the lifetime of this object.
  /// @param timer_ready_callback will be invoked with the timer handle and info whenever a managed
  /// timer is ready for servicing.
  TimersManager(
    EventsQueue * events_queue,
    std::function<void(pybind11::handle)> timer_ready_callback);
  ~TimersManager();

  /// Accessor for underlying rcl timer manager, for management of non-Python timers.
  RclTimersManager & rcl_manager() {return rcl_manager_;}

  // Both of these methods expect the GIL to be held when they are called.
  void AddTimer(pybind11::handle timer);
  void RemoveTimer(pybind11::handle timer);

private:
  struct PyRclMapping
  {
    /// Marks the corresponding Python object as in-use for as long as we're using the rcl pointer
    /// derived from it.
    std::unique_ptr<ScopedWith> with;

    /// The underlying rcl object
    rcl_timer_t * rcl_ptr{};
  };

  RclTimersManager rcl_manager_;
  const std::function<void(pybind11::handle)> ready_callback_;

  std::unordered_map<pybind11::handle, PyRclMapping, PythonHasher> timer_mappings_;
};

}  // namespace events_executor
}  // namespace rclpy

#endif  // RCLPY__EVENTS_EXECUTOR__TIMERS_MANAGER_HPP_
