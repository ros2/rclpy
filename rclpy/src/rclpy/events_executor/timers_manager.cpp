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
#include "events_executor/timers_manager.hpp"

#include <rcl/error_handling.h>

#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>

#include "events_executor/delayed_event_thread.hpp"
#include "timer.hpp"

namespace pl = std::placeholders;
namespace py = pybind11;

namespace rclpy
{
namespace events_executor
{

namespace
{

// Implementation note: the original iRobot TimersManager associated with the rclcpp EventsExecutor
// maintained a heap with all outstanding timers sorted by their next expiration time.  Here that
// approach will be skipped in favor of just looking at every timer every update for the following
// reasons:
//
// * This approach is simpler
// * In the applications this has been used in so far, no Node types exist that have a large number
//   of timers outstanding at once.  Assuming no more than a few timers exist in the whole process,
//   the heap seems like overkill.
// * Every time a timer ticks or is reset, the heap needs to be resorted anyway.
//
// We will however yell a bit if we ever see a large number of timers that disproves this
// assumption, so we can reassess this decision.
constexpr size_t WARN_TIMERS_COUNT = 8;

typedef std::function<void(const rcl_time_jump_t *)> ClockJumpCallbackT;
typedef std::function<void()> TimerResetCallbackT;

extern "C" void RclClockJumpTrampoline(
  const rcl_time_jump_t * time_jump, bool before_jump, void * user_data)
{
  // rcl calls this both before and after a clock change, and we never want the before callback, so
  // let's just eliminate that case early.
  if (before_jump) {
    return;
  }
  auto cb = reinterpret_cast<ClockJumpCallbackT *>(user_data);
  (*cb)(time_jump);
}

extern "C" void RclTimerResetTrampoline(const void * user_data, size_t)
{
  auto cb = reinterpret_cast<const TimerResetCallbackT *>(user_data);
  (*cb)();
}

}  // namespace

/// Manages a single clock source, and all timers operating on that source.  All methods (including
/// construction and destruction) are assumed to happen on the thread running the provided events
/// queue.
class RclTimersManager::ClockManager : public std::enable_shared_from_this<ClockManager>
{
public:
  ClockManager(EventsQueue * events_queue, rcl_clock_t * clock)
  : events_queue_(events_queue), clock_(clock)
  {
    // Need to establish a clock jump callback so we can tell when debug time is updated.
    rcl_jump_threshold_t threshold;
    threshold.on_clock_change = true;
    threshold.min_forward.nanoseconds = 1;
    threshold.min_backward.nanoseconds = -1;
    // Note, this callback could happen on any thread
    jump_cb_ = [this](const rcl_time_jump_t * time_jump) {
        bool on_debug{};
        switch (time_jump->clock_change) {
          case RCL_ROS_TIME_NO_CHANGE:
          case RCL_ROS_TIME_ACTIVATED:
            on_debug = true;
            break;
          case RCL_ROS_TIME_DEACTIVATED:
          case RCL_SYSTEM_TIME_NO_CHANGE:
            on_debug = false;
            break;
        }
        events_queue_->Enqueue(CallIfAlive(&ClockManager::HandleJump, on_debug));
      };
    if (
      RCL_RET_OK !=
      rcl_clock_add_jump_callback(clock_, threshold, RclClockJumpTrampoline, &jump_cb_))
    {
      throw std::runtime_error(
        std::string("Failed to set RCL clock jump callback: ") + rcl_get_error_string().str);
    }

    // This isn't necessary yet but every timer will eventually depend on it.  Again, this could
    // happen on any thread.
    reset_cb_ = [this]() {events_queue_->Enqueue(CallIfAlive(&ClockManager::UpdateTimers));};

    // Initialize which timebase we're on
    if (clock_->type == RCL_ROS_TIME) {
      if (RCL_RET_OK != rcl_is_enabled_ros_time_override(clock_, &on_debug_time_)) {
        throw std::runtime_error(
          std::string("Failed to get RCL clock override state: ") + rcl_get_error_string().str);
      }
    }
  }

  ~ClockManager()
  {
    if (RCL_RET_OK != rcl_clock_remove_jump_callback(clock_, RclClockJumpTrampoline, &jump_cb_)) {
      py::gil_scoped_acquire gil_acquire;
      py::print(
        std::string("Failed to remove RCL clock jump callback: ") + rcl_get_error_string().str);
    }
    while (!timers_.empty()) {
      RemoveTimer(timers_.begin()->first);
    }
  }

  bool empty() const {return timers_.empty();}

  void AddTimer(
    rcl_timer_t * timer, std::function<void()> ready_callback)
  {
    // All timers have the same reset callback
    if (
      RCL_RET_OK != rcl_timer_set_on_reset_callback(timer, RclTimerResetTrampoline, &reset_cb_))
    {
      throw std::runtime_error(
        std::string("Failed to set timer reset callback: ") + rcl_get_error_string().str);
    }
    timers_[timer] = ready_callback;
    if (timers_.size() >= WARN_TIMERS_COUNT) {
      py::print("Warning, the number of timers associated with this clock is large.");
      py::print("Management of this number of timers may be inefficient.");
    }
    UpdateTimers();
  }

  void RemoveTimer(rcl_timer_t * timer)
  {
    auto it = timers_.find(timer);
    if (it == timers_.end()) {
      throw py::key_error("Attempt to remove unmanaged timer");
    }

    if (RCL_RET_OK != rcl_timer_set_on_reset_callback(timer, nullptr, nullptr)) {
      throw std::runtime_error(
        std::string("Failed to clear timer reset callback: ") + rcl_get_error_string().str);
    }
    timers_.erase(it);
    // We could re-evaluate how long we need to block for now that a timer has been removed; but,
    // there's no real harm in one extra wakeup that then decides it doesn't need to do anything,
    // and this timer might not even be the next to fire, so we won't bother.
  }

private:
  /// Returns a function suitable for being invoked later, which would invoke the given method on
  /// `this` with the given args, provided that `this` still exists at that time.
  template<typename ... Args>
  std::function<void()> CallIfAlive(void (ClockManager::*method)(Args...), Args... args)
  {
    std::weak_ptr<ClockManager> weak_this(shared_from_this());
    return [ = ]() {
             auto locked = weak_this.lock();
             if (locked) {
               (locked.get()->*method)(args ...);
             }
           };
  }

  void HandleJump(bool on_debug_time)
  {
    on_debug_time_ = on_debug_time;
    UpdateTimers();
  }

  void UpdateTimers()
  {
    // Let's not assume that rcl_clock_get_now() and std::chrono::steady_clock::now() are on the
    // same timebase.
    int64_t rcl_now{};
    if (RCL_RET_OK != rcl_clock_get_now(clock_, &rcl_now)) {
      throw std::runtime_error(
        std::string("Failed to read RCL clock: ") + rcl_get_error_string().str);
    }
    const auto chrono_now = std::chrono::steady_clock::now();

    // First, evaluate all of our timers and dispatch any that are ready now.  While we're at it,
    // keep track of the earliest next timer callback that is due.
    std::optional<int64_t> next_ready_time_ns;
    for (const auto & timer_cb_pair : timers_) {
      auto this_next_time_ns = GetNextCallTimeNanoseconds(timer_cb_pair.first);
      if (this_next_time_ns) {
        if (*this_next_time_ns <= rcl_now) {
          ready_timers_.insert(timer_cb_pair.first);
          events_queue_->Enqueue(CallIfAlive(&ClockManager::DispatchTimer, timer_cb_pair.first));
        } else if (!next_ready_time_ns || (*this_next_time_ns < *next_ready_time_ns)) {
          next_ready_time_ns = this_next_time_ns;
        }
      }
    }

    // If we posted any timers for dispatch, then we'll re-evaluate things immediately after those
    // complete.  Otherwise, if we're on debug time, we'll re-check everything at the next jump
    // callback.  If neither of those things are true, then we need to schedule a wakeup for when
    // we anticipate the next timer being ready.
    if (ready_timers_.empty() && !on_debug_time_ && next_ready_time_ns) {
      next_update_wait_.EnqueueAt(
        chrono_now + std::chrono::nanoseconds(*next_ready_time_ns - rcl_now),
        CallIfAlive(&ClockManager::UpdateTimers));
    } else {
      next_update_wait_.Cancel();
    }
  }

  void DispatchTimer(rcl_timer_t * rcl_timer)
  {
    ready_timers_.erase(rcl_timer);
    // If we've dispatched all ready timers, then trigger another update to see when the next
    // timers will be ready.
    if (ready_timers_.empty()) {
      events_queue_->Enqueue(CallIfAlive(&ClockManager::UpdateTimers));
    }

    const auto map_it = timers_.find(rcl_timer);
    if (map_it == timers_.end()) {
      // Perhaps the timer was removed before a pending callback could be dispatched?
      return;
    }

    // This notifies RCL that we're considering the timer triggered, for the purposes of updating
    // the next trigger time.
    rcl_timer_call_info_t info;
    const auto ret = rcl_timer_call_with_info(rcl_timer, &info);
    switch (ret) {
      case RCL_RET_OK:
        // Dispatch the actual user callback.
        map_it->second();
        break;
      case RCL_RET_TIMER_CANCELED:
        // Someone canceled the timer after we queried the call time.  Nevermind, then...
        rcl_reset_error();
        break;
      default:
        throw std::runtime_error(
          std::string("Failed to call RCL timer: ") + rcl_get_error_string().str);
    }
  }

  /// Returns the absolute time in nanoseconds when the next callback on the given timer is due.
  /// Returns std::nullopt if the timer is canceled.
  static std::optional<int64_t> GetNextCallTimeNanoseconds(const rcl_timer_t * rcl_timer)
  {
    int64_t next_call_time{};
    const rcl_ret_t ret = rcl_timer_get_next_call_time(rcl_timer, &next_call_time);
    switch (ret) {
      case RCL_RET_OK:
        return next_call_time;
      case RCL_RET_TIMER_CANCELED:
        return {};
      default:
        throw std::runtime_error(
          std::string("Failed to fetch timer ready time: ") + rcl_get_error_string().str);
    }
  }

  EventsQueue * const events_queue_;
  rcl_clock_t * const clock_;
  ClockJumpCallbackT jump_cb_;
  TimerResetCallbackT reset_cb_;
  bool on_debug_time_{};

  std::unordered_map<rcl_timer_t *, std::function<void()>> timers_;
  std::unordered_set<rcl_timer_t *> ready_timers_;
  DelayedEventThread next_update_wait_{events_queue_};
};

RclTimersManager::RclTimersManager(EventsQueue * events_queue)
: events_queue_(events_queue) {}

RclTimersManager::~RclTimersManager() {}

namespace
{
rcl_clock_t * GetTimerClock(rcl_timer_t * timer)
{
  rcl_clock_t * clock{};
  if (RCL_RET_OK != rcl_timer_clock(timer, &clock)) {
    throw std::runtime_error(
      std::string("Failed to determine clock for timer: ") + rcl_get_error_string().str);
  }
  return clock;
}
}  // namespace

void RclTimersManager::AddTimer(
  rcl_timer_t * timer, std::function<void()> ready_callback)
{
  // Figure out the clock this timer is using, make sure a manager exists for that clock, then
  // forward the timer to that clock's manager.
  rcl_clock_t * clock = GetTimerClock(timer);
  auto it = clock_managers_.find(clock);
  if (it == clock_managers_.end()) {
    std::tie(it, std::ignore) = clock_managers_.insert(
      std::make_pair(clock, std::make_shared<ClockManager>(events_queue_, clock)));
  }
  it->second->AddTimer(timer, ready_callback);
}

void RclTimersManager::RemoveTimer(rcl_timer_t * timer)
{
  const rcl_clock_t * clock = GetTimerClock(timer);
  auto it = clock_managers_.find(clock);
  if (it == clock_managers_.end()) {
    throw py::key_error("Attempt to remove timer from unmanaged clock");
  }
  it->second->RemoveTimer(timer);
  if (it->second->empty()) {
    clock_managers_.erase(it);
  }
}

TimersManager::TimersManager(
  EventsQueue * events_queue,
  std::function<void(py::handle)> timer_ready_callback)
: rcl_manager_(events_queue), ready_callback_(timer_ready_callback)
{
}

TimersManager::~TimersManager() {}

void TimersManager::AddTimer(py::handle timer)
{
  PyRclMapping mapping;
  py::handle handle = timer.attr("handle");
  mapping.with = std::make_unique<ScopedWith>(handle);
  mapping.rcl_ptr = py::cast<const Timer &>(handle).rcl_ptr();
  rcl_manager_.AddTimer(mapping.rcl_ptr, std::bind(ready_callback_, timer));
  timer_mappings_[timer] = std::move(mapping);
}

void TimersManager::RemoveTimer(py::handle timer)
{
  const auto it = timer_mappings_.find(timer);
  if (it == timer_mappings_.end()) {
    throw py::key_error("Attempt to remove unmanaged timer");
  }
  rcl_manager_.RemoveTimer(it->second.rcl_ptr);
  timer_mappings_.erase(it);
}

}  // namespace events_executor
}  // namespace rclpy
