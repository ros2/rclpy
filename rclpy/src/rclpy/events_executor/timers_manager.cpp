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
#include <utility>

#include <asio/post.hpp>
#include <asio/steady_timer.hpp>

#include "timer.hpp"

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
// * The rcl timer interface doesn't expose any way to get timer expiration info in absolute terms;
//   you can only find out 'time until next callback'.  This means if you are trying to sort the
//   list of timers, the 'value' of each entry in the heap changes depending on when you look at it
//   during the process of sorting.
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
/// construction and destruction) are assumed to happen on the thread running the provided asio
/// executor.
class RclTimersManager::ClockManager
{
public:
  ClockManager(const asio::any_io_executor & executor, rcl_clock_t * clock)
  : executor_(executor), clock_(clock)
  {
    // Need to establish a clock jump callback so we can tell when debug time is updated.
    rcl_jump_threshold_t threshold{.on_clock_change = true, .min_forward = 1, .min_backward = -1};
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
        asio::post(executor_, std::bind(&ClockManager::HandleJump, this, on_debug));
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
    reset_cb_ = [this]() {asio::post(executor_, std::bind(&ClockManager::UpdateTimers, this));};

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

  void AddTimer(rcl_timer_t * timer, std::function<void()> ready_callback)
  {
    // All timers have the same reset callback
    if (RCL_RET_OK != rcl_timer_set_on_reset_callback(timer, RclTimerResetTrampoline, &reset_cb_)) {
      throw std::runtime_error(
        std::string("Failed to set timer reset callback: ") + rcl_get_error_string().str);
    }
    timers_[timer] = ready_callback;
    if (timers_.size() == WARN_TIMERS_COUNT) {
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
  void HandleJump(bool on_debug_time)
  {
    on_debug_time_ = on_debug_time;
    UpdateTimers();
  }

  void UpdateTimers()
  {
    // First, evaluate all of our timers and dispatch any that are ready now.  While we're at it,
    // keep track of the earliest next timer callback that is due.
    std::optional<int64_t> next_ready_ns;
    for (const auto & timer_cb_pair : timers_) {
      auto next_call_ns = GetNextCallNanoseconds(timer_cb_pair.first);
      if (next_call_ns <= 0) {
        // This just notifies RCL that we're considering the timer triggered, for the purposes of
        // updating the next trigger time.
        const auto ret = rcl_timer_call(timer_cb_pair.first);
        switch (ret) {
          case RCL_RET_OK:
            break;
          case RCL_RET_TIMER_CANCELED:
            // Someone apparently canceled the timer *after* we just queried the next call time?
            // Nevermind, then...
            rcl_reset_error();
            continue;
          default:
            throw std::runtime_error(
              std::string("Failed to call RCL timer: ") + rcl_get_error_string().str);
        }

        // Post the user callback to be invoked later once timing-sensitive code is done.
        asio::post(executor_, timer_cb_pair.second);

        // Update time until *next* call.
        next_call_ns = GetNextCallNanoseconds(timer_cb_pair.first);
      }
      if (!next_ready_ns || (next_call_ns < *next_ready_ns)) {
        next_ready_ns = next_call_ns;
      }
    }

    // If we're not on debug time, we should schedule another wakeup when we anticipate the next
    // timer being ready.  If we are, we'll just re-check everything at the next jump callback.
    if (!on_debug_time_ && next_ready_ns) {
      next_update_wait_.expires_from_now(std::chrono::nanoseconds(*next_ready_ns));
      next_update_wait_.async_wait([this](const asio::error_code & ec) {
          if (!ec) {
            UpdateTimers();
          } else if (ec != asio::error::operation_aborted) {
            throw std::runtime_error("Error waiting for next timer: " + ec.message());
          }
      });
    } else {
      next_update_wait_.cancel();
    }
  }

  /// Returns the number of nanoseconds until the next callback on the given timer is due.  Value
  /// may be negative or zero if callback time has already been reached.  Returns std::nullopt if
  /// the timer is canceled.
  static std::optional<int64_t> GetNextCallNanoseconds(const rcl_timer_t * rcl_timer)
  {
    int64_t time_until_next_call{};
    const rcl_ret_t ret = rcl_timer_get_time_until_next_call(rcl_timer, &time_until_next_call);
    switch (ret) {
      case RCL_RET_OK:
        return time_until_next_call;
      case RCL_RET_TIMER_CANCELED:
        return {};
      default:
        throw std::runtime_error(
          std::string("Failed to fetch timer ready time: ") + rcl_get_error_string().str);
    }
  }

  asio::any_io_executor executor_;
  rcl_clock_t * const clock_;
  ClockJumpCallbackT jump_cb_;
  TimerResetCallbackT reset_cb_;
  bool on_debug_time_{};

  std::unordered_map<rcl_timer_t *, std::function<void()>> timers_;
  asio::steady_timer next_update_wait_{executor_};
};

RclTimersManager::RclTimersManager(const asio::any_io_executor & executor)
: executor_(executor) {}

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

void RclTimersManager::AddTimer(rcl_timer_t * timer, std::function<void()> ready_callback)
{
  // Figure out the clock this timer is using, make sure a manager exists for that clock, then
  // forward the timer to that clock's manager.
  rcl_clock_t * clock = GetTimerClock(timer);
  auto it = clock_managers_.find(clock);
  if (it == clock_managers_.end()) {
    std::tie(it, std::ignore) = clock_managers_.insert(
      std::make_pair(clock, std::make_unique<ClockManager>(executor_, clock)));
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
  const asio::any_io_executor & executor, std::function<void(py::handle)> timer_ready_callback)
: rcl_manager_(executor), ready_callback_(timer_ready_callback)
{
}

TimersManager::~TimersManager() {}

void TimersManager::AddTimer(py::handle timer)
{
  PyRclMapping mapping;
  py::handle handle = timer.attr("handle");
  mapping.with = std::make_unique<ScopedWith>(handle);
  mapping.rcl_ptr = py::cast<Timer>(handle).rcl_ptr();
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
