// Copyright 2025 Brad Martin
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

#ifndef RCLPY__EVENTS_EXECUTOR__DELAYED_EVENT_THREAD_HPP_
#define RCLPY__EVENTS_EXECUTOR__DELAYED_EVENT_THREAD_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

#include "events_executor/events_queue.hpp"

namespace rclpy
{
namespace events_executor
{

/// This object manages posting an event handler to an EventsQueue after a specified delay.  The
/// current delay may be changed or canceled at any time.  This is done by way of a self-contained
/// child thread to perform the wait.
class DelayedEventThread
{
public:
  /// The pointer is aliased and must live for the lifetime of this object.
  explicit DelayedEventThread(EventsQueue *);
  ~DelayedEventThread();

  /// Schedules an event handler to be enqueued at the specified time point.  Replaces any previous
  /// wait and handler, which will never be dispatched if it has not been already.
  void EnqueueAt(std::chrono::steady_clock::time_point when, std::function<void()> handler);

  /// Cancels any previously-scheduled handler.
  void Cancel() {EnqueueAt({}, {});}

private:
  void RunThread();

  EventsQueue * const events_queue_;
  std::mutex mutex_;
  bool done_{};
  std::condition_variable cv_;
  std::chrono::steady_clock::time_point when_;
  std::function<void()> handler_;
  std::thread thread_;
};

}  // namespace events_executor
}  // namespace rclpy

#endif  // RCLPY__EVENTS_EXECUTOR__DELAYED_EVENT_THREAD_HPP_
