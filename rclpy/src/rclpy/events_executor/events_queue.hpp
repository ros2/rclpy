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

#ifndef RCLPY__EVENTS_EXECUTOR__EVENTS_QUEUE_HPP_
#define RCLPY__EVENTS_EXECUTOR__EVENTS_QUEUE_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>

namespace rclpy
{
namespace events_executor
{

/// This class represents a queue of event handlers to dispatch.  Handlers may be enqueued from any
/// thread, and will always be dispatched during a call to a Run*() method on the thread invoking
/// that method.  Multiple threads may not invoke Run*() methods simultaneously.
class EventsQueue
{
public:
  ~EventsQueue();

  /// Add an event handler to the queue to be dispatched.  Can be invoked by any thread.
  void Enqueue(std::function<void()>);

  /// Run event handlers indefinitely, until stopped.
  void Run();

  /// Run all ready event handlers, and any that become ready before the given deadline.  Calling
  /// Stop() will make this return immediately even if ready handlers are enqueued.
  void RunUntil(std::chrono::steady_clock::time_point);

  /// Causes any Run*() methods outstanding to return immediately.  Can be invoked from any thread.
  /// The stopped condition persists (causing any *subsequent* Run*() calls to also return) until
  /// Restart() is invoked.
  void Stop();

  /// Ends a previous stopped condition, allowing Run*() methods to operate again.  May be invoked
  /// from any thread.
  void Restart();

private:
  std::queue<std::function<void()>> queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stopped_{};
};

}  // namespace events_executor
}  // namespace rclpy

#endif  // RCLPY__EVENTS_EXECUTOR__EVENTS_QUEUE_HPP_
