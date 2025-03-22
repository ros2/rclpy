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
#include "events_executor/delayed_event_thread.hpp"

#include <utility>

namespace rclpy
{
namespace events_executor
{

DelayedEventThread::DelayedEventThread(EventsQueue * events_queue)
: events_queue_(events_queue), thread_([this]() {RunThread();})
{
}

DelayedEventThread::~DelayedEventThread()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    done_ = true;
  }
  cv_.notify_one();
  thread_.join();
}

void DelayedEventThread::EnqueueAt(
  std::chrono::steady_clock::time_point when, std::function<void()> handler)
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    when_ = when;
    handler_ = handler;
  }
  cv_.notify_one();
}

void DelayedEventThread::RunThread()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (!done_) {
    if (handler_) {
      // Make sure we don't dispatch a stale wait if it changes while we're waiting.
      const auto latched_when = when_;
      if (
        (std::cv_status::timeout == cv_.wait_until(lock, latched_when)) && handler_ &&
        (when_ <= latched_when))
      {
        auto handler = std::move(handler_);
        handler_ = {};
        events_queue_->Enqueue(std::move(handler));
      }
    } else {
      // Wait indefinitely until we get signaled that there's something worth looking at.
      cv_.wait(lock);
    }
  }
}

}  // namespace events_executor
}  // namespace rclpy
