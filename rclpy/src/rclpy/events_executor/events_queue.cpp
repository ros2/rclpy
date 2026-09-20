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
#include "events_executor/events_queue.hpp"

namespace rclpy
{
namespace events_executor
{

EventsQueue::~EventsQueue() {}

void EventsQueue::Enqueue(std::function<void()> event_handler)
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push(event_handler);
  }
  cv_.notify_one();
}

void EventsQueue::Run() {RunUntil(std::chrono::steady_clock::time_point::max());}

void EventsQueue::RunUntil(std::chrono::steady_clock::time_point deadline)
{
  while (true) {
    std::function<void()> handler;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait_until(lock, deadline, [this]() {return stopped_ || !queue_.empty();});
      if (stopped_ || queue_.empty()) {
        // We stopped for some reason other than being ready to run something (stopped or timeout)
        return;
      }
      handler = queue_.front();
      queue_.pop();
    }
    handler();
  }
}

void EventsQueue::Stop()
{
  std::unique_lock<std::mutex> lock(mutex_);
  stopped_ = true;
  cv_.notify_one();
}

void EventsQueue::Restart()
{
  std::unique_lock<std::mutex> lock(mutex_);
  stopped_ = false;
}

}  // namespace events_executor
}  // namespace rclpy
