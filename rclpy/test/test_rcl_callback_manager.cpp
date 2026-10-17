// Copyright 2026 Nathan C Hui
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

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>

#include "events_executor/rcl_support.hpp"

using rclpy::events_executor::EventsQueue;
using rclpy::events_executor::RclCallbackManager;
using rclpy::events_executor::RclEventCallbackTrampoline;

TEST(RclCallbackManager, DispatchesRegisteredCallback)
{
  EventsQueue queue;
  RclCallbackManager manager(&queue);
  int key;
  size_t received_events = 0;
  const auto user_data = manager.MakeCallback(
    &key, [&received_events](size_t count) {received_events += count;}, nullptr);
  RclEventCallbackTrampoline(user_data, 3);
  RclEventCallbackTrampoline(user_data, 2);
  EXPECT_EQ(received_events, 0u);
  queue.Enqueue([&queue]() {queue.Stop();});
  queue.Run();
  EXPECT_EQ(received_events, 5u);
  manager.RemoveCallback(&key);
}

TEST(RclCallbackManager, RemovalReleasesCapturesBeforeStaleEventDispatch)
{
  EventsQueue queue;
  RclCallbackManager manager(&queue);
  int key;
  auto capture = std::make_shared<int>(0);
  std::weak_ptr<int> weak_capture = capture;
  bool called = false;
  const auto user_data = manager.MakeCallback(
    &key, [capture, &called](size_t) {called = true;}, nullptr);
  capture.reset();
  RclEventCallbackTrampoline(user_data, 1);
  manager.RemoveCallback(&key);
  EXPECT_TRUE(weak_capture.expired());
  queue.Enqueue([&queue]() {queue.Stop();});
  queue.Run();
  EXPECT_FALSE(called);
}

TEST(RclCallbackManager, RemovalReleasesCapturesBeforeQueueDestruction)
{
  EventsQueue queue;
  RclCallbackManager manager(&queue);
  int key;
  auto capture = std::make_shared<int>(0);
  std::weak_ptr<int> weak_capture = capture;
  const auto user_data = manager.MakeCallback(
    &key, [capture](size_t) {}, nullptr);
  capture.reset();
  RclEventCallbackTrampoline(user_data, 1);
  manager.RemoveCallback(&key);
  EXPECT_TRUE(weak_capture.expired());
}
