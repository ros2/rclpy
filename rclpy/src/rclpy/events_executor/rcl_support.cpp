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
#include "events_executor/rcl_support.hpp"

#include <utility>

namespace py = pybind11;

namespace rclpy
{
namespace events_executor
{

extern "C" void RclEventCallbackTrampoline(const void * user_data, size_t number_of_events)
{
  const auto cb = reinterpret_cast<const std::function<void(size_t)> *>(user_data);
  (*cb)(number_of_events);
}

RclCallbackManager::RclCallbackManager(EventsQueue * events_queue)
: events_queue_(events_queue) {}

RclCallbackManager::~RclCallbackManager()
{
  // Should not still have any callbacks registered when we exit, because otherwise RCL can call
  // pointers that will no longer be valid.  We can't throw an exception here, but we can explode.
  if (!owned_cbs_.empty()) {
    py::gil_scoped_acquire gil_acquire;
    py::print("Destroying callback manager with callbacks remaining");
    ::abort();
  }
}

const void * RclCallbackManager::MakeCallback(
  const void * key, std::function<void(size_t)> callback, std::shared_ptr<ScopedWith> with)
{
  // We don't support replacing an existing callback with a new one, because it gets tricky making
  // sure we don't delete an old callback while the middleware still holds a pointer to it.
  if (owned_cbs_.count(key) != 0) {
    throw py::key_error("Attempt to replace existing callback");
  }
  CbEntry new_entry;
  new_entry.cb =
    std::make_unique<std::function<void(size_t)>>([this, callback, key](size_t number_of_events) {
        events_queue_->Enqueue([this, callback, key, number_of_events]() {
          if (!owned_cbs_.count(key)) {
            // This callback has been removed, just drop it as the objects it may want to touch may
            // no longer exist.
            return;
          }
          callback(number_of_events);
      });
    });
  new_entry.with = with;
  const void * ret = new_entry.cb.get();
  owned_cbs_[key] = std::move(new_entry);
  return ret;
}

void RclCallbackManager::RemoveCallback(const void * key)
{
  if (!owned_cbs_.erase(key)) {
    throw py::key_error("Attempt to remove nonexistent callback");
  }
}

}  // namespace events_executor
}  // namespace rclpy
