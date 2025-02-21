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

#ifndef RCLPY__EVENTS_EXECUTOR__RCL_SUPPORT_HPP_
#define RCLPY__EVENTS_EXECUTOR__RCL_SUPPORT_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <unordered_map>

#include "events_executor/events_queue.hpp"
#include "events_executor/scoped_with.hpp"

namespace rclpy
{
namespace events_executor
{

/// Use this for all RCL event callbacks.  Use the return value from
/// RclCallbackManager::MakeCallback() as the user data arg.
///
/// Note that RCL callbacks are invoked in some arbitrary thread originating from the middleware.
/// Callbacks should process quickly to avoid blocking the middleware; i.e.  all actual work should
/// be posted to an EventsQueue running in another thread.
extern "C" void RclEventCallbackTrampoline(const void * user_data, size_t number_of_events);

/// Creates and maintains callback wrappers used with the RCL C library.
class RclCallbackManager
{
public:
  /// All user callbacks will be posted on the @p events_queue given to the constructor.  This
  /// pointer is aliased and must live for the lifetime of this object.  These callbacks will be
  /// invoked without the Python Global Interpreter Lock held, so if they need to access Python at
  /// all make sure to acquire that explicitly.
  explicit RclCallbackManager(EventsQueue * events_queue);
  ~RclCallbackManager();

  /// Creates a callback wrapper to be passed to RCL C functions.  @p key should be a pointer to
  /// the rcl object that will be associated with the callback.  @p with protects the _rclpy object
  /// handle owning the RCL object, for the duration the callback is established.
  const void * MakeCallback(
    const void * key, std::function<void(size_t)> callback, std::shared_ptr<ScopedWith> with);

  /// Discard a previously constructed callback.  @p key should be the same value provided to
  /// MakeCallback().  Caution: ensure that RCL is no longer using a callback before invoking this.
  void RemoveCallback(const void * key);

private:
  /// The C RCL interface deals in raw pointers, so someone needs to own the C++ function objects
  /// we'll be calling into.  We use unique pointers so the raw pointer to the object remains
  /// stable while the map is manipulated.
  struct CbEntry
  {
    std::unique_ptr<std::function<void(size_t)>> cb;
    std::shared_ptr<ScopedWith> with;
  };

  EventsQueue * const events_queue_;

  /// The map key is the raw pointer to the RCL entity object (subscription, etc) associated with
  /// the callback.
  std::unordered_map<const void *, CbEntry> owned_cbs_;
};

}  // namespace events_executor
}  // namespace rclpy

#endif  // RCLPY__EVENTS_EXECUTOR__RCL_SUPPORT_HPP_
