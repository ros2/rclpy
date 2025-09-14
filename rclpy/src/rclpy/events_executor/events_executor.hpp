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

#ifndef RCLPY__EVENTS_EXECUTOR__EVENTS_EXECUTOR_HPP_
#define RCLPY__EVENTS_EXECUTOR__EVENTS_EXECUTOR_HPP_

#include <pybind11/pybind11.h>

#include <rcl/client.h>
#include <rcl/service.h>
#include <rcl/subscription.h>
#include <rcl/timer.h>
#include <rcl/wait.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "events_executor/events_queue.hpp"
#include "events_executor/rcl_support.hpp"
#include "events_executor/scoped_with.hpp"
#include "events_executor/timers_manager.hpp"
#include "signal_handler.hpp"
#include "wait_set.hpp"

namespace rclpy
{
namespace events_executor
{

/// Events executor implementation for rclpy
///
/// This executor implementation attempts to replicate the function of the rclcpp EventsExecutor
/// for the benefit of rclpy applications.  It is implemented in C++ to minimize the overhead of
/// processing the event loop.
///
/// We assume all public methods could be invoked from any thread.  Callbacks on the executor loop
/// will be issued on the thread that called one of the spin*() variants (ignoring any parallelism
/// that might be allowed by the callback group configuration).
class EventsExecutor
{
public:
  /// @param context the rclpy Context object to operate on
  explicit EventsExecutor(pybind11::object context);

  ~EventsExecutor();

  // rclpy Executor API methods:
  pybind11::object get_context() const {return rclpy_context_;}
  pybind11::object create_task(
    pybind11::object callback, pybind11::args args = {}, const pybind11::kwargs & kwargs = {});
  pybind11::object create_future();
  bool shutdown(std::optional<double> timeout_sec = {});
  bool add_node(pybind11::object node);
  void remove_node(pybind11::handle node);
  void wake();
  pybind11::list get_nodes() const;
  void spin(std::optional<double> timeout_sec = {}, bool stop_after_user_callback = false);
  void spin_until_future_complete(
    pybind11::handle future, std::optional<double> timeout_sec = {},
    bool stop_after_user_callback = false);
  EventsExecutor * enter();
  void exit(pybind11::object, pybind11::object, pybind11::object);

private:
  // Structure to hold entities discovered underlying a Waitable object.
  struct WaitableSubEntities
  {
    std::vector<const rcl_subscription_t *> subscriptions;
    std::vector<rcl_timer_t *> timers;  // Can't be const
    std::vector<const rcl_client_t *> clients;
    std::vector<const rcl_service_t *> services;
    std::vector<const rcl_event_t *> events;
  };

  /// Updates the sets of known entities based on the currently tracked nodes.  This is not thread
  /// safe, so it must be posted to the EventsQueue if the executor is currently spinning.  Expects
  /// the GIL to be held before calling.  If @p shutdown is true, a purge of all known nodes and
  /// entities is forced.
  void UpdateEntitiesFromNodes(bool shutdown);

  /// Given an existing set of entities and a set with the desired new state, updates the existing
  /// set and invokes callbacks on each added or removed entity.
  void UpdateEntitySet(
    pybind11::set & entity_set, const pybind11::set & new_entity_set,
    std::function<void(pybind11::handle)> added_entity_callback,
    std::function<void(pybind11::handle)> removed_entity_callback);

  void HandleAddedSubscription(pybind11::handle);
  void HandleRemovedSubscription(pybind11::handle);
  void HandleSubscriptionReady(pybind11::handle, size_t number_of_events);

  void HandleAddedTimer(pybind11::handle);
  void HandleRemovedTimer(pybind11::handle);
  void HandleTimerReady(pybind11::handle);

  void HandleAddedClient(pybind11::handle);
  void HandleRemovedClient(pybind11::handle);
  void HandleClientReady(pybind11::handle, size_t number_of_events);

  void HandleAddedService(pybind11::handle);
  void HandleRemovedService(pybind11::handle);
  void HandleServiceReady(pybind11::handle, size_t number_of_events);

  void HandleAddedWaitable(pybind11::handle);
  void HandleRemovedWaitable(pybind11::handle);
  void HandleWaitableSubReady(
    pybind11::handle waitable, const rcl_subscription_t *,
    std::shared_ptr<rclpy::WaitSet> wait_set, size_t wait_set_sub_index,
    std::shared_ptr<ScopedWith> with_waitset, size_t number_of_events);
  void HandleWaitableTimerReady(
    pybind11::handle waitable, const rcl_timer_t *, std::shared_ptr<rclpy::WaitSet> wait_set,
    size_t wait_set_timer_index, std::shared_ptr<ScopedWith> with_waitable,
    std::shared_ptr<ScopedWith> with_waitset);
  void HandleWaitableClientReady(
    pybind11::handle waitable, const rcl_client_t *, std::shared_ptr<rclpy::WaitSet> wait_set,
    size_t wait_set_client_index, std::shared_ptr<ScopedWith> with_waitset,
    size_t number_of_events);
  void HandleWaitableServiceReady(
    pybind11::handle waitable, const rcl_service_t *, std::shared_ptr<rclpy::WaitSet> wait_set,
    size_t wait_set_service_index, std::shared_ptr<ScopedWith> with_waitset,
    size_t number_of_events);
  void HandleWaitableEventReady(
    pybind11::handle waitable, const rcl_event_t *, std::shared_ptr<rclpy::WaitSet> wait_set,
    size_t wait_set_event_index, std::shared_ptr<ScopedWith> with_waitset,
    size_t number_of_events);
  void HandleWaitableReady(
    pybind11::handle waitable, std::shared_ptr<rclpy::WaitSet> wait_set, size_t number_of_events);

  /// Helper for create_task().  @p task needs to have had one reference manually added to it.  See
  /// create_task() implementation for details.
  void IterateTask(pybind11::handle task);

  /// Posts a call to IterateTask() for every outstanding entry in tasks_; should be invoked from
  /// other Handle*Ready() methods to check if any asynchronous Tasks have been unblocked by the
  /// newly-handled event.
  void PostOutstandingTasks();

  void HandleCallbackExceptionInNodeEntity(
    const pybind11::error_already_set &, pybind11::handle entity,
    const std::string & node_entity_attr);
  void HandleCallbackExceptionWithLogger(
    const pybind11::error_already_set &, pybind11::object logger, const std::string & entity_type);

  /// Raises the given python object instance as a Python exception
  void Raise(pybind11::object);

  const pybind11::object rclpy_context_;

  // Imported python objects we depend on
  const pybind11::object inspect_iscoroutine_;
  const pybind11::object inspect_signature_;
  const pybind11::object rclpy_task_;
  const pybind11::object rclpy_future_;

  EventsQueue events_queue_;
  ScopedSignalCallback signal_callback_;

  pybind11::set nodes_;                ///< The set of all nodes we're executing
  std::atomic<bool> wake_pending_{};   ///< An unhandled call to wake() has been made
  std::timed_mutex spinning_mutex_;    ///< Held while a thread is spinning

  /// This flag is used by spin_once() to signal that the EventsQueue should be stopped after a
  /// single user-visible callback has been dispatched.
  bool stop_after_user_callback_{};

  // Collection of awaitable entities we're servicing
  pybind11::set subscriptions_;
  pybind11::set timers_;
  pybind11::set clients_;
  pybind11::set services_;
  pybind11::set waitables_;

  /// Collection of asynchronous Tasks awaiting new events to further iterate.
  std::vector<pybind11::handle> blocked_tasks_;

  /// Cache for rcl pointers underlying each waitables_ entry, because those are harder to retrieve
  /// than the other entity types.
  std::unordered_map<pybind11::handle, WaitableSubEntities, PythonHasher> waitable_entities_;

  RclCallbackManager rcl_callback_manager_;
  TimersManager timers_manager_;
};

void define_events_executor(pybind11::object module);

}  // namespace events_executor
}  // namespace rclpy

#endif  // RCLPY__EVENTS_EXECUTOR__EVENTS_EXECUTOR_HPP_
