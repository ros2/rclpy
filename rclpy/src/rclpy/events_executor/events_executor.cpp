// Copyright 2024-2025 Brad Martin
// Copyright 2024 Merlin Labs, Inc.
//
// Based on a similar approach as the iRobot rclcpp EventsExecutor implementation:
// https://github.com/ros2/rclcpp/blob/7907b2fee0b1381dc21900efd1745e11f5caa670/rclcpp/src/rclcpp/experimental/executors/events_executor/events_executor.cpp
// Original copyright for that file is:
// Copyright 2023 iRobot Corporation.
//
// Also borrows some code from the original rclpy Executor implementations:
// https://github.com/ros2/rclpy/blob/06d78fb28a6d61ede793201ae75474f3e5432b47/rclpy/rclpy/executors.py
// Original copyright for that file is:
// Copyright 2017 Open Source Robotics Foundation, Inc.
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
#include "events_executor/events_executor.hpp"

#include <pybind11/eval.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <Python.h>

#include <rcl/error_handling.h>

#include <chrono>
#include <utility>

#include "client.hpp"
#include "context.hpp"
#include "service.hpp"
#include "subscription.hpp"

namespace pl = std::placeholders;
namespace py = pybind11;

namespace rclpy
{
namespace events_executor
{

EventsExecutor::EventsExecutor(py::object context)
: rclpy_context_(context),
  inspect_iscoroutine_(py::module_::import("inspect").attr("iscoroutine")),
  inspect_signature_(py::module_::import("inspect").attr("signature")),
  rclpy_task_(py::module_::import("rclpy.task").attr("Task")),
  rclpy_future_(py::module_::import("rclpy.task").attr("Future")),
  signal_callback_([this]() {events_queue_.Stop();}),
  rcl_callback_manager_(&events_queue_),
  timers_manager_(
    &events_queue_, std::bind(&EventsExecutor::HandleTimerReady, this, pl::_1))
{
}

EventsExecutor::~EventsExecutor() {shutdown();}

pybind11::object EventsExecutor::create_task(
  py::object callback, py::args args, const py::kwargs & kwargs)
{
  // Create and return a rclpy.task.Task() object, and schedule it to be called later.
  using py::literals::operator""_a;
  py::object task = rclpy_task_(callback, args, kwargs, "executor"_a = py::cast(this));
  // The Task needs to be owned at least until we invoke it from the callback we post, however we
  // can't pass a bare py::object because that's going to try to do Python refcounting while
  // preparing to go into or coming back from the callback, while the GIL is not held.  We'll do
  // manual refcounting on it instead.
  py::handle cb_task_handle = task;
  cb_task_handle.inc_ref();
  events_queue_.Enqueue(std::bind(&EventsExecutor::IterateTask, this, cb_task_handle));
  return task;
}

pybind11::object EventsExecutor::create_future()
{
  using py::literals::operator""_a;
  return rclpy_future_("executor"_a = py::cast(this));
}

bool EventsExecutor::shutdown(std::optional<double> timeout)
{
  // NOTE: The rclpy context can invoke this with a lock on the context held.  Therefore we must
  // not try to go access that context during this method or we can deadlock.
  // https://github.com/ros2/rclpy/blob/06d78fb28a6d61ede793201ae75474f3e5432b47/rclpy/rclpy/context.py#L101-L103

  events_queue_.Stop();

  // Block until spinning is done, or timeout.  Release the GIL while we block though.
  {
    py::gil_scoped_release gil_release;
    std::unique_lock<std::timed_mutex> spin_lock(spinning_mutex_, std::defer_lock);
    if (timeout) {
      if (!spin_lock.try_lock_for(std::chrono::duration<double>(*timeout))) {
        return false;
      }
    } else {
      spin_lock.lock();
    }
  }

  // Tear down any callbacks we still have registered.
  for (py::handle node : py::list(nodes_)) {
    remove_node(node);
  }
  UpdateEntitiesFromNodes(true);
  return true;
}

bool EventsExecutor::add_node(py::object node)
{
  if (nodes_.contains(node)) {
    return false;
  }
  nodes_.add(node);
  // Caution, the Node executor setter method calls executor.add_node() again making this
  // reentrant.
  node.attr("executor") = py::cast(this);
  wake();
  return true;
}

void EventsExecutor::remove_node(py::handle node)
{
  if (!nodes_.contains(node)) {
    return;
  }
  // Why does pybind11 provide a C++ method for add() but not discard() or remove()?
  nodes_.attr("remove")(node);
  // Not sure why rclpy doesn't change the node.executor at this point
  wake();
}

void EventsExecutor::wake()
{
  if (!wake_pending_.exchange(true)) {
    // Update tracked entities.
    events_queue_.Enqueue([this]() {
        py::gil_scoped_acquire gil_acquire;
        UpdateEntitiesFromNodes(!py::cast<bool>(rclpy_context_.attr("ok")()));
    });
  }
}

py::list EventsExecutor::get_nodes() const {return nodes_;}

// NOTE: The timeouts on the below two methods are always realtime even if we're running in debug
// time.  This is true of other executors too, because debug time is always associated with a
// specific node and more than one node may be connected to an executor instance.
// https://github.com/ros2/rclpy/blob/06d78fb28a6d61ede793201ae75474f3e5432b47/rclpy/rclpy/executors.py#L184-L185

void EventsExecutor::spin(std::optional<double> timeout_sec, bool stop_after_user_callback)
{
  {
    std::unique_lock<std::timed_mutex> spin_lock(spinning_mutex_, std::try_to_lock);
    if (!spin_lock) {
      throw std::runtime_error("Attempt to spin an already-spinning Executor");
    }
    stop_after_user_callback_ = stop_after_user_callback;
    // Any blocked tasks may have become unblocked while we weren't looking.
    PostOutstandingTasks();
    // Release the GIL while we block.  Any callbacks on the events queue that want to touch Python
    // will need to reacquire it though.
    py::gil_scoped_release gil_release;
    if (timeout_sec) {
      const auto timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(*timeout_sec));
      const auto end = std::chrono::steady_clock::now() + timeout_ns;
      events_queue_.RunUntil(end);
    } else {
      events_queue_.Run();
    }
    events_queue_.Restart();
  }

  const bool ok = py::cast<bool>(rclpy_context_.attr("ok")());
  if (!ok) {
        Raise(py::module_::import("rclpy.executors").attr("ExternalShutdownException")());
  }
}

void EventsExecutor::spin_until_future_complete(
  py::handle future, std::optional<double> timeout_sec, bool stop_after_user_callback)
{
  py::cpp_function cb([this](py::handle) {events_queue_.Stop();});
  future.attr("add_done_callback")(cb);
  spin(timeout_sec, stop_after_user_callback);
  // In case the future didn't complete (we hit the timeout or dispatched a different user callback
  // after being asked to only run one), we need to clean up our callback; otherwise, it could fire
  // later when the executor isn't valid, or we haven't been asked to wait for this future; also,
  // we could end up adding a bunch more of these same callbacks if this method gets invoked in a
  // loop.
  future.attr("remove_done_callback")(cb);
}

EventsExecutor * EventsExecutor::enter() {return this;}
void EventsExecutor::exit(py::object, py::object, py::object) {shutdown();}

void EventsExecutor::UpdateEntitiesFromNodes(bool shutdown)
{
  // Clear pending flag as early as possible, so we error on the side of retriggering a few
  // harmless updates rather than potentially missing important additions.
  wake_pending_.store(false);

  // Collect all entities currently associated with our nodes
  py::set subscriptions;
  py::set timers;
  py::set clients;
  py::set services;
  py::set waitables;
  if (!shutdown) {
    for (py::handle node : nodes_) {
      subscriptions.attr("update")(py::set(node.attr("subscriptions")));
      timers.attr("update")(py::set(node.attr("timers")));
      clients.attr("update")(py::set(node.attr("clients")));
      services.attr("update")(py::set(node.attr("services")));
      waitables.attr("update")(py::set(node.attr("waitables")));

      // It doesn't seem to be possible to support guard conditions with a callback-based (as
      // opposed to waitset-based) API.  Fortunately we don't seem to need to.
      if (!py::set(node.attr("guards")).empty()) {
        throw std::runtime_error("Guard conditions not supported");
      }
    }
  } else {
    // Remove all tracked entities and nodes.
    nodes_.clear();
  }

  // Perform updates for added and removed entities
  UpdateEntitySet(
    subscriptions_, subscriptions,
    std::bind(&EventsExecutor::HandleAddedSubscription, this, pl::_1),
    std::bind(&EventsExecutor::HandleRemovedSubscription, this, pl::_1));
  UpdateEntitySet(
    timers_, timers, std::bind(&EventsExecutor::HandleAddedTimer, this, pl::_1),
    std::bind(&EventsExecutor::HandleRemovedTimer, this, pl::_1));
  UpdateEntitySet(
    clients_, clients, std::bind(&EventsExecutor::HandleAddedClient, this, pl::_1),
    std::bind(&EventsExecutor::HandleRemovedClient, this, pl::_1));
  UpdateEntitySet(
    services_, services, std::bind(&EventsExecutor::HandleAddedService, this, pl::_1),
    std::bind(&EventsExecutor::HandleRemovedService, this, pl::_1));
  UpdateEntitySet(
    waitables_, waitables, std::bind(&EventsExecutor::HandleAddedWaitable, this, pl::_1),
    std::bind(&EventsExecutor::HandleRemovedWaitable, this, pl::_1));

  if (shutdown) {
    // Stop spinning after everything is torn down.
    events_queue_.Stop();
  }
}

void EventsExecutor::UpdateEntitySet(
  py::set & entity_set, const py::set & new_entity_set,
  std::function<void(py::handle)> added_entity_callback,
  std::function<void(py::handle)> removed_entity_callback)
{
  py::set added_entities = new_entity_set - entity_set;
  for (py::handle added_entity : added_entities) {
    added_entity_callback(added_entity);
  }

  py::set removed_entities = entity_set - new_entity_set;
  for (py::handle removed_entity : removed_entities) {
    removed_entity_callback(removed_entity);
  }

  entity_set = new_entity_set;
}

void EventsExecutor::HandleAddedSubscription(py::handle subscription)
{
  py::handle handle = subscription.attr("handle");
  auto with = std::make_shared<ScopedWith>(handle);
  const rcl_subscription_t * rcl_ptr = py::cast<const Subscription &>(handle).rcl_ptr();
  const auto cb = std::bind(&EventsExecutor::HandleSubscriptionReady, this, subscription, pl::_1);
  if (
    RCL_RET_OK != rcl_subscription_set_on_new_message_callback(
                    rcl_ptr, RclEventCallbackTrampoline,
                    rcl_callback_manager_.MakeCallback(rcl_ptr, cb, with)))
  {
    throw std::runtime_error(
      std::string("Failed to set the on new message callback for subscription: ") +
      rcl_get_error_string().str);
  }
}

void EventsExecutor::HandleRemovedSubscription(py::handle subscription)
{
  py::handle handle = subscription.attr("handle");
  const rcl_subscription_t * rcl_ptr = py::cast<const Subscription &>(handle).rcl_ptr();
  if (RCL_RET_OK != rcl_subscription_set_on_new_message_callback(rcl_ptr, nullptr, nullptr)) {
    throw std::runtime_error(
      std::string("Failed to clear the on new message callback for subscription: ") +
      rcl_get_error_string().str);
  }
  rcl_callback_manager_.RemoveCallback(rcl_ptr);
}

void EventsExecutor::HandleSubscriptionReady(py::handle subscription, size_t number_of_events)
{
  if (stop_after_user_callback_) {
    events_queue_.Stop();
  }
  py::gil_scoped_acquire gil_acquire;

  // Largely based on rclpy.Executor._take_subscription() and _execute_subcription().
  // https://github.com/ros2/rclpy/blob/06d78fb28a6d61ede793201ae75474f3e5432b47/rclpy/rclpy/executors.py#L355-L367
  //
  // NOTE: Simple object attributes we can count on to be owned by the parent object, but bound
  // method calls and function return values need to be owned by us.
  Subscription & _rclpy_sub = py::cast<Subscription &>(subscription.attr("handle"));
  const py::object msg_type = subscription.attr("msg_type");
  const bool raw = py::cast<bool>(subscription.attr("raw"));
  const int callback_type = py::cast<int>(subscription.attr("_callback_type").attr("value"));
  const int message_only =
    py::cast<int>(subscription.attr("CallbackType").attr("MessageOnly").attr("value"));
  const py::handle callback = subscription.attr("callback");

  // rmw_cyclonedds has a bug which causes number_of_events to be zero in the case where messages
  // were waiting for us when we registered the callback, and the topic is using KEEP_ALL history
  // policy.  We'll work around that by checking for zero and just taking messages until we start
  // getting None in that case.  https://github.com/ros2/rmw_cyclonedds/issues/509
  bool got_none = false;
  for (size_t i = 0; number_of_events ? i < number_of_events : !got_none; ++i) {
    py::object msg_info = _rclpy_sub.take_message(msg_type, raw);
    if (!msg_info.is_none()) {
      try {
        if (callback_type == message_only) {
          callback(py::cast<py::tuple>(msg_info)[0]);
        } else {
          callback(msg_info);
        }
      } catch (const py::error_already_set & e) {
        HandleCallbackExceptionInNodeEntity(e, subscription, "subscriptions");
        throw;
      }
    } else {
      got_none = true;
    }
  }

  PostOutstandingTasks();
}

void EventsExecutor::HandleAddedTimer(py::handle timer) {timers_manager_.AddTimer(timer);}

void EventsExecutor::HandleRemovedTimer(py::handle timer) {timers_manager_.RemoveTimer(timer);}

void EventsExecutor::HandleTimerReady(py::handle timer)
{
  py::gil_scoped_acquire gil_acquire;
  py::object callback = timer.attr("callback");
  py::object result;
  try {
    result = callback();
  } catch (const py::error_already_set & e) {
    HandleCallbackExceptionInNodeEntity(e, timer, "timers");
    throw;
  }

  // The type markup claims the callback can't be a coroutine, but this seems to be a lie because
  // the unit test does exactly that.
  if (py::cast<bool>(inspect_iscoroutine_(result))) {
    // Create a Task to manage iteration of this coroutine later.
    create_task(result);
  } else if (stop_after_user_callback_) {
    events_queue_.Stop();
  }
  PostOutstandingTasks();
}

void EventsExecutor::HandleAddedClient(py::handle client)
{
  py::handle handle = client.attr("handle");
  auto with = std::make_shared<ScopedWith>(handle);
  const rcl_client_t * rcl_ptr = py::cast<const Client &>(handle).rcl_ptr();
  const auto cb = std::bind(&EventsExecutor::HandleClientReady, this, client, pl::_1);
  if (
    RCL_RET_OK != rcl_client_set_on_new_response_callback(
                    rcl_ptr, RclEventCallbackTrampoline,
                    rcl_callback_manager_.MakeCallback(rcl_ptr, cb, with)))
  {
    throw std::runtime_error(
      std::string("Failed to set the on new response callback for client: ") +
      rcl_get_error_string().str);
  }
}

void EventsExecutor::HandleRemovedClient(py::handle client)
{
  py::handle handle = client.attr("handle");
  const rcl_client_t * rcl_ptr = py::cast<const Client &>(handle).rcl_ptr();
  if (RCL_RET_OK != rcl_client_set_on_new_response_callback(rcl_ptr, nullptr, nullptr)) {
    throw std::runtime_error(
      std::string("Failed to clear the on new response callback for client: ") +
      rcl_get_error_string().str);
  }
  rcl_callback_manager_.RemoveCallback(rcl_ptr);
}

void EventsExecutor::HandleClientReady(py::handle client, size_t number_of_events)
{
  if (stop_after_user_callback_) {
    events_queue_.Stop();
  }
  py::gil_scoped_acquire gil_acquire;

  // Largely based on rclpy.Executor._take_client() and _execute_client().
  // https://github.com/ros2/rclpy/blob/06d78fb28a6d61ede793201ae75474f3e5432b47/rclpy/rclpy/executors.py#L369-L384
  Client & _rclpy_client = py::cast<Client &>(client.attr("handle"));
  const py::handle srv_type = client.attr("srv_type");
  const py::object res_type = srv_type.attr("Response");
  const py::object get_pending_request = client.attr("get_pending_request");

  for (size_t i = 0; i < number_of_events; ++i) {
    py::tuple seq_and_response = _rclpy_client.take_response(res_type);
    py::handle header = seq_and_response[0];
    py::handle response = seq_and_response[1];
    if (!header.is_none()) {
      py::object sequence = header.attr("request_id").attr("sequence_number");
      py::object future;
      try {
        future = get_pending_request(sequence);
      } catch (const py::error_already_set & e) {
        if (e.matches(PyExc_KeyError)) {
          // The request was cancelled
          continue;
        }
        throw;
      }
      future.attr("_set_executor")(py::cast(this));
      try {
        future.attr("set_result")(response);
      } catch (const py::error_already_set & e) {
        HandleCallbackExceptionInNodeEntity(e, client, "clients");
        throw;
      }
    }
  }

  PostOutstandingTasks();
}

void EventsExecutor::HandleAddedService(py::handle service)
{
  py::handle handle = service.attr("handle");
  auto with = std::make_shared<ScopedWith>(handle);
  const rcl_service_t * rcl_ptr = py::cast<const Service &>(handle).rcl_ptr();
  const auto cb = std::bind(&EventsExecutor::HandleServiceReady, this, service, pl::_1);
  if (
    RCL_RET_OK != rcl_service_set_on_new_request_callback(
                    rcl_ptr, RclEventCallbackTrampoline,
                    rcl_callback_manager_.MakeCallback(rcl_ptr, cb, with)))
  {
    throw std::runtime_error(
      std::string("Failed to set the on new request callback for service: ") +
      rcl_get_error_string().str);
  }
}

void EventsExecutor::HandleRemovedService(py::handle service)
{
  py::handle handle = service.attr("handle");
  const rcl_service_t * rcl_ptr = py::cast<const Service &>(handle).rcl_ptr();
  if (RCL_RET_OK != rcl_service_set_on_new_request_callback(rcl_ptr, nullptr, nullptr)) {
    throw std::runtime_error(
      std::string("Failed to clear the on new request callback for service: ") +
      rcl_get_error_string().str);
  }
  rcl_callback_manager_.RemoveCallback(rcl_ptr);
}

void EventsExecutor::HandleServiceReady(py::handle service, size_t number_of_events)
{
  if (stop_after_user_callback_) {
    events_queue_.Stop();
  }
  py::gil_scoped_acquire gil_acquire;

  // Largely based on rclpy.Executor._take_service() and _execute_service().
  // https://github.com/ros2/rclpy/blob/06d78fb28a6d61ede793201ae75474f3e5432b47/rclpy/rclpy/executors.py#L386-L397
  Service & _rclpy_service = py::cast<Service &>(service.attr("handle"));
  const py::handle srv_type = service.attr("srv_type");
  const py::object req_type = srv_type.attr("Request");
  const py::handle res_type = srv_type.attr("Response");
  const py::handle callback = service.attr("callback");
  const py::object send_response = service.attr("send_response");

  for (size_t i = 0; i < number_of_events; ++i) {
    py::tuple request_and_header = _rclpy_service.service_take_request(req_type);
    py::handle request = request_and_header[0];
    py::handle header = request_and_header[1];
    if (!request.is_none()) {
      py::object response;
      try {
        response = callback(request, res_type());
      } catch (const py::error_already_set & e) {
        HandleCallbackExceptionInNodeEntity(e, service, "services");
        throw;
      }
      send_response(response, header);
    }
  }

  PostOutstandingTasks();
}

void EventsExecutor::HandleAddedWaitable(py::handle waitable)
{
  // The Waitable API is too abstract for us to work with directly; it only exposes APIs for
  // dealing with wait sets, and all of the rcl callback API requires knowing exactly what kinds of
  // rcl objects you're working with.  We'll try to figure out what kind of stuff is hiding behind
  // the abstraction by having the Waitable add itself to a wait set, then take stock of what all
  // ended up there.  We'll also have to hope that no Waitable implementations ever change their
  // component entities over their lifetimes.
  auto with_waitable = std::make_shared<ScopedWith>(waitable);
  const py::object num_entities = waitable.attr("get_num_entities")();
  if (py::cast<size_t>(num_entities.attr("num_guard_conditions")) != 0) {
    throw std::runtime_error("Guard conditions not supported");
  }
  auto wait_set = std::make_shared<WaitSet>(
    py::cast<size_t>(num_entities.attr("num_subscriptions")), 0U,
    py::cast<size_t>(num_entities.attr("num_timers")),
    py::cast<size_t>(num_entities.attr("num_clients")),
    py::cast<size_t>(num_entities.attr("num_services")),
    py::cast<size_t>(num_entities.attr("num_events")),
    py::cast<Context &>(rclpy_context_.attr("handle")));
  auto with_waitset = std::make_shared<ScopedWith>(py::cast(wait_set));
  waitable.attr("add_to_wait_set")(wait_set);
  rcl_wait_set_t * const rcl_waitset = wait_set->rcl_ptr();
  // We null out each entry in the waitset as we set it up, so that the waitset itself can be
  // reused when something becomes ready to signal to the Waitable what's ready and what's not.  We
  // also bind with_waitset into each callback we set up, to ensure that object doesn't get
  // destroyed while any of these callbacks are still registered.
  WaitableSubEntities sub_entities;
  for (size_t i = 0; i < rcl_waitset->size_of_subscriptions; ++i) {
    const rcl_subscription_t * const rcl_sub = rcl_waitset->subscriptions[i];
    rcl_waitset->subscriptions[i] = nullptr;
    sub_entities.subscriptions.push_back(rcl_sub);
    const auto cb = std::bind(
      &EventsExecutor::HandleWaitableSubReady, this, waitable, rcl_sub, wait_set, i, with_waitset,
      pl::_1);
    if (
      RCL_RET_OK != rcl_subscription_set_on_new_message_callback(
                      rcl_sub, RclEventCallbackTrampoline,
                      rcl_callback_manager_.MakeCallback(rcl_sub, cb, with_waitable)))
    {
      throw std::runtime_error(
        std::string("Failed to set the on new message callback for Waitable subscription: ") +
        rcl_get_error_string().str);
    }
  }
  for (size_t i = 0; i < rcl_waitset->size_of_timers; ++i) {
    // Unfortunately we do require a non-const pointer here, while the waitset structure contains a
    // const pointer.
    rcl_timer_t * const rcl_timer = const_cast<rcl_timer_t *>(rcl_waitset->timers[i]);
    rcl_waitset->timers[i] = nullptr;
    sub_entities.timers.push_back(rcl_timer);
    // Since this callback doesn't go through RclCallbackManager which would otherwise own an
    // instance of `with_waitable` associated with this callback, we'll bind it directly into the
    // callback instead.
    const auto cb = std::bind(
      &EventsExecutor::HandleWaitableTimerReady, this, waitable, rcl_timer, wait_set, i,
      with_waitable, with_waitset);
    timers_manager_.rcl_manager().AddTimer(rcl_timer, cb);
  }
  for (size_t i = 0; i < rcl_waitset->size_of_clients; ++i) {
    const rcl_client_t * const rcl_client = rcl_waitset->clients[i];
    rcl_waitset->clients[i] = nullptr;
    sub_entities.clients.push_back(rcl_client);
    const auto cb = std::bind(
      &EventsExecutor::HandleWaitableClientReady, this, waitable, rcl_client, wait_set, i,
      with_waitset, pl::_1);
    if (
      RCL_RET_OK != rcl_client_set_on_new_response_callback(
                      rcl_client, RclEventCallbackTrampoline,
                      rcl_callback_manager_.MakeCallback(rcl_client, cb, with_waitable)))
    {
      throw std::runtime_error(
        std::string("Failed to set the on new response callback for Waitable client: ") +
        rcl_get_error_string().str);
    }
  }
  for (size_t i = 0; i < rcl_waitset->size_of_services; ++i) {
    const rcl_service_t * const rcl_service = rcl_waitset->services[i];
    rcl_waitset->services[i] = nullptr;
    sub_entities.services.push_back(rcl_service);
    const auto cb = std::bind(
      &EventsExecutor::HandleWaitableServiceReady, this, waitable, rcl_service, wait_set, i,
      with_waitset, pl::_1);
    if (
      RCL_RET_OK != rcl_service_set_on_new_request_callback(
                      rcl_service, RclEventCallbackTrampoline,
                      rcl_callback_manager_.MakeCallback(rcl_service, cb, with_waitable)))
    {
      throw std::runtime_error(
        std::string("Failed to set the on new request callback for Waitable service: ") +
        rcl_get_error_string().str);
    }
  }
  for (size_t i = 0; i < rcl_waitset->size_of_events; ++i) {
    const rcl_event_t * const rcl_event = rcl_waitset->events[i];
    rcl_waitset->events[i] = nullptr;
    sub_entities.events.push_back(rcl_event);
    const auto cb = std::bind(
      &EventsExecutor::HandleWaitableEventReady, this, waitable, rcl_event, wait_set, i,
      with_waitset, pl::_1);
    if (
      RCL_RET_OK != rcl_event_set_callback(
                      rcl_event, RclEventCallbackTrampoline,
                      rcl_callback_manager_.MakeCallback(rcl_event, cb, with_waitable)))
    {
      throw std::runtime_error(
        std::string("Failed to set the callback for Waitable event: ") +
        rcl_get_error_string().str);
    }
  }

  // Save the set of discovered sub-entities for later use during tear-down since we can't repeat
  // the wait set trick then, as the RCL context may already be destroyed at that point.
  waitable_entities_[waitable] = std::move(sub_entities);
}

void EventsExecutor::HandleRemovedWaitable(py::handle waitable)
{
  const auto nh = waitable_entities_.extract(waitable);
  if (!nh) {
    throw std::runtime_error("Couldn't find sub-entities entry for removed Waitable");
  }
  const WaitableSubEntities & sub_entities = nh.mapped();
  for (const rcl_subscription_t * const rcl_sub : sub_entities.subscriptions) {
    if (RCL_RET_OK != rcl_subscription_set_on_new_message_callback(rcl_sub, nullptr, nullptr)) {
      throw std::runtime_error(
        std::string("Failed to clear the on new message "
                    "callback for Waitable subscription: ") +
        rcl_get_error_string().str);
    }
    rcl_callback_manager_.RemoveCallback(rcl_sub);
  }
  for (rcl_timer_t * const rcl_timer : sub_entities.timers) {
    timers_manager_.rcl_manager().RemoveTimer(rcl_timer);
  }
  for (const rcl_client_t * const rcl_client : sub_entities.clients) {
    if (RCL_RET_OK != rcl_client_set_on_new_response_callback(rcl_client, nullptr, nullptr)) {
      throw std::runtime_error(
        std::string("Failed to clear the on new response "
                    "callback for Waitable client: ") +
        rcl_get_error_string().str);
    }
    rcl_callback_manager_.RemoveCallback(rcl_client);
  }
  for (const rcl_service_t * const rcl_service : sub_entities.services) {
    if (RCL_RET_OK != rcl_service_set_on_new_request_callback(rcl_service, nullptr, nullptr)) {
      throw std::runtime_error(
        std::string("Failed to clear the on new request "
                    "callback for Waitable service: ") +
        rcl_get_error_string().str);
    }
    rcl_callback_manager_.RemoveCallback(rcl_service);
  }
  for (const rcl_event_t * const rcl_event : sub_entities.events) {
    if (RCL_RET_OK != rcl_event_set_callback(rcl_event, nullptr, nullptr)) {
      throw std::runtime_error(
        std::string("Failed to clear the callback for Waitable event: ") +
        rcl_get_error_string().str);
    }
    rcl_callback_manager_.RemoveCallback(rcl_event);
  }
}

void EventsExecutor::HandleWaitableSubReady(
  py::handle waitable, const rcl_subscription_t * rcl_sub, std::shared_ptr<WaitSet> wait_set,
  size_t wait_set_sub_index, std::shared_ptr<ScopedWith>, size_t number_of_events)
{
  py::gil_scoped_acquire gil_acquire;

  // We need to set up the wait set to make it look like our subscription object is ready, and then
  // poke the Waitable to do what it needs to do from there.
  rcl_wait_set_t * const rcl_waitset = wait_set->rcl_ptr();
  rcl_waitset->subscriptions[wait_set_sub_index] = rcl_sub;
  HandleWaitableReady(waitable, wait_set, number_of_events);
  // Null out the wait set again so that other callbacks can use it on other objects.
  rcl_waitset->subscriptions[wait_set_sub_index] = nullptr;
}

void EventsExecutor::HandleWaitableTimerReady(
  py::handle waitable, const rcl_timer_t * rcl_timer, std::shared_ptr<WaitSet> wait_set,
  size_t wait_set_timer_index, std::shared_ptr<ScopedWith>, std::shared_ptr<ScopedWith>)
{
  py::gil_scoped_acquire gil_acquire;

  // We need to set up the wait set to make it look like our timer object is ready, and then poke
  // the Waitable to do what it needs to do from there.
  rcl_wait_set_t * const rcl_waitset = wait_set->rcl_ptr();
  rcl_waitset->timers[wait_set_timer_index] = rcl_timer;
  HandleWaitableReady(waitable, wait_set, 1);
  // Null out the wait set again so that other callbacks can use it on other objects.
  rcl_waitset->timers[wait_set_timer_index] = nullptr;
}

void EventsExecutor::HandleWaitableClientReady(
  py::handle waitable, const rcl_client_t * rcl_client, std::shared_ptr<WaitSet> wait_set,
  size_t wait_set_client_index, std::shared_ptr<ScopedWith>, size_t number_of_events)
{
  py::gil_scoped_acquire gil_acquire;

  // We need to set up the wait set to make it look like our client object is ready, and then poke
  // the Waitable to do what it needs to do from there.
  rcl_wait_set_t * const rcl_waitset = wait_set->rcl_ptr();
  rcl_waitset->clients[wait_set_client_index] = rcl_client;
  HandleWaitableReady(waitable, wait_set, number_of_events);
  // Null out the wait set again so that other callbacks can use it on other objects.
  rcl_waitset->clients[wait_set_client_index] = nullptr;
}

void EventsExecutor::HandleWaitableServiceReady(
  py::handle waitable, const rcl_service_t * rcl_service, std::shared_ptr<WaitSet> wait_set,
  size_t wait_set_service_index, std::shared_ptr<ScopedWith>, size_t number_of_events)
{
  py::gil_scoped_acquire gil_acquire;

  // We need to set up the wait set to make it look like our service object is ready, and then poke
  // the Waitable to do what it needs to do from there.
  rcl_wait_set_t * const rcl_waitset = wait_set->rcl_ptr();
  rcl_waitset->services[wait_set_service_index] = rcl_service;
  HandleWaitableReady(waitable, wait_set, number_of_events);
  // Null out the wait set again so that other callbacks can use it on other objects.
  rcl_waitset->services[wait_set_service_index] = nullptr;
}

void EventsExecutor::HandleWaitableEventReady(
  py::handle waitable, const rcl_event_t * rcl_event, std::shared_ptr<WaitSet> wait_set,
  size_t wait_set_event_index, std::shared_ptr<ScopedWith>, size_t number_of_events)
{
  py::gil_scoped_acquire gil_acquire;

  // We need to set up the wait set to make it look like our event object is ready, and then poke
  // the Waitable to do what it needs to do from there.
  rcl_wait_set_t * const rcl_waitset = wait_set->rcl_ptr();
  rcl_waitset->events[wait_set_event_index] = rcl_event;
  HandleWaitableReady(waitable, wait_set, number_of_events);
  // Null out the wait set again so that other callbacks can use it on other objects.
  rcl_waitset->events[wait_set_event_index] = nullptr;
}

void EventsExecutor::HandleWaitableReady(
  py::handle waitable, std::shared_ptr<WaitSet> wait_set, size_t number_of_events)
{
  if (stop_after_user_callback_) {
    events_queue_.Stop();
  }
  // Largely based on rclpy.Executor._take_waitable()
  // https://github.com/ros2/rclpy/blob/a19180c238d4d97ed2b58868d8fb7fa3e3b621f2/rclpy/rclpy/executors.py#L447-L454
  py::object is_ready = waitable.attr("is_ready");
  py::object take_data = waitable.attr("take_data");
  py::object execute = waitable.attr("execute");
  py::object futures = waitable.attr("_futures");
  for (auto & future : futures) {
    future.attr("_set_executor")(py::cast(this));
  }
  for (size_t i = 0; i < number_of_events; ++i) {
    // This method can have side effects, so it needs to be called even though it looks like just
    // an accessor.
    if (!is_ready(wait_set)) {
      throw std::runtime_error("Failed to make Waitable ready");
    }
    py::object data = take_data();
    // execute() is an async method, we need a Task to run it
    create_task(execute(data));
  }

  PostOutstandingTasks();
}

void EventsExecutor::IterateTask(py::handle task)
{
  if (stop_after_user_callback_) {
    events_queue_.Stop();
  }
  py::gil_scoped_acquire gil_acquire;
  // Calling this won't throw, but it may set the exception property on the task object.
  task();
  if (py::cast<bool>(task.attr("done")())) {
    py::object ex = task.attr("exception")();
    // Drop reference with GIL held.  This doesn't necessarily destroy the underlying Task, since
    // the `create_task()` caller may have retained a reference to the returned value.
    task.dec_ref();

    if (!ex.is_none()) {
      // It's not clear how to easily turn a Python exception into a C++ one, so let's just throw
      // it again and let pybind translate it normally.
      try {
        Raise(ex);
      } catch (py::error_already_set & cpp_ex) {
        // There's no good way to know what node this task came from.  If we only have one node, we
        // can use the logger from that, otherwise we'll have to leave it undefined.
        py::object logger = py::none();
        if (nodes_.size() == 1) {
          logger = nodes_[0].attr("get_logger")();
        }
        HandleCallbackExceptionWithLogger(cpp_ex, logger, "task");
        throw;
      }
    }
  } else {
    // Task needs more iteration.  Store the handle and revisit it later after the next ready
    // entity which may unblock it.
    // TODO(bmartin427) This matches the behavior of SingleThreadedExecutor and avoids busy
    // looping, but I don't love it because if the task is waiting on something other than an rcl
    // entity (e.g. an asyncio sleep, or a Future triggered from another thread, or even another
    // Task), there can be arbitrarily long latency before some rcl activity causes us to go
    // revisit that Task.
    blocked_tasks_.push_back(task);
  }
}

void EventsExecutor::PostOutstandingTasks()
{
  for (auto & task : blocked_tasks_) {
    events_queue_.Enqueue(std::bind(&EventsExecutor::IterateTask, this, task));
  }
  // Clear the entire outstanding tasks list.  Any tasks that need further iteration will re-add
  // themselves during IterateTask().
  blocked_tasks_.clear();
}

void EventsExecutor::HandleCallbackExceptionInNodeEntity(
  const py::error_already_set & exc, py::handle entity, const std::string & node_entity_attr)
{
  // Try to identify the node associated with the entity that threw the exception, so we can log to
  // it.
  for (py::handle node : nodes_) {
    if (py::set(node.attr(node_entity_attr.c_str())).contains(entity)) {
      return HandleCallbackExceptionWithLogger(exc, node.attr("get_logger")(), node_entity_attr);
    }
  }

  // Failed to find a node
  HandleCallbackExceptionWithLogger(exc, py::none(), node_entity_attr);
}

void EventsExecutor::HandleCallbackExceptionWithLogger(
  const py::error_already_set & exc, py::object logger, const std::string & entity_type)
{
  if (logger.is_none()) {
    py::object logging = py::module_::import("rclpy.logging");
    logger = logging.attr("get_logger")("UNKNOWN");
  }

  // The logger API won't let you call it with two different severities, from what it considers the
  // same code location.  Since it has no visibility into C++, all calls made from here will be
  // attributed to the python that last called into here.  Instead we will call out to python for
  // logging.
  py::dict scope;
  scope["logger"] = logger;
  scope["node_entity_attr"] = entity_type;
  scope["exc_value"] = exc.value();
  scope["exc_trace"] = exc.trace();
  py::exec(
    R"(
import traceback
logger.fatal(f"Exception in '{node_entity_attr}' callback: {exc_value}")
logger.warn("Error occurred at:\n" + "".join(traceback.format_tb(exc_trace)))
)",
    scope);
}

void EventsExecutor::Raise(py::object ex)
{
  py::dict scope;
  scope["ex"] = ex;
  py::exec("raise ex", scope);
}

// pybind11 module bindings

void define_events_executor(py::object module)
{
  py::class_<EventsExecutor>(module, "EventsExecutor")
  .def(py::init<py::object>(), py::arg("context"))
  .def_property_readonly("context", &EventsExecutor::get_context)
  .def("create_task", &EventsExecutor::create_task, py::arg("callback"))
  .def("create_future", &EventsExecutor::create_future)
  .def("shutdown", &EventsExecutor::shutdown, py::arg("timeout_sec") = py::none())
  .def("add_node", &EventsExecutor::add_node, py::arg("node"))
  .def("remove_node", &EventsExecutor::remove_node, py::arg("node"))
  .def("wake", &EventsExecutor::wake)
  .def("get_nodes", &EventsExecutor::get_nodes)
  .def("spin", [](EventsExecutor & exec) {exec.spin();})
  .def(
      "spin_once",
    [](EventsExecutor & exec, std::optional<double> timeout_sec) {
      exec.spin(timeout_sec, true);
      },
      py::arg("timeout_sec") = py::none())
  .def(
      "spin_until_future_complete",
    [](EventsExecutor & exec, py::handle future, std::optional<double> timeout_sec) {
      exec.spin_until_future_complete(future, timeout_sec);
      },
      py::arg("future"), py::arg("timeout_sec") = py::none())
  .def(
      "spin_once_until_future_complete",
    [](EventsExecutor & exec, py::handle future, std::optional<double> timeout_sec) {
      exec.spin_until_future_complete(future, timeout_sec, true);
      },
      py::arg("future"), py::arg("timeout_sec") = py::none())
  .def("__enter__", &EventsExecutor::enter)
  .def("__exit__", &EventsExecutor::exit);
}

}  // namespace events_executor
}  // namespace rclpy
