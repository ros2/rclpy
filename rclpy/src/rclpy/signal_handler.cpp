// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <pybind11/pybind11.h>

namespace py = pybind11;

#include <csignal>

#include <atomic>

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

#include "rcutils/allocator.h"


#include "guard_condition.hpp"

#if __APPLE__ || _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _POSIX_SOURCE

#define SIGNAL_HANDLER_T struct sigaction

static SIGNAL_HANDLER_T
make_null_signal_handler()
{
  SIGNAL_HANDLER_T handler;
  memset(&handler, 0, sizeof(handler));
  return handler;
}

#define NULL_SIGNAL_HANDLER make_null_signal_handler()

#define DEFINE_SIGNAL_HANDLER(name) \
  static void _ ## name(int signum, siginfo_t * info, void * context); \
  static SIGNAL_HANDLER_T get_ ## name() { \
    SIGNAL_HANDLER_T handler; \
    memset(&handler, 0, sizeof(handler)); \
    sigemptyset(&handler.sa_mask); \
    handler.sa_sigaction = _ ## name; \
    handler.sa_flags = SA_SIGINFO; \
    return handler; \
  } \
  static bool is_ ## name(SIGNAL_HANDLER_T handler) { \
    return _ ## name == handler.sa_sigaction; \
  } \
  static void _ ## name(int signum, siginfo_t * info, void * context)

#define SIGNAL_HANDLER_ARGS signum, info, context

static void
call_signal_handler(
  SIGNAL_HANDLER_T handler, int signum,
  siginfo_t * siginfo, void * context)
{
  if (handler.sa_flags & SA_SIGINFO) {
    if (handler.sa_sigaction != NULL) {
      handler.sa_sigaction(signum, siginfo, context);
    }
  } else {
    if (handler.sa_handler != NULL &&    // Is set
      handler.sa_handler != SIG_DFL &&      // Is not default
      handler.sa_handler != SIG_IGN)      // Is not ignored
    {
      handler.sa_handler(signum);
    }
  }
}

static SIGNAL_HANDLER_T
install_signal_handler(int signum, SIGNAL_HANDLER_T handler)
{
  SIGNAL_HANDLER_T old_handler;
  sigaction(signum, &handler, &old_handler);
  return old_handler;
}

#define is_null_signal_handler(handler) \
  (handler.sa_flags & SA_SIGINFO ? \
  !handler.sa_sigaction : \
  !handler.sa_handler)

#else

#ifdef _WIN32
#define SIGNAL_HANDLER_T _crt_signal_t
#else
#define SIGNAL_HANDLER_T sig_t
#endif  // _WIN32

#define NULL_SIGNAL_HANDLER NULL

#define DEFINE_SIGNAL_HANDLER(name) \
  static void _ ## name(int signum); \
  static SIGNAL_HANDLER_T get_ ## name() {return _ ## name;} \
  static bool is_ ## name(SIGNAL_HANDLER_T handler) { \
    return _ ## name == handler; \
  } \
  static void _ ## name(int signum)

#define SIGNAL_HANDLER_ARGS signum

#define call_signal_handler(handler, ...) handler(__VA_ARGS__)

#define install_signal_handler(signum, handler) signal(signum, handler)

#define is_null_signal_handler(signal_handler) (NULL_SIGNAL_HANDLER == signal_handler)

#endif

// Forward declarations
static bool trigger_guard_conditions();
static void unregister_sigint_signal_handler();

/// Original signal handler for chaining purposes
SIGNAL_HANDLER_T g_original_sigint_handler;

/// Signal handler function
DEFINE_SIGNAL_HANDLER(rclpy_sigint_handler)
{
  if (!is_null_signal_handler(g_original_sigint_handler)) {
    call_signal_handler(g_original_sigint_handler, SIGNAL_HANDLER_ARGS);
  }

  if (!trigger_guard_conditions()) {
    // There may have been another signal handler chaining to this
    // one when we last tried to unregister ourselves.

    // Try to unregister again.
    unregister_sigint_signal_handler();
  }
}

/// Unregister our SIGINT handler and restore the original one
static void
unregister_sigint_signal_handler()
{
  const SIGNAL_HANDLER_T current_sigint_handler =
    install_signal_handler(SIGINT, g_original_sigint_handler);
  if (!is_rclpy_sigint_handler(current_sigint_handler)) {
    // Oops, someone else must have registered a signal handler
    // that chains to us put it back so it continues to work
    install_signal_handler(SIGINT, current_sigint_handler);
    return;
  }
  // Got ourself out of the chain
  g_original_sigint_handler = NULL_SIGNAL_HANDLER;
}

/// Register our SIGINT handler and store the current one
static void
register_sigint_signal_handler()
{
  if (!is_null_signal_handler(g_original_sigint_handler)) {
    // Handler already registered
    return;
  }
  g_original_sigint_handler =
    install_signal_handler(SIGINT, get_rclpy_sigint_handler());
}

/// Global reference to guard conditions
/// End with sentinel value instead of count to avoid mismatch if signal
/// interrupts while adding or removing from the list
std::atomic<rcl_guard_condition_t **> g_guard_conditions;

/// Trigger all registered guard conditions
/**
 * This triggers guard conditions when a signal is received.
 * These wake executors currently blocked in `rcl_wait`.
 * Returns True if at least one guard condition was triggered.
 */
static bool
trigger_guard_conditions()
{
  rcl_guard_condition_t ** guard_conditions = g_guard_conditions.load();
  if (!guard_conditions || !guard_conditions[0]) {
    return false;
  }

  rcl_guard_condition_t ** pgc = guard_conditions;
  // Trigger python signal handlers
  while (NULL != *pgc) {
    rcl_ret_t ret = rcl_trigger_guard_condition(*pgc);
    if (ret != RCL_RET_OK) {
      // TODO(sloretz) find signal safe way to tell the world an error occurred
      rcl_reset_error();
    }
    ++pgc;
  }
  return true;
}

namespace rclpy
{
/// Warn if getting g_guard_conditions could deadlock the signal handler
void
check_signal_safety()
{
  static bool did_warn = false;
  if (!did_warn && !g_guard_conditions.is_lock_free()) {
    did_warn = true;
    const char * deadlock_msg =
      "Global guard condition list access is not lock-free on this platform."
      "The program may deadlock when receiving SIGINT.";
    if (PyErr_WarnEx(PyExc_ResourceWarning, deadlock_msg, 1)) {
      throw py::error_already_set();
    }
  }
}

/// Register a guard condition to be triggered when SIGINT is received.
/**
 * Raises ValueError if the argument was already registered
 *
 * \param[in] guard_condition a guard condition
 * \return None
 */
void
register_sigint_guard_condition(const GuardCondition & guard_condition)
{
  check_signal_safety();

  rcl_guard_condition_t * gc = guard_condition.rcl_ptr();
  rcl_guard_condition_t ** guard_conditions = g_guard_conditions.load();

  // Figure out how big the list currently is
  size_t count_gcs = 0;
  if (NULL != guard_conditions) {
    while (NULL != guard_conditions[count_gcs]) {
      if (gc == guard_conditions[count_gcs]) {
        throw py::value_error("Guard condition was already registered");
      }
      ++count_gcs;
    }
  }

  // Current size of guard condition list: count_gcs + 1 (sentinel value)
  // Allocate space for one more guard condition: count_cs + 1 (new gc) + 1 (sentinel value)
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto new_gcs = static_cast<rcl_guard_condition_t **>(
    allocator.allocate(sizeof(rcl_guard_condition_t *) * (count_gcs + 2), allocator.state));

  // populate the new guard condition list, ending with a sentinel of NULL
  for (size_t i = 0; i < count_gcs; ++i) {
    new_gcs[i] = guard_conditions[i];
  }
  new_gcs[count_gcs] = gc;
  new_gcs[count_gcs + 1] = NULL;

  // Swap the lists and free the old
  rcl_guard_condition_t ** old_gcs = g_guard_conditions.exchange(new_gcs);
  if (NULL != old_gcs) {
    allocator.deallocate(old_gcs, allocator.state);
  }

  // make sure our signal handler is registered
  register_sigint_signal_handler();
}

/// Unregister a guard condition so it is not triggered when SIGINT is received.
/**
 * Raises ValueError if the argument was not registered
 *
 * \param[in] guard_condition a guard condition
 * \return None
 */
void
unregister_sigint_guard_condition(const GuardCondition & guard_condition)
{
  rcl_guard_condition_t * gc = guard_condition.rcl_ptr();
  rcl_guard_condition_t ** guard_conditions = g_guard_conditions.load();

  // Figure out how big the list currently is
  size_t count_gcs = 0;
  bool found_gc = false;
  // assumes guard condition was only added to list once
  size_t found_index = 0;

  if (NULL != guard_conditions) {
    while (NULL != guard_conditions[count_gcs]) {
      if (gc == guard_conditions[count_gcs]) {
        found_gc = true;
        found_index = count_gcs;
      }
      ++count_gcs;
    }
  }

  if (!found_gc) {
    throw py::value_error("Guard condition was not registered");
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();

  if (count_gcs == 1) {
    // Just delete the list if there are no guard conditions left
    rcl_guard_condition_t ** old_gcs = g_guard_conditions.exchange(NULL);
    allocator.deallocate(old_gcs, allocator.state);
    unregister_sigint_signal_handler();
  } else {
    // Create space for one less guard condition
    // current list size: count_gcs + 1 (sentinel)
    // new list size: count_gcs - 1 (removing a guard condition) + 1 (sentinel)
    rcl_allocator_t allocator = rcl_get_default_allocator();
    auto new_gcs = static_cast<rcl_guard_condition_t **>(
      allocator.allocate(sizeof(rcl_guard_condition_t *) * (count_gcs), allocator.state));

    // Put remaining guard conditions in the list, ending with a sentinel of NULL
    for (size_t i = 0; i < found_index; ++i) {
      new_gcs[i] = guard_conditions[i];
    }
    // one less guard condition
    --count_gcs;
    for (size_t i = found_index; i < count_gcs; ++i) {
      new_gcs[i] = guard_conditions[i + 1];
    }
    // Put sentinel at end
    new_gcs[count_gcs] = NULL;

    // Replace guard condition list
    rcl_guard_condition_t ** old_gcs = g_guard_conditions.exchange(new_gcs);
    allocator.deallocate(old_gcs, allocator.state);
  }
}

void
define_signal_handler_api(py::module m)
{
  g_original_sigint_handler = NULL_SIGNAL_HANDLER;

  m.def(
    "register_sigint_guard_condition", &rclpy::register_sigint_guard_condition,
    "Register a guard condition to be called on SIGINT.");
  m.def(
    "unregister_sigint_guard_condition", &rclpy::unregister_sigint_guard_condition,
    "Stop triggering a guard condition when SIGINT occurs.");
}
}  // namespace rclpy
