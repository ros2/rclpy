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

#include "signal_handler.hpp"

#include <pybind11/pybind11.h>

#include <rcl/allocator.h>
#include <rcl/error_handling.h>
#include <rcl/guard_condition.h>
#include <rcutils/logging_macros.h>

#include <atomic>
#include <csignal>
#include <stdexcept>
#include <string>
#include <thread>

#include "context.hpp"
#include "guard_condition.hpp"

// includes for semaphore notification code
#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <dispatch/dispatch.h>
#else  // posix
#include <semaphore.h>
#endif


namespace py = pybind11;

static bool trigger_guard_conditions();

namespace
{
#if defined(_WIN32)
HANDLE g_signal_handler_sem;
#elif defined(__APPLE__)
dispatch_semaphore_t g_signal_handler_sem;
#else  // posix
sem_t g_signal_handler_sem;
#endif

std::thread g_deferred_signal_handling_thread;

// relying on python GIL for safety
std::atomic<bool> g_signal_handler_installed = false;

void
notify_signal_handler() noexcept
{
#if defined(_WIN32)
  if (!ReleaseSemaphore(g_signal_handler_sem, 1, NULL)) {
    return;
  }
#elif defined(__APPLE__)
  dispatch_semaphore_signal(g_signal_handler_sem);
#else  // posix
  if (-1 == sem_post(&g_signal_handler_sem)) {
    return;
  }
#endif
}

void
wait_for_signal()
{
#if defined(_WIN32)
  DWORD dw_wait_result = WaitForSingleObject(g_signal_handler_sem, INFINITE);
  switch (dw_wait_result) {
    case WAIT_ABANDONED:
      RCUTILS_LOG_ERROR_NAMED(
        "rclpy.signals",
        "WaitForSingleObject() failed in wait_for_signal() with WAIT_ABANDONED: %s",
        GetLastError());
      break;
    case WAIT_OBJECT_0:
      // successful
      break;
    case WAIT_TIMEOUT:
      RCUTILS_LOG_ERROR_NAMED(
        "rclpy.signals", "WaitForSingleObject() timedout out in wait_for_signal()");
      break;
    case WAIT_FAILED:
      RCUTILS_LOG_ERROR_NAMED(
        "rclpy.signals", "WaitForSingleObject() failed in wait_for_signal(): %s", GetLastError());
      break;
    default:
      RCUTILS_LOG_ERROR_NAMED(
        "rclpy.signals", "WaitForSingleObject() gave unknown return in wait_for_signal(): %s",
        GetLastError());
  }
#elif defined(__APPLE__)
  dispatch_semaphore_wait(g_signal_handler_sem, DISPATCH_TIME_FOREVER);
#else  // posix
  int s;
  do {
    s = sem_wait(&g_signal_handler_sem);
  } while (-1 == s && EINTR == errno);
#endif
}

void
setup_deferred_signal_handler()
{
  if (g_signal_handler_installed.exchange(true)) {
    return;
  }
#if defined(_WIN32)
  g_signal_handler_sem = CreateSemaphore(
    NULL,  // default security attributes
    0,  // initial semaphore count
    1,  // maximum semaphore count
    NULL);  // unnamed semaphore
  if (NULL == g_signal_handler_sem) {
    throw std::runtime_error("CreateSemaphore() failed in setup_wait_for_signal()");
  }
#elif defined(__APPLE__)
  g_signal_handler_sem = dispatch_semaphore_create(0);
#else  // posix
  if (-1 == sem_init(&g_signal_handler_sem, 0, 0)) {
    throw std::runtime_error(std::string("sem_init() failed: ") + strerror(errno));
  }
#endif
  g_deferred_signal_handling_thread = std::thread(
    []() {
      while (g_signal_handler_installed.load()) {
        wait_for_signal();
        if (g_signal_handler_installed.load()) {
          trigger_guard_conditions();
          rclpy::shutdown_contexts();
        }
      }
    });
}

void
teardown_deferred_signal_handler()
{
  if (!g_signal_handler_installed.exchange(false)) {
    return;
  }
  notify_signal_handler();
  g_deferred_signal_handling_thread.join();
#if defined(_WIN32)
  CloseHandle(g_signal_handler_sem);
#elif defined(__APPLE__)
  dispatch_release(g_signal_handler_sem);
#else  // posix
  if (-1 == sem_destroy(&g_signal_handler_sem)) {
    std::runtime_error{"invalid semaphore in teardown_wait_for_signal()"};
  }
#endif
}

struct CleanupdeferredSignalHandler
{
  ~CleanupdeferredSignalHandler()
  {
    // just in case nobody calls rclpy.shutdown()
    teardown_deferred_signal_handler();
  }
};

// TODO(ivanpauno): Create a singleton SignalHandler class instead of this mess.
CleanupdeferredSignalHandler cleanup_deferred_signal_handler;

}  // namespace

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

static void
call_signal_handler(
  SIGNAL_HANDLER_T handler, int signum)
{
  if (handler != NULL && handler != SIG_DFL && handler != SIG_IGN) {
    handler(signum);
  }
}

#define install_signal_handler(signum, handler) signal(signum, handler)

#define is_null_signal_handler(signal_handler) (NULL_SIGNAL_HANDLER == signal_handler)

#endif

// Forward declarations
static void unregister_sigint_signal_handler();
static void unregister_sigterm_signal_handler();

/// Original signal handler for chaining purposes
SIGNAL_HANDLER_T g_original_sigint_handler;

/// Original sigterm handler for chaining purposes
SIGNAL_HANDLER_T g_original_sigterm_handler;

/// Flag to indicate if a sigint handler was installed
static bool g_sigint_installed;

/// Flag to indicate if a sigterm handler was installed
static bool g_sigterm_installed;

/// Signal handler function
DEFINE_SIGNAL_HANDLER(rclpy_sigint_handler)
{
  if (!is_null_signal_handler(g_original_sigint_handler)) {
    call_signal_handler(g_original_sigint_handler, SIGNAL_HANDLER_ARGS);
  }

  if (!g_signal_handler_installed.load()) {
    // There may have been another signal handler chaining to this
    // one when we last tried to unregister ourselves.

    // Try to unregister again.
    unregister_sigint_signal_handler();
  } else {
    notify_signal_handler();
  }
}

/// Signal handler function
DEFINE_SIGNAL_HANDLER(rclpy_sigterm_handler)
{
  if (!is_null_signal_handler(g_original_sigterm_handler)) {
    call_signal_handler(g_original_sigterm_handler, SIGNAL_HANDLER_ARGS);
  }

  if (!g_signal_handler_installed.load()) {
    // There may have been another signal handler chaining to this
    // one when we last tried to unregister ourselves.

    // Try to unregister again.
    unregister_sigterm_signal_handler();
  } else {
    notify_signal_handler();
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
  g_sigint_installed = false;
}

/// Register our SIGINT handler and store the current one
static void
register_sigint_signal_handler()
{
  if (g_sigint_installed) {
    // Handler already registered
    return;
  }
  g_sigint_installed = true;
  g_original_sigint_handler =
    install_signal_handler(SIGINT, get_rclpy_sigint_handler());
}

/// Unregister our SIGTERM handler and restore the original one
static void
unregister_sigterm_signal_handler()
{
  const SIGNAL_HANDLER_T current_sigterm_handler =
    install_signal_handler(SIGTERM, g_original_sigterm_handler);
  if (!is_rclpy_sigterm_handler(current_sigterm_handler)) {
    // Oops, someone else must have registered a signal handler
    // that chains to us put it back so it continues to work
    install_signal_handler(SIGTERM, current_sigterm_handler);
    return;
  }
  // Got ourself out of the chain
  g_original_sigterm_handler = NULL_SIGNAL_HANDLER;
  g_sigterm_installed = false;
}

/// Register our SIGTERM handler and store the current one
static void
register_sigterm_signal_handler()
{
  if (g_sigterm_installed) {
    // Handler already registered
    return;
  }
  g_sigterm_installed = true;
  g_original_sigterm_handler =
    install_signal_handler(SIGTERM, get_rclpy_sigterm_handler());
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
      RCUTILS_LOG_ERROR_NAMED(
        "rclpy.signals",
        "rcl_trigger_guard_condition() failed: %s", rcutils_get_error_string().str);
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

/// Enum to indicate which signal handlers to install.
enum class SignalHandlerOptions : int
{
  /// Install no signal handlers.
  No = 0,
  /// Install only a sigint handler.
  SigInt = 1,
  /// Install only a sigterm handler.
  SigTerm = 2,
  /// Install both a sigint and a sigterm handler.
  All = 3,
};

/// Install rclpy signal handlers.
/**
 * \param options rclpy.signals.SignalHandlerOptions integer value.
 */
void
install_signal_handlers(SignalHandlerOptions options)
{
  setup_deferred_signal_handler();
  switch (options) {
    case SignalHandlerOptions::No:
      return;
    case SignalHandlerOptions::SigInt:
      register_sigint_signal_handler();
      return;
    case SignalHandlerOptions::SigTerm:
      register_sigterm_signal_handler();
      return;
    case SignalHandlerOptions::All:
      register_sigint_signal_handler();
      register_sigterm_signal_handler();
      return;
  }
}

/// Return the currently active signal handler options.
/**
 * \return rclpy.signals.SignalHandlerOptions converted to integer value.
 */
SignalHandlerOptions
get_current_signal_handlers_options()
{
  // conversion to SignalHandlerOptions value
  return SignalHandlerOptions{g_sigterm_installed * 2 + g_sigint_installed};
}

/// Uninstall the currently installed signal handlers.
void
uninstall_signal_handlers()
{
  unregister_sigint_signal_handler();
  unregister_sigterm_signal_handler();
  teardown_deferred_signal_handler();
}

void
define_signal_handler_api(py::module m)
{
  g_original_sigint_handler = NULL_SIGNAL_HANDLER;
  g_original_sigterm_handler = NULL_SIGNAL_HANDLER;

  m.def(
    "register_sigint_guard_condition", &rclpy::register_sigint_guard_condition,
    "Register a guard condition to be called on SIGINT.");
  m.def(
    "unregister_sigint_guard_condition", &rclpy::unregister_sigint_guard_condition,
    "Stop triggering a guard condition when SIGINT occurs.");
  m.def(
    "install_signal_handlers", &rclpy::install_signal_handlers,
    "Install rclpy signal handlers.");
  m.def(
    "get_current_signal_handlers_options", &rclpy::get_current_signal_handlers_options,
    "Get currently installed signal handler options.");
  m.def(
    "uninstall_signal_handlers", &rclpy::uninstall_signal_handlers,
    "Uninstall rclpy signal handlers.");
  py::enum_<SignalHandlerOptions>(
    m, "SignalHandlerOptions", "Enum with values: `ALL`, `SIGINT`, `SIGTERM`, `NO`.")
  .value("ALL", SignalHandlerOptions::All)
  .value("NO", SignalHandlerOptions::No)
  .value("SIGINT", SignalHandlerOptions::SigInt)
  .value("SIGTERM", SignalHandlerOptions::SigTerm);
}
}  // namespace rclpy
