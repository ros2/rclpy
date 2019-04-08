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

#include <Python.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcutils/allocator.h>
#include <rcutils/stdatomic_helper.h>

#include <signal.h>

#ifdef _WIN32
  #define SIGNAL_HANDLER_T _crt_signal_t
#else
  #define SIGNAL_HANDLER_T sig_t
#endif  // _WIN32

/// Global reference to original signal handler for chaining purposes
SIGNAL_HANDLER_T g_original_signal_handler = NULL;

typedef _Atomic (rcl_guard_condition_t **) atomic_rcl_guard_condition_ptrptr_t;

/// Global reference to guard conditions
/// End with sentinel value instead of count to avoid mismatch if signal
/// interrupts while adding or removing from the list
atomic_rcl_guard_condition_ptrptr_t g_guard_conditions = NULL;

/// Warn if getting g_guard_conditions could deadlock the signal handler
/// \return 0 if no exception is raised, -1 if an exception was raised
static int
check_signal_safety()
{
  static bool did_warn = false;
  if (!did_warn && !atomic_is_lock_free(g_guard_conditions)) {
    did_warn = true;
    const char * deadlock_msg =
      "Global guard condition list access is not lock-free on this platform."
      "The program may deadlock when receiving SIGINT.";
    return PyErr_WarnEx(PyExc_ResourceWarning, deadlock_msg, 1);
  }
  return 0;
}

// Forward declaration
static void catch_function(int signo);

/// Restore the original signal handler when ours was registered
static void
restore_original_signal_handler()
{
  const SIGNAL_HANDLER_T current_handler = signal(SIGINT, g_original_signal_handler);
  if (current_handler != catch_function) {
    // Oops, someone else must have registered a signal handler that chains to us
    // put it back so it continues to work
    signal(SIGINT, current_handler);
    return;
  }
  // Got ourself out of the chain
  g_original_signal_handler = NULL;
}

/// Register our signal handler and store the current
static void
register_signal_handler()
{
  if (NULL != g_original_signal_handler) {
    // We must already be registered
    return;
  }
  g_original_signal_handler = signal(SIGINT, catch_function);
}

/// Call the original signal handler if there was one
static void
call_original_signal_handler(int signo)
{
  if (NULL != g_original_signal_handler) {
    g_original_signal_handler(signo);
  }
}

/// Catch signals
/**
 * This triggers guard conditions when a signal is received.
 * These wake executors currently blocked in `rcl_wait`.
 */
static void catch_function(int signo)
{
  rcl_guard_condition_t ** guard_conditions;
  rcutils_atomic_load(&g_guard_conditions, guard_conditions);
  if (NULL == guard_conditions || NULL == guard_conditions[0]) {
    call_original_signal_handler(signo);
    // There may have been another signal handler chaining to this one when we last tried to
    // restore the old signal handler.
    // Try to restore the old handler again.
    restore_original_signal_handler();
    return;
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

  // Chain signal handlers
  call_original_signal_handler(signo);
}

/// Register a guard condition to be triggered when SIGINT is received.
/**
 * Raises ValueError if the argument is not a guard condition handle
 * Raises ValueError if the argument was already registered
 *
 * \param[in] pygc a guard condition pycapsule
 * \return None
 */
static PyObject *
rclpy_register_sigint_guard_condition(PyObject * Py_UNUSED(self), PyObject * args)
{
  // Expect a pycapsule with a guard condition
  PyObject * pygc;
  if (!PyArg_ParseTuple(args, "O", &pygc)) {
    return NULL;
  }

  if (0 != check_signal_safety()) {
    // exception raised
    return NULL;
  }

  rcl_guard_condition_t * gc = (rcl_guard_condition_t *)PyCapsule_GetPointer(
    pygc, "rcl_guard_condition_t");
  if (!gc) {
    return NULL;
  }

  rcl_guard_condition_t ** guard_conditions;
  rcutils_atomic_load(&g_guard_conditions, guard_conditions);

  // Figure out how big the list currently is
  size_t count_gcs = 0;
  if (NULL != guard_conditions) {
    while (NULL != guard_conditions[count_gcs]) {
      if (gc == guard_conditions[count_gcs]) {
        PyErr_Format(PyExc_ValueError, "Guard condition was already registered");
        return NULL;
      }
      ++count_gcs;
    }
  }

  // Current size of guard condition list: count_gcs + 1 (sentinel value)
  // Allocate space for one more guard condition: count_cs + 1 (new gc) + 1 (sentinel value)
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_guard_condition_t ** new_gcs =
    allocator.allocate(sizeof(rcl_guard_condition_t *) * (count_gcs + 2), allocator.state);

  // populate the new guard condition list, ending with a sentinel of NULL
  for (size_t i = 0; i < count_gcs; ++i) {
    new_gcs[i] = guard_conditions[i];
  }
  new_gcs[count_gcs] = gc;
  new_gcs[count_gcs + 1] = NULL;

  // Swap the lists and free the old
  rcl_guard_condition_t ** old_gcs;
  rcutils_atomic_exchange(&g_guard_conditions, old_gcs, new_gcs);
  if (NULL != old_gcs) {
    allocator.deallocate(old_gcs, allocator.state);
  }

  // make sure our signal handler is registered
  register_signal_handler();

  Py_RETURN_NONE;
}

/// Unregister a guard condition so it is not triggered when SIGINT is received.
/**
 * Raises ValueError if the argument is not a guard condition handle
 * Raises ValueError if the argument was not registered
 *
 * \param[in] pygc a guard condition pycapsule
 * \return None
 */
static PyObject *
rclpy_unregister_sigint_guard_condition(PyObject * Py_UNUSED(self), PyObject * args)
{
  // Expect a pycapsule with a guard condition
  PyObject * pygc;
  if (!PyArg_ParseTuple(args, "O", &pygc)) {
    return NULL;
  }

  rcl_guard_condition_t * gc = (rcl_guard_condition_t *)PyCapsule_GetPointer(
    pygc, "rcl_guard_condition_t");
  if (!gc) {
    return NULL;
  }

  rcl_guard_condition_t ** guard_conditions;
  rcutils_atomic_load(&g_guard_conditions, guard_conditions);

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

  if (count_gcs == 0 || !found_gc) {
    PyErr_Format(PyExc_ValueError, "Guard condition was not registered");
    return NULL;
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();

  if (count_gcs == 1) {
    // Just delete the list if there are no guard conditions left
    rcl_guard_condition_t ** old_gcs;
    rcutils_atomic_exchange(&g_guard_conditions, old_gcs, NULL);
    allocator.deallocate(old_gcs, allocator.state);
    restore_original_signal_handler();
  } else {
    // Create space for one less guard condition
    // current list size: count_gcs + 1 (sentinel)
    // new list size: count_gcs - 1 (removing a guard condition) + 1 (sentinel)
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_guard_condition_t ** new_gcs =
      allocator.allocate(sizeof(rcl_guard_condition_t *) * (count_gcs), allocator.state);

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
    rcl_guard_condition_t ** old_gcs;
    rcutils_atomic_exchange(&g_guard_conditions, old_gcs, new_gcs);
    allocator.deallocate(old_gcs, allocator.state);
  }

  Py_RETURN_NONE;
}

/// Define the public methods of this module
static PyMethodDef rclpy_signal_handler_methods[] = {
  {
    "rclpy_register_sigint_guard_condition", rclpy_register_sigint_guard_condition,
    METH_VARARGS,
    "Register a guard condition to be called on SIGINT."
  },
  {
    "rclpy_unregister_sigint_guard_condition", rclpy_unregister_sigint_guard_condition,
    METH_VARARGS,
    "Stop triggering a guard condition when SIGINT occurs."
  },
  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rclpy_signal_handler__doc__,
  "RCLPY module for handling signals.");

/// Define the Python module
static struct PyModuleDef _rclpy_signal_handler_module = {
  PyModuleDef_HEAD_INIT,
  "_rclpy_signal_handler",
  rclpy_signal_handler__doc__,
  -1,  /* -1 means that the module keeps state in global variables */
  rclpy_signal_handler_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy_signal_handler(void)
{
  atomic_init(&g_guard_conditions, NULL);
  return PyModule_Create(&_rclpy_signal_handler_module);
}
