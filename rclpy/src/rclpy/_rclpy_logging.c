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

#include <Python.h>

#include <rcutils/error_handling.h>
#include <rcutils/logging.h>
#include <rcutils/time.h>

/// Initialize the logging system.
/**
 * \return None or
 * \return NULL on failure
 */
static PyObject *
rclpy_logging_initialize(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcutils_ret_t ret = rcutils_logging_initialize();
  if (ret != RCUTILS_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to initialize logging system, return code: %d\n", ret);
    rcutils_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Shutdown the logging system.
/**
 * \return None or
 * \return NULL on failure
 */
static PyObject *
rclpy_logging_shutdown(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  // TODO(dhood): error checking
  rcutils_ret_t ret = rcutils_logging_shutdown();
  if (ret != RCUTILS_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to shutdown logging system, return code: %d\n", ret);
    rcutils_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Set the level of a logger.
/**
 *
 * \param[in] name Fully-qualified name of logger.
 * \param[in] level to set
 * \return None or
 * \return NULL on failure
 */
static PyObject *
rclpy_logging_set_logger_level(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * name;
  int level;
  if (!PyArg_ParseTuple(args, "si", &name, &level)) {
    return NULL;
  }

  rcutils_ret_t ret = rcutils_logging_set_logger_level(name, level);
  if (ret != RCUTILS_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to set level \"%d\" for logger \"%s\", return code: %d\n",
      level, name, ret);
    rcutils_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Get the effective level of a logger.
/**
 * The "effective" logger level is determined as the logger level if it has been set explicitly,
 * otherwise it defers to the level of the logger's ancestors, and if all are unset
 * the default level is used.
 *
 * \param[in] name Fully-qualified name of logger.
 * \return The effective level, or
 * \return NULL of failure.
 */
static PyObject *
rclpy_logging_get_logger_effective_level(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * name;
  if (!PyArg_ParseTuple(args, "s", &name)) {
    return NULL;
  }
  int logger_level = rcutils_logging_get_logger_effective_level(name);

  if (logger_level < 0) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get effective level for logger \"%s\", return code: %d\n",
      name, logger_level);
    rcutils_reset_error();
    return NULL;
  }
  return PyLong_FromLong(logger_level);
}

/// Determine if the logger is enabled for a severity.
/**
 *
 * \param[in] name Fully-qualified name of logger.
 * \param[in] severity Logging severity to compare against.
 * \return True if the logger is enabled for the severity,
 * \return False otherwise.
 */
static PyObject *
rclpy_logging_logger_is_enabled_for(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * name;
  int severity;
  if (!PyArg_ParseTuple(args, "si", &name, &severity)) {
    return NULL;
  }

  bool is_enabled = rcutils_logging_logger_is_enabled_for(name, severity);
  if (is_enabled) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

/// Log a message through rcutils with the specified severity.
/**
 *
 * \param[in] severity Enum of type RCUTILS_LOG_SEVERITY.
 * \param[in] name Name of logger.
 * \param[in] message String to log.
 * \param[in] function_name String with the function name of the caller.
 * \param[in] file_name String with the file name of the caller.
 * \param[in] line_number Line number of the calling function.
 * \return None
 */
static PyObject *
rclpy_logging_rcutils_log(PyObject * Py_UNUSED(self), PyObject * args)
{
  int severity;
  const char * name;
  const char * message;
  const char * function_name;
  const char * file_name;
  unsigned PY_LONG_LONG line_number;
  if (!PyArg_ParseTuple(args, "issssK",
    &severity, &name, &message, &function_name, &file_name, &line_number))
  {
    return NULL;
  }

  RCUTILS_LOGGING_AUTOINIT
  rcutils_log_location_t logging_location = {function_name, file_name, line_number};
  rcutils_log(&logging_location, severity, name, message);
  Py_RETURN_NONE;
}

/// Define the public methods of this module
static PyMethodDef rclpy_logging_methods[] = {
  {
    "rclpy_logging_initialize", rclpy_logging_initialize, METH_NOARGS,
    "Initialize the logging system."
  },
  {
    "rclpy_logging_shutdown", rclpy_logging_shutdown, METH_NOARGS,
    "Shutdown the logging system."
  },
  {
    "rclpy_logging_set_logger_level", rclpy_logging_set_logger_level,
    METH_VARARGS, "Set the level of a logger."
  },
  {
    "rclpy_logging_get_logger_effective_level",
    rclpy_logging_get_logger_effective_level,
    METH_VARARGS, "Get the effective level of a logger."
  },
  {
    "rclpy_logging_logger_is_enabled_for", rclpy_logging_logger_is_enabled_for,
    METH_VARARGS, "Determine if a logger is enabled for a severity."
  },
  {
    "rclpy_logging_rcutils_log", rclpy_logging_rcutils_log, METH_VARARGS,
    "Log a message with the specified severity"
  },

  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rclpy_logging__doc__,
  "RCLPY module for logging.");

/// Define the Python module
static struct PyModuleDef _rclpy_logging_module = {
  PyModuleDef_HEAD_INIT,
  "_rclpy_logging",
  rclpy_logging__doc__,
  -1,   /* -1 means that the module keeps state in global variables */
  rclpy_logging_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy_logging(void)
{
  return PyModule_Create(&_rclpy_logging_module);
}
