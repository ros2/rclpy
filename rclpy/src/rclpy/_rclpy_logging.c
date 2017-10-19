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

#include <rcutils/logging.h>
#include <rcutils/logging_macros.h>
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
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Get the default severity threshold of the logging system.
/**
 * \return severity
 */
static PyObject *
rclpy_logging_get_default_severity_threshold(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  int severity = rcutils_logging_get_default_severity_threshold();

  return PyLong_FromLong(severity);
}

/// Set the default severity threshold of the logging system.
/**
 *
 * \param[in] severity Threshold to set
 * \return None
 */
static PyObject *
rclpy_logging_set_default_severity_threshold(PyObject * Py_UNUSED(self), PyObject * args)
{
  int severity;
  if (!PyArg_ParseTuple(args, "i", &severity)) {
    return NULL;
  }

  rcutils_logging_set_default_severity_threshold(severity);
  Py_RETURN_NONE;
}

/// Get the severity threshold of a logger.
/**
 * \param[in] name Fully-qualified name of logger.
 * \return The logger severity threshold if it has been set, or
 * \return `RCUTILS_LOG_SEVERITY_UNSET` if it is unset, or
 * \return NULL on failure
 */
static PyObject *
rclpy_logging_get_logger_severity_threshold(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * name;
  if (!PyArg_ParseTuple(args, "s", &name)) {
    return NULL;
  }
  int severity = rcutils_logging_get_logger_severity_threshold(name);

  if (severity < 0) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get severity threshold for logger \"%s\", return code: %d\n",
      name, severity);
    return NULL;
  }
  return PyLong_FromLong(severity);
}

/// Set the severity threshold of a logger.
/**
 *
 * \param[in] name Fully-qualified name of logger.
 * \param[in] severity Threshold to set
 * \return None or
 * \return NULL on failure
 */
static PyObject *
rclpy_logging_set_logger_severity_threshold(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * name;
  int severity;
  if (!PyArg_ParseTuple(args, "si", &name, &severity)) {
    return NULL;
  }

  rcutils_ret_t ret = rcutils_logging_set_logger_severity_threshold(name, severity);
  if (ret != RCUTILS_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to set severity threshold \"%d\" for logger \"%s\", return code: %d\n",
      severity, name, ret);
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Get the effective severity threshold of a logger.
/**
 * The effective severity threshold is determined as the logger severity if it has been set,
 * otherwise it defers to the severity threshold of the logger's ancestors, and if all are unset
 * the default severity threshold is used.
 *
 * \param[in] name Fully-qualified name of logger.
 * \return The effective severity threshold, or
 * \return NULL of failure.
 */
static PyObject *
rclpy_logging_get_logger_effective_severity_threshold(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * name;
  if (!PyArg_ParseTuple(args, "s", &name)) {
    return NULL;
  }
  int severity = rcutils_logging_get_logger_effective_severity_threshold(name);

  if (severity < 0) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get effective severity threshold for logger \"%s\", return code: %d\n",
      name, severity);
    return NULL;
  }
  return PyLong_FromLong(severity);
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
    "rclpy_logging_get_default_severity_threshold", rclpy_logging_get_default_severity_threshold,
    METH_NOARGS, "Get the global severity threshold."
  },
  {
    "rclpy_logging_set_default_severity_threshold", rclpy_logging_set_default_severity_threshold,
    METH_VARARGS, "Set the global severity threshold."
  },
  {
    "rclpy_logging_get_logger_severity_threshold", rclpy_logging_get_logger_severity_threshold,
    METH_VARARGS, "Get the severity threshold of a logger."
  },
  {
    "rclpy_logging_set_logger_severity_threshold", rclpy_logging_set_logger_severity_threshold,
    METH_VARARGS, "Set the severity threshold of a logger."
  },
  {
    "rclpy_logging_get_logger_effective_severity_threshold",
    rclpy_logging_get_logger_effective_severity_threshold,
    METH_VARARGS, "Get the effective severity threshold of a logger."
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
