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
 * \return None
 */
static PyObject *
rclpy_logging_initialize(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcutils_logging_initialize();
  Py_RETURN_NONE;
}

/// Get the global severity threshold of the logging system.
/**
 * \return severity
 */
static PyObject *
rclpy_logging_get_severity_threshold(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  int severity = rcutils_logging_get_severity_threshold();

  return PyLong_FromLong(severity);
}

/// Set the global severity threshold of the logging system.
/**
 *
 * \param[in] severity Threshold to set
 * \return None
 */
static PyObject *
rclpy_logging_set_severity_threshold(PyObject * Py_UNUSED(self), PyObject * args)
{
  int severity;
  if (!PyArg_ParseTuple(args, "i", &severity)) {
    return NULL;
  }

  rcutils_logging_set_severity_threshold(severity);
  Py_RETURN_NONE;
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
  unsigned PY_LONG_LONG severity;
  const char * name;
  const char * message;
  const char * function_name;
  const char * file_name;
  unsigned PY_LONG_LONG line_number;
  if (!PyArg_ParseTuple(args, "KssssK",
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
  {"rclpy_logging_initialize", rclpy_logging_initialize, METH_NOARGS,
   "Initialize the logging system."},
  {"rclpy_logging_get_severity_threshold", rclpy_logging_get_severity_threshold, METH_NOARGS,
   "Get the global severity threshold."},
  {"rclpy_logging_set_severity_threshold", rclpy_logging_set_severity_threshold, METH_VARARGS,
   "Set the global severity threshold."},
  {"rclpy_logging_rcutils_log", rclpy_logging_rcutils_log, METH_VARARGS,
   "Log a message with the specified severity"},

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
