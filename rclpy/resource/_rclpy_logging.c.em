// generated from rclpy/resource/_rcly_logging.c.em

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
rclpy_logging_initialize()
{
  rcutils_logging_initialize();
  Py_RETURN_NONE;
}

/// Set the global severity threshold of the logging system.
/**
 *
 * \param[in] severity Threshold to set
 * \return None, or
 * \return NULL on failure
 */
static PyObject *
rclpy_logging_set_severity_threshold(PyObject * Py_UNUSED(module), PyObject * args)
{
  int severity;
  if (!PyArg_ParseTuple(args, "i", &severity)) {
    return NULL;
  }

  rcutils_logging_set_severity_threshold(severity);
  Py_RETURN_NONE;
}

/// Get the global severity threshold of the logging system.
/**
 * \return severity
 */
static PyObject *
rclpy_logging_get_severity_threshold()
{
  int severity = rcutils_logging_get_severity_threshold();

  return PyLong_FromLong(severity);
}
@{
import sys
sys.path.insert(0, logging_rcutils_config_path)
from logging_rcutils_config import supported_feature_combinations, supported_logging_severities
}@

@[for severity in supported_logging_severities]@
@[ for suffix in supported_feature_combinations]@
/// rclpy_logging_log_@(severity.lower())@(suffix.lower()).
/**
 * Log a message with severity @(severity)@
@[ if supported_feature_combinations[suffix].doc_lines]@
 with the following conditions:
@[ else]@
.
@[ end if]@
@[ for doc_line in supported_feature_combinations[suffix].doc_lines]@
 * @(doc_line)
@[ end for]@
 *
 * \param message String to log.
@[ for param_name, properties in supported_feature_combinations[suffix].params.items()]@
 * \param @(param_name) @(properties['doc_line'])
@[ end for]@
 * \return None
 */
static PyObject *
rclpy_logging_log_@(severity.lower())@(suffix.lower())(PyObject * Py_UNUSED(module), PyObject * args, PyObject * keywds)
{
  const char * message;
  static char *kwlist[] = {"message"@(''.join([', "' + p['scoped_name'] + '"' for n, p in supported_feature_combinations[suffix].params.items()])), NULL};
@{
additional_tuple_types = ''
}@
@[  for param_name, properties in supported_feature_combinations[suffix].params.items()]@
  @(properties['c_type']) @(properties['scoped_name']);
@{
additional_tuple_types += properties['tuple_type']
}@
@[  end for]@
  if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|@(additional_tuple_types)", kwlist, &message@(''.join([', &' + p['scoped_name'] for n, p in supported_feature_combinations[suffix].params.items()])))) {
    return NULL;
  }
  RCUTILS_LOG_@(severity)@(suffix)(@(''.join([p['scoped_name'] + ', ' for n, p in supported_feature_combinations[suffix].params.items()]))message)
  Py_RETURN_NONE;
}

@[ end for]@
@[end for]@

/// Define the public methods of this module
static PyMethodDef rclpy_logging_methods[] = {
  {"rclpy_logging_initialize", rclpy_logging_initialize, METH_NOARGS,
   "Initialize the logging system."},
  {"rclpy_logging_get_severity_threshold", rclpy_logging_get_severity_threshold, METH_NOARGS,
   "Get the global severity threshold."},
  {"rclpy_logging_set_severity_threshold", rclpy_logging_set_severity_threshold, METH_VARARGS,
   "Set the global severity threshold."},

@[for severity in supported_logging_severities]@
@[for suffix in supported_feature_combinations]@
  {"rclpy_logging_log_@(severity.lower())@(suffix.lower())", (PyCFunction)rclpy_logging_log_@(severity.lower())@(suffix.lower()), METH_VARARGS|METH_KEYWORDS,
   "Log a message with severity @(severity) and feature(s) @(suffix)"},
@[ end for]@
@[end for]@

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
