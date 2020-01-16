// Copyright 2020 Open Source Robotics Foundation, Inc.
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
#include <stddef.h>

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"
#include "rcutils/types/rcutils_ret.h"

#include "rclpy_common/handle.h"


static PyObject *
rclpy_handle_add_dependency(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * dependent_capsule;
  PyObject * dependency_capsule;
  if (!PyArg_ParseTuple(args, "OO", &dependent_capsule, &dependency_capsule)) {
    return NULL;
  }

  rclpy_handle_t * dependent =
    PyCapsule_GetPointer(dependent_capsule, PyCapsule_GetName(dependent_capsule));
  rclpy_handle_t * dependency =
    PyCapsule_GetPointer(dependency_capsule, PyCapsule_GetName(dependency_capsule));

  if (PyErr_Occurred()) {
    return NULL;
  }

  if (RCUTILS_RET_OK != _rclpy_handle_add_dependency(dependent, dependency)) {
    PyErr_Format(PyExc_RuntimeError, "Failed to add dependency to handle");
    return NULL;
  }

  return Py_None;
}

static PyObject *
rclpy_handle_get_name(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * handle_capsule;
  if (!PyArg_ParseTuple(args, "O", &handle_capsule)) {
    return NULL;
  }

  if (PyErr_Occurred()) {
    return NULL;
  }
  return PyUnicode_FromString(PyCapsule_GetName(handle_capsule));
}

static PyObject *
rclpy_handle_get_pointer(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * handle_capsule;
  if (!PyArg_ParseTuple(args, "O", &handle_capsule)) {
    return NULL;
  }

  if (PyErr_Occurred()) {
    return NULL;
  }

  void * ptr = rclpy_handle_get_pointer_from_capsule(
    handle_capsule, PyCapsule_GetName(handle_capsule));

  if (!ptr || PyErr_Occurred()) {
    return NULL;
  }

  return PyLong_FromVoidPtr(ptr);
}

/// Define the public methods of this module
static PyMethodDef rclpy_handle_methods[] = {
  {
    "rclpy_handle_add_dependency", rclpy_handle_add_dependency,
    METH_VARARGS,
    "Add a dependency to a handle."
  },
  {
    "rclpy_handle_get_name", rclpy_handle_get_name,
    METH_VARARGS,
    "Get handle name."
  },
  {
    "rclpy_handle_get_pointer", rclpy_handle_get_pointer,
    METH_VARARGS,
    "Get handle pointer."
  },
  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rclpy_handle__doc__,
  "rclpy module for working with Handle objects.");

/// Define the Python module
static struct PyModuleDef _rclpy_handle_module = {
  PyModuleDef_HEAD_INIT,
  "_rclpy_handle",
  rclpy_handle__doc__,
  -1,  /* -1 means that the module keeps state in global variables */
  rclpy_handle_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy_handle(void)
{
  return PyModule_Create(&_rclpy_handle_module);
}
