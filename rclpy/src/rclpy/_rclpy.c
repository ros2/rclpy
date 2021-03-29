// Copyright 2016-2020 Open Source Robotics Foundation, Inc.
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

// Must be before Python.h; makes #-formats use ssize_t instead of int
#define PY_SSIZE_T_CLEAN
#include <Python.h>

typedef struct
{
} rclpy_module_state_t;

/// Define the public methods of this module
static PyMethodDef rclpy_methods[] = {
  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rclpy__doc__, "ROS 2 Python client library.");

static int rclpy_traverse(PyObject * m, visitproc visit, void * arg)
{
  (void) m;
  (void) visit;
  (void) arg;
  return 0;
}

static int rclpy_clear(PyObject * m)
{
  (void) m;
  return 0;
}

static PyModuleDef_Slot rclpy_slots[] = {
  {0, NULL}
};

/// Define the Python module
static struct PyModuleDef _rclpymodule = {
  PyModuleDef_HEAD_INIT,
  "_rclpy",
  rclpy__doc__,
  sizeof(rclpy_module_state_t),
  rclpy_methods,
  rclpy_slots,
  rclpy_traverse,
  rclpy_clear,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy(void)
{
  return PyModuleDef_Init(&_rclpymodule);
}
