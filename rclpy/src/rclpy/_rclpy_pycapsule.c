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

/// Get the name of a pycapsule.
/**
 * Raises TypeError if the argument is not a pycapsule
 *
 * \param[in] pycapsule a pycapsule
 * \return Name or None if the capsule has no name.
 */
static PyObject *
rclpy_pycapsule_name(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pycapsule;
  if (!PyArg_ParseTuple(args, "O", &pycapsule)) {
    return NULL;
  }

  const char * name = PyCapsule_GetName(pycapsule);

  if (PyErr_Occurred()) {
    return NULL;
  }

  if (NULL == name) {
    Py_RETURN_NONE;
  }
  return PyUnicode_FromString(name);
}

/// Get the address held by a pycapsule.
/**
 * Raises TypeError if the argument is not a pycapsule
 *
 * \param[in] pycapsule a pycapsule
 * \return integer with the address held by the pycapsule.
 */
static PyObject *
rclpy_pycapsule_pointer(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pycapsule;
  if (!PyArg_ParseTuple(args, "O", &pycapsule)) {
    return NULL;
  }

  const char * name = PyCapsule_GetName(pycapsule);

  if (PyErr_Occurred()) {
    return NULL;
  }

  void * pointer = PyCapsule_GetPointer(pycapsule, name);

  if (PyErr_Occurred()) {
    return NULL;
  }

  _Static_assert(sizeof(uint64_t) >= sizeof(void *), "Unable to cast pointer to integer");
  return PyLong_FromUnsignedLongLong((uint64_t)pointer);
}

/// Destroy a pycapsule without waiting for the garbage collector.
/**
 * Raises TypeError if the argument is not a pycapsule.
 * Raises ValueError if the pycapsule does not have a destructor.
 *
 * \param[in] pycapsule a pycapsule
 * \return None
 */
static PyObject *
rclpy_pycapsule_destroy(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pycapsule;
  if (!PyArg_ParseTuple(args, "O", &pycapsule)) {
    return NULL;
  }

  PyCapsule_Destructor destructor = PyCapsule_GetDestructor(pycapsule);

  if (PyErr_Occurred()) {
    return NULL;
  }

  if (NULL == destructor) {
    PyErr_Format(PyExc_ValueError, "PyCapsule does not have a destructor.");
  }

  // Need name to get pointer
  const char * name = PyCapsule_GetName(pycapsule);

  if (PyErr_Occurred()) {
    return NULL;
  }

  void * pointer = PyCapsule_GetPointer(pycapsule, name);

  if (NULL == pointer) {
    return NULL;
  }

  destructor(pointer);

  if (0 != PyCapsule_SetDestructor(pycapsule, NULL)) {
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Define the public methods of this module
static PyMethodDef rclpy_pycapsule_methods[] = {
  {
    "rclpy_pycapsule_name", rclpy_pycapsule_name,
    METH_VARARGS,
    "Return the name of a pycapsule, or None."
  },
  {
    "rclpy_pycapsule_pointer", rclpy_pycapsule_pointer,
    METH_VARARGS,
    "Return the address held by a pycapsule."
  },
  {
    "rclpy_pycapsule_destroy", rclpy_pycapsule_destroy,
    METH_VARARGS,
    "Destroy a pycapsule and clear its destructor"
  },
  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rclpy_pycapsule__doc__,
  "RCLPY module for working with PyCapsule objects.");

/// Define the Python module
static struct PyModuleDef _rclpy_pycapsule_module = {
  PyModuleDef_HEAD_INIT,
  "_rclpy_pycapsule",
  rclpy_pycapsule__doc__,
  -1,  /* -1 means that the module keeps state in global variables */
  rclpy_pycapsule_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy_pycapsule(void)
{
  return PyModule_Create(&_rclpy_pycapsule_module);
}
