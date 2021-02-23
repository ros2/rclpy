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

/// Get the name of a pycapsule.
/**
 * Raises TypeError if the argument is not a pycapsule
 *
 * \param[in] pycapsule a pycapsule
 * \return Name or None if the capsule has no name.
 */
const char *
rclpy_pycapsule_name(py::capsule capsule)
{
  return capsule.name();
}

/// Get the address held by a pycapsule.
/**
 * Raises TypeError if the argument is not a pycapsule
 *
 * \param[in] pycapsule a pycapsule
 * \return integer with the address held by the pycapsule.
 */
size_t
rclpy_pycapsule_pointer(py::capsule capsule)
{
  // TODO(sloretz) use get_pointer in pybind11 2.6
  return reinterpret_cast<size_t>(static_cast<void *>(capsule));
}

/// Destroy a pycapsule without waiting for the garbage collector.
/**
 * Raises TypeError if the argument is not a pycapsule.
 * Raises ValueError if the pycapsule does not have a destructor.
 *
 * \param[in] pycapsule a pycapsule
 * \return None
 */
void
rclpy_pycapsule_destroy(py::capsule capsule)
{
  // Need to work with raw PyObject because py::capsule doesn't have apis for destructor
  PyObject * pycapsule = capsule.ptr();

  PyCapsule_Destructor destructor = PyCapsule_GetDestructor(pycapsule);

  if (PyErr_Occurred()) {
    throw py::error_already_set();
  }

  if (!destructor) {
    throw py::value_error("PyCapsule does not have a destructor.");
  }

  destructor(pycapsule);

  if (0 != PyCapsule_SetDestructor(pycapsule, NULL)) {
    throw py::error_already_set();
  }
}

PYBIND11_MODULE(_rclpy_pycapsule, m) {
  m.doc() = "rclpy module for working with PyCapsule objects.";
  m.def("rclpy_pycapsule_name", &rclpy_pycapsule_name);
  m.def("rclpy_pycapsule_pointer", &rclpy_pycapsule_pointer);
  m.def("rclpy_pycapsule_destroy", &rclpy_pycapsule_destroy);
}
