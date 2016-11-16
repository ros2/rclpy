// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <rmw/rmw.h>


/// Return a Python QoSProfile object
/* This function creates a QoSProfile object from the QoS Policies provided
 * \param[in] pyqos_history enum of type QoSHistoryPolicy
 * \param[in] pyqos_depth int size of the DDS message queue
 * \param[in] pyqos_reliability enum of type QoSReliabilityPolicy
 * \param[in] pyqos_durability enum of type QoSDurabilityPolicy
 * \return NULL on failure
 *         Capsule to a rmw_qos_profile_t object
 */
static PyObject *
rclpy_convert_from_py_qos_policy(PyObject * Py_UNUSED(self), PyObject * args)
{
  unsigned PY_LONG_LONG pyqos_history;
  unsigned PY_LONG_LONG pyqos_depth;
  unsigned PY_LONG_LONG pyqos_reliability;
  unsigned PY_LONG_LONG pyqos_durability;

  if (!PyArg_ParseTuple(
      args, "KKKK", &pyqos_history, &pyqos_depth, &pyqos_reliability, &pyqos_durability))
  {
    return NULL;
  }

  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)PyMem_Malloc(sizeof(rmw_qos_profile_t));
  qos_profile->history = pyqos_history;
  qos_profile->depth = pyqos_depth;
  qos_profile->reliability = pyqos_reliability;
  qos_profile->durability = pyqos_durability;
  PyObject * pyqos_profile = PyCapsule_New(qos_profile, NULL, NULL);
  return pyqos_profile;
}

/// Convert a C rmw_qos_profile_t into a Python QoSProfile object
/*
 * \param[in] void pointer to a rmw_qos_profile_t structure
 * \return QoSProfile object
 */
static PyObject *
rclpy_convert_to_py_qos_policy(void * profile)
{
  PyObject * pyqos_module = PyImport_ImportModule("rclpy.qos");
  PyObject * pyqos_policy_class = PyObject_GetAttrString(pyqos_module, "QoSProfile");
  PyObject * pyqos_profile = NULL;
  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)profile;
  pyqos_profile = PyObject_CallObject(pyqos_policy_class, NULL);
  assert(pyqos_profile != NULL);

  PyObject_SetAttrString(pyqos_profile, "depth", PyLong_FromSize_t(qos_profile->depth));
  PyObject_SetAttrString(pyqos_profile, "history", PyLong_FromUnsignedLong(qos_profile->history));
  PyObject_SetAttrString(pyqos_profile, "reliability",
    PyLong_FromUnsignedLong(qos_profile->reliability));
  PyObject_SetAttrString(pyqos_profile, "durability",
    PyLong_FromUnsignedLong(qos_profile->durability));

  assert(pyqos_profile != NULL);
  return pyqos_profile;
}

/// Fetch a predefined qos_profile from rmw and convert it to a Python QoSProfile Object
/* This function takes a string defining a rmw_qos_profile_t and return the corresponding Python QoSProfile object
 * \param[in] string with the name of the profile to load
 * \return QoSProfile object
 */
static PyObject *
rclpy_get_rmw_qos_profile(PyObject * Py_UNUSED(self), PyObject * args)
{
  const char * pyrmw_profile;
  if (!PyArg_ParseTuple(
      args, "z", &pyrmw_profile))
  {
    return NULL;
  }
  PyObject * pyqos_profile = NULL;
  if (0 == strcmp(pyrmw_profile, "qos_profile_sensor_data")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_sensor_data);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_default")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_default);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_system_default")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_system_default);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_services_default")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_services_default);
    // NOTE(mikaelarguedas) all conditions following this one are defined but not used
    // because parameters are not implemented in Python yet
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameters")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_parameters);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameter_events")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_parameter_events);
  } else {
    PyErr_Format(PyExc_RuntimeError,
      "Requested unknown rmw_qos_profile: %s", pyrmw_profile);
    return NULL;
  }
  return pyqos_profile;
}


/// Define the public methods of this module
static PyMethodDef _rclpy_implementation_rmw_agnostic_methods[] = {
  {"rclpy_convert_from_py_qos_policy", rclpy_convert_from_py_qos_policy, METH_VARARGS,
   "Convert a QoSPolicy python object into a rmw_qos_profile_t."},

  {"rclpy_get_rmw_qos_profile", rclpy_get_rmw_qos_profile, METH_VARARGS,
   "Get QOS profile."},

  {NULL, NULL, 0, NULL}  /* sentinel */
};

/// Define the Python module
static struct PyModuleDef _rclpy_implementation_rmw_agnosticmodule = {
  PyModuleDef_HEAD_INIT,
  "_rclpy_implementation_rmw_agnostic",
  "_rclpy_implementation_rmw_agnostic_doc",
  -1,   /* -1 means that the module keeps state in global variables */
  _rclpy_implementation_rmw_agnostic_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy_implementation_rmw_agnostic(void)
{
  return PyModule_Create(&_rclpy_implementation_rmw_agnosticmodule);
}
