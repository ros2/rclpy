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

#include <rcl/graph.h>
#include <rmw/rmw.h>


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
    // NOTE(mikaelarguedas) all conditions following this one are defined but not used
    // because services and parameters are not implemented in Python yet
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameters")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_parameters);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_services_default")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_services_default);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameter_events")) {
    pyqos_profile = rclpy_convert_to_py_qos_policy((void *)&rmw_qos_profile_parameter_events);
  } else {
    PyErr_Format(PyExc_RuntimeError,
      "Requested unknown rmw_qos_profile: %s", pyrmw_profile);
    return NULL;
  }
  return pyqos_profile;
}

static PyObject *
rclpy_get_topic_names_and_types(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pyrcl_topic_names_and_type;

  if (!PyArg_ParseTuple(args, "OO", &pynode, &pyrcl_topic_names_and_type)) {
    return NULL;
  }

  const rcl_node_t * node = (const rcl_node_t *)PyCapsule_GetPointer(pynode, NULL);
  rcl_topic_names_and_types_t * topic_names_and_types = NULL;
  rcl_ret_t ret = rcl_get_topic_names_and_types(node, topic_names_and_types);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to get_topic_names_and_types: %s", "!!!TODO!!! NEED TO GET NAME FROM RMW_NODE_T");
    return NULL;
  }
  PyObject * pynames_types_module = PyImport_ImportModule("rclpy.names_and_types");
  PyObject * pytopic_names_types_class = PyObject_GetAttrString(
    pynames_types_module, "TopicNamesAndTypes");
  PyObject * pytopic_names_types = NULL;
  pytopic_names_types = PyObject_CallObject(pytopic_names_types_class, NULL);

  PyObject * pytopic_names = PyList_New(topic_names_and_types->topic_count);
  PyObject * pytype_names = PyList_New(topic_names_and_types->topic_count);
  size_t idx;
  for (idx = 0; idx < topic_names_and_types->topic_count; idx++) {
    PyList_SetItem(
      pytopic_names, idx, PyUnicode_FromString(topic_names_and_types->topic_names[idx]));
    PyList_SetItem(
      pytype_names, idx, PyUnicode_FromString(topic_names_and_types->type_names[idx]));
  }
  assert(PySequence_Check(pytopic_names));
  assert(pytopic_names != NULL);
  assert(PySequence_Check(pytype_names));
  assert(pytype_names != NULL);
  PyObject_SetAttrString(pytopic_names_types, "topic_names", pytopic_names);
  PyObject_SetAttrString(pytopic_names_types, "type_names", pytype_names);
  PyObject_SetAttrString(
    pytopic_names_types,
    "topic_count",
    PyLong_FromUnsignedLong(topic_names_and_types->topic_count));

  assert(pytopic_names_types != NULL);
  return pytopic_names_types;
}

static PyMethodDef _rclpy_implementation_rmw_agnostic_methods[] = {
  {"rclpy_convert_from_py_qos_policy", rclpy_convert_from_py_qos_policy, METH_VARARGS,
   "Convert a QoSPolicy python object into a rmw_qos_profile_t."},

  {"rclpy_get_rmw_qos_profile", rclpy_get_rmw_qos_profile, METH_VARARGS,
   "Get QOS profile."},

  {"rclpy_get_topic_names_and_types", rclpy_get_topic_names_and_types, METH_VARARGS,
   "Get topic list from graph API."},

  {NULL, NULL, 0, NULL}  /* sentinel */
};

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

PyMODINIT_FUNC PyInit__rclpy_implementation_rmw_agnostic(void)
{
  return PyModule_Create(&_rclpy_implementation_rmw_agnosticmodule);
}
