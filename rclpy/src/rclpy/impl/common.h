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
#ifndef RCLPY__IMPL__COMMON_H_
#define RCLPY__IMPL__COMMON_H_

#include <Python.h>

#include <rmw/types.h>

typedef void * create_ros_message_signature (void);
typedef void destroy_ros_message_signature (void *);
typedef bool convert_from_py_signature (PyObject *, void *);
typedef PyObject * convert_to_py_signature (void *);

/// Convert a C rmw_qos_profile_t into a Python QoSProfile object
/**
 * \param[in] void pointer to a rmw_qos_profile_t structure
 * \return QoSProfile object
 */
static PyObject *
rclpy_convert_to_py_qos_policy(void * profile)
{
  PyObject * pyqos_module = PyImport_ImportModule("rclpy.qos");
  if (!pyqos_module) {
    return NULL;
  }

  PyObject * pyqos_policy_class = PyObject_GetAttrString(pyqos_module, "QoSProfile");
  Py_DECREF(pyqos_module);
  if (!pyqos_policy_class) {
    return NULL;
  }

  PyObject * pyqos_profile = PyObject_CallObject(pyqos_policy_class, NULL);
  Py_DECREF(pyqos_policy_class);
  if (!pyqos_profile) {
    return NULL;
  }

  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)profile;

  PyObject * pyqos_depth = PyLong_FromSize_t(qos_profile->depth);
  if (!pyqos_depth) {
    Py_DECREF(pyqos_profile);
    return NULL;
  }

  PyObject * pyqos_history = PyLong_FromUnsignedLong(qos_profile->history);
  if (!pyqos_history) {
    Py_DECREF(pyqos_profile);
    Py_DECREF(pyqos_depth);
    return NULL;
  }

  PyObject * pyqos_reliability = PyLong_FromUnsignedLong(qos_profile->reliability);
  if (!pyqos_reliability) {
    Py_DECREF(pyqos_profile);
    Py_DECREF(pyqos_depth);
    Py_DECREF(pyqos_history);
    return NULL;
  }

  PyObject * pyqos_durability = PyLong_FromUnsignedLong(qos_profile->durability);
  if (!pyqos_durability) {
    Py_DECREF(pyqos_profile);
    Py_DECREF(pyqos_depth);
    Py_DECREF(pyqos_history);
    Py_DECREF(pyqos_reliability);
    return NULL;
  }

  PyObject * pyqos_avoid_ros_namespace_conventions =
    PyBool_FromLong(qos_profile->avoid_ros_namespace_conventions);
  if (!pyqos_avoid_ros_namespace_conventions) {
    Py_DECREF(pyqos_profile);
    Py_DECREF(pyqos_depth);
    Py_DECREF(pyqos_history);
    Py_DECREF(pyqos_reliability);
    Py_DECREF(pyqos_durability);
    return NULL;
  }

  // A success returns 0, and a failure returns -1
  int set_result = 0;
  set_result += PyObject_SetAttrString(pyqos_profile, "depth", pyqos_depth);
  set_result += PyObject_SetAttrString(pyqos_profile, "history", pyqos_history);
  set_result += PyObject_SetAttrString(pyqos_profile, "reliability", pyqos_reliability);
  set_result += PyObject_SetAttrString(pyqos_profile, "durability", pyqos_durability);
  set_result += PyObject_SetAttrString(pyqos_profile,
      "avoid_ros_namespace_conventions", pyqos_avoid_ros_namespace_conventions);

  Py_DECREF(pyqos_depth);
  Py_DECREF(pyqos_history);
  Py_DECREF(pyqos_reliability);
  Py_DECREF(pyqos_durability);
  Py_DECREF(pyqos_avoid_ros_namespace_conventions);

  if (0 != set_result) {
    Py_DECREF(pyqos_profile);
    return NULL;
  }
  return pyqos_profile;
}

static void *
get_capsule_pointer(PyObject * pymetaclass, const char * attr)
{
  PyObject * pyattr = PyObject_GetAttrString(pymetaclass, attr);
  if (!pyattr) {
    return NULL;
  }
  void * ptr = PyCapsule_GetPointer(pyattr, NULL);
  Py_DECREF(pyattr);
  return ptr;
}
#endif  // RCLPY__IMPL__COMMON_H_
