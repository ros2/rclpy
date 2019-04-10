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

#include "rclpy_common/common.h"

/**
 * Mirrors the struct rmw_qos_profile_t from rmw/types.h,
 * with the addition of member `profile` to to hold the Python QoSProfile
 */
typedef struct rclpy_qos_profile
{
  PyObject * profile;
  PyObject * depth;
  PyObject * history;
  PyObject * reliability;
  PyObject * durability;
  PyObject * avoid_ros_namespace_conventions;
} rclpy_qos_profile_t;

void
init_rclpy_qos_profile(rclpy_qos_profile_t * rclpy_profile, PyObject * py_profile)
{
  memset(rclpy_profile, 0, sizeof(*rclpy_profile));
  rclpy_profile->profile = py_profile;
}

void
cleanup_rclpy_qos_profile(rclpy_qos_profile_t * profile, bool profile_also)
{
  if (profile_also) {
    Py_XDECREF(profile->profile);
  }
  Py_XDECREF(profile->depth);
  Py_XDECREF(profile->history);
  Py_XDECREF(profile->reliability);
  Py_XDECREF(profile->durability);
  Py_XDECREF(profile->avoid_ros_namespace_conventions);
}

PyObject *
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

  // start qos setting
  rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)profile;
  rclpy_qos_profile_t rclpy_qos;
  init_rclpy_qos_profile(&rclpy_qos, pyqos_profile);

  int set_result = -1;

  rclpy_qos.depth = PyLong_FromSize_t(qos_profile->depth);
  if (!rclpy_qos.depth) {
    cleanup_rclpy_qos_profile(&rclpy_qos, true);
    return NULL;
  }

  rclpy_qos.history = PyLong_FromUnsignedLong(qos_profile->history);
  if (!rclpy_qos.history) {
    cleanup_rclpy_qos_profile(&rclpy_qos, true);
    return NULL;
  }

  rclpy_qos.reliability = PyLong_FromUnsignedLong(qos_profile->reliability);
  if (!rclpy_qos.reliability) {
    cleanup_rclpy_qos_profile(&rclpy_qos, true);
    return NULL;
  }

  rclpy_qos.durability = PyLong_FromUnsignedLong(qos_profile->durability);
  if (!rclpy_qos.durability) {
    cleanup_rclpy_qos_profile(&rclpy_qos, true);
    return NULL;
  }

  rclpy_qos.avoid_ros_namespace_conventions =
    PyBool_FromLong(qos_profile->avoid_ros_namespace_conventions);
  if (!rclpy_qos.avoid_ros_namespace_conventions) {
    cleanup_rclpy_qos_profile(&rclpy_qos, true);
    return NULL;
  }

  // A success returns 0, and a failure returns -1
  set_result = 0;
  set_result += PyObject_SetAttrString(pyqos_profile, "depth", rclpy_qos.depth);
  set_result += PyObject_SetAttrString(pyqos_profile, "history", rclpy_qos.history);
  set_result += PyObject_SetAttrString(pyqos_profile, "reliability", rclpy_qos.reliability);
  set_result += PyObject_SetAttrString(pyqos_profile, "durability", rclpy_qos.durability);
  set_result += PyObject_SetAttrString(pyqos_profile,
      "avoid_ros_namespace_conventions", rclpy_qos.avoid_ros_namespace_conventions);

  cleanup_rclpy_qos_profile(&rclpy_qos, false);

  if (0 != set_result) {
    Py_DECREF(pyqos_profile);
    return NULL;
  }
  return pyqos_profile;
}

void *
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

void *
rclpy_create_from_py(PyObject * pymessage, destroy_ros_message_signature ** destroy_ros_message)
{
  PyObject * pymetaclass = PyObject_GetAttrString(pymessage, "__class__");
  if (!pymetaclass) {
    return NULL;
  }

  create_ros_message_signature * create_ros_message = get_capsule_pointer(
    pymetaclass, "_CREATE_ROS_MESSAGE");
  if (!create_ros_message) {
    Py_DECREF(pymetaclass);
    return NULL;
  }

  *destroy_ros_message = get_capsule_pointer(
    pymetaclass, "_DESTROY_ROS_MESSAGE");
  Py_DECREF(pymetaclass);
  if (!destroy_ros_message) {
    return NULL;
  }

  void * message = create_ros_message();
  if (!message) {
    return PyErr_NoMemory();
  }
  return message;
}

void *
rclpy_convert_from_py(PyObject * pymessage, destroy_ros_message_signature ** destroy_ros_message)
{
  void * message = rclpy_create_from_py(pymessage, destroy_ros_message);
  if (!message) {
    return NULL;
  }

  PyObject * pymetaclass = PyObject_GetAttrString(pymessage, "__class__");
  if (!pymetaclass) {
    return NULL;
  }

  convert_from_py_signature * convert = get_capsule_pointer(
    pymetaclass, "_CONVERT_FROM_PY");
  Py_DECREF(pymetaclass);
  if (!convert) {
    return NULL;
  }

  if (!convert(pymessage, message)) {
    (**destroy_ros_message)(message);
    return NULL;
  }
  return message;
}

PyObject *
rclpy_convert_to_py(void * message, PyObject * pyclass)
{
  PyObject * pymetaclass = PyObject_GetAttrString(pyclass, "__class__");
  if (!pymetaclass) {
    return NULL;
  }
  convert_to_py_signature * convert = get_capsule_pointer(
    pymetaclass, "_CONVERT_TO_PY");
  Py_DECREF(pymetaclass);
  if (!convert) {
    return NULL;
  }
  return convert(message);
}
