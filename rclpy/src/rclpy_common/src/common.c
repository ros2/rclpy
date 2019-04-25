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

#include "rcl/error_handling.h"

#include "rclpy_common/common.h"

/**
 * Mirrors the struct rmw_qos_profile_t from rmw/types.h
 */
typedef struct rclpy_qos_profile
{
  PyObject * depth;
  PyObject * history;
  PyObject * reliability;
  PyObject * durability;
  PyObject * lifespan;
  PyObject * deadline;
  PyObject * liveliness;
  PyObject * liveliness_lease_duration;
  PyObject * avoid_ros_namespace_conventions;
} rclpy_qos_profile_t;

void
init_rclpy_qos_profile(rclpy_qos_profile_t * rclpy_profile)
{
  memset(rclpy_profile, 0, sizeof(*rclpy_profile));
}

void
cleanup_rclpy_qos_profile(rclpy_qos_profile_t * profile)
{
  Py_XDECREF(profile->depth);
  Py_XDECREF(profile->history);
  Py_XDECREF(profile->reliability);
  Py_XDECREF(profile->durability);
  Py_XDECREF(profile->lifespan);
  Py_XDECREF(profile->deadline);
  Py_XDECREF(profile->liveliness);
  Py_XDECREF(profile->liveliness_lease_duration);
  Py_XDECREF(profile->avoid_ros_namespace_conventions);
}

bool
rclpy_names_and_types_fini(rcl_names_and_types_t * names_and_types)
{
  if (!names_and_types) {
    return true;
  }
  rcl_ret_t ret = rcl_names_and_types_fini(names_and_types);
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to destroy rcl_names_and_types_t: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return false;
  }
  return true;
}

PyObject *
rclpy_convert_to_py_names_and_types(rcl_names_and_types_t * names_and_types)
{
  if (!names_and_types) {
    return NULL;
  }

  PyObject * pynames_and_types = PyList_New(names_and_types->names.size);
  if (!pynames_and_types) {
    return NULL;
  }

  size_t i;
  for (i = 0; i < names_and_types->names.size; ++i) {
    PyObject * pytuple = PyTuple_New(2);
    if (!pytuple) {
      Py_DECREF(pynames_and_types);
      return NULL;
    }
    PyObject * pyname = PyUnicode_FromString(names_and_types->names.data[i]);
    if (!pyname) {
      Py_DECREF(pynames_and_types);
      Py_DECREF(pytuple);
      return NULL;
    }
    PyTuple_SET_ITEM(pytuple, 0, pyname);
    PyObject * pytypes_list = PyList_New(names_and_types->types[i].size);
    if (!pytypes_list) {
      Py_DECREF(pynames_and_types);
      Py_DECREF(pytuple);
      return NULL;
    }
    size_t j;
    for (j = 0; j < names_and_types->types[i].size; ++j) {
      PyObject * pytype = PyUnicode_FromString(names_and_types->types[i].data[j]);
      if (!pytype) {
        Py_DECREF(pynames_and_types);
        Py_DECREF(pytuple);
        Py_DECREF(pytypes_list);
        return NULL;
      }
      PyList_SET_ITEM(pytypes_list, j, pytype);
    }
    PyTuple_SET_ITEM(pytuple, 1, pytypes_list);
    PyList_SET_ITEM(pynames_and_types, i, pytuple);
  }
  return pynames_and_types;
}

static
PyObject *
_convert_rmw_time_to_py_duration(const rmw_time_t * duration)
{
  uint64_t total_nanoseconds = RCUTILS_S_TO_NS(duration->sec) + duration->nsec;
  PyObject * pyduration_module = NULL;
  PyObject * pyduration_class = NULL;
  PyObject * args = NULL;
  PyObject * kwargs = NULL;
  PyObject * pyduration_object = NULL;

  pyduration_module = PyImport_ImportModule("rclpy.duration");
  if (!pyduration_module) {
    goto cleanup;
  }
  pyduration_class = PyObject_GetAttrString(pyduration_module, "Duration");
  if (!pyduration_class) {
    goto cleanup;
  }
  args = PyTuple_New(0);
  if (!args) {
    goto cleanup;
  }
  kwargs = Py_BuildValue("{s:l}", "nanoseconds", total_nanoseconds);
  if (!kwargs) {
    goto cleanup;
  }
  pyduration_object = PyObject_Call(pyduration_class, args, kwargs);
  if (!pyduration_object) {
    goto cleanup;
  }

cleanup:
  Py_XDECREF(pyduration_module);
  Py_XDECREF(pyduration_class);
  Py_XDECREF(args);
  Py_XDECREF(kwargs);
  return pyduration_object;
}

PyObject *
rclpy_common_convert_to_py_qos_policy(const rmw_qos_profile_t * qos_profile)
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
  rclpy_qos_profile_t rclpy_qos;
  init_rclpy_qos_profile(&rclpy_qos);

  int set_result = -1;

  rclpy_qos.depth = PyLong_FromSize_t(qos_profile->depth);
  if (!rclpy_qos.depth) {
    Py_DECREF(pyqos_profile);
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.history = PyLong_FromUnsignedLong(qos_profile->history);
  if (!rclpy_qos.history) {
    Py_DECREF(pyqos_profile);
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.reliability = PyLong_FromUnsignedLong(qos_profile->reliability);
  if (!rclpy_qos.reliability) {
    Py_DECREF(pyqos_profile);
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.durability = PyLong_FromUnsignedLong(qos_profile->durability);
  if (!rclpy_qos.durability) {
    Py_DECREF(pyqos_profile);
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.lifespan = _convert_rmw_time_to_py_duration(&qos_profile->lifespan);
  if (!rclpy_qos.lifespan) {
    Py_DECREF(pyqos_profile);
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.deadline = _convert_rmw_time_to_py_duration(&qos_profile->deadline);
  if (!rclpy_qos.deadline) {
    Py_DECREF(pyqos_profile);
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.liveliness = PyLong_FromUnsignedLong(qos_profile->liveliness);
  if (!rclpy_qos.liveliness) {
    Py_DECREF(pyqos_profile);
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.liveliness_lease_duration = _convert_rmw_time_to_py_duration(
    &qos_profile->liveliness_lease_duration);
  if (!rclpy_qos.liveliness_lease_duration) {
    Py_DECREF(pyqos_profile);
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.avoid_ros_namespace_conventions =
    PyBool_FromLong(qos_profile->avoid_ros_namespace_conventions);
  if (!rclpy_qos.avoid_ros_namespace_conventions) {
    Py_DECREF(pyqos_profile);
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  // A success returns 0, and a failure returns -1
  set_result = 0;
  set_result += PyObject_SetAttrString(pyqos_profile, "depth", rclpy_qos.depth);
  set_result += PyObject_SetAttrString(pyqos_profile, "history", rclpy_qos.history);
  set_result += PyObject_SetAttrString(pyqos_profile, "reliability", rclpy_qos.reliability);
  set_result += PyObject_SetAttrString(pyqos_profile, "durability", rclpy_qos.durability);
  set_result += PyObject_SetAttrString(pyqos_profile, "lifespan", rclpy_qos.lifespan);
  set_result += PyObject_SetAttrString(pyqos_profile, "deadline", rclpy_qos.deadline);
  set_result += PyObject_SetAttrString(pyqos_profile, "liveliness", rclpy_qos.liveliness);
  set_result += PyObject_SetAttrString(pyqos_profile,
      "liveliness_lease_duration", rclpy_qos.liveliness_lease_duration);
  set_result += PyObject_SetAttrString(pyqos_profile,
      "avoid_ros_namespace_conventions", rclpy_qos.avoid_ros_namespace_conventions);

  cleanup_rclpy_qos_profile(&rclpy_qos);

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
    (**destroy_ros_message)(message);
    return NULL;
  }

  convert_from_py_signature * convert = get_capsule_pointer(
    pymetaclass, "_CONVERT_FROM_PY");
  Py_DECREF(pymetaclass);
  if (!convert) {
    (**destroy_ros_message)(message);
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
