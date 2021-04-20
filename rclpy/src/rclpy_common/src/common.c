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

#include <assert.h>

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
  assert(rclpy_profile);
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

void *
rclpy_common_get_type_support(PyObject * pymsg_type)
{
  PyObject * pymetaclass = PyObject_GetAttrString(pymsg_type, "__class__");
  if (!pymetaclass) {
    return NULL;
  }

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");
  Py_DECREF(pymetaclass);
  if (!pyts) {
    return NULL;
  }

  void * ts = PyCapsule_GetPointer(pyts, NULL);
  Py_DECREF(pyts);
  return ts;
}

static
PyObject *
_convert_rmw_time_to_py_duration(const rmw_time_t * duration)
{
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
  kwargs = Py_BuildValue("{sKsK}", "seconds", duration->sec, "nanoseconds", duration->nsec);
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
rclpy_common_convert_to_qos_dict(const rmw_qos_profile_t * qos_profile)
{
  // Convert rmw members to Python objects
  rclpy_qos_profile_t rclpy_qos;
  init_rclpy_qos_profile(&rclpy_qos);

  rclpy_qos.depth = PyLong_FromSize_t(qos_profile->depth);
  if (!rclpy_qos.depth) {
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.history = PyLong_FromUnsignedLong(qos_profile->history);
  if (!rclpy_qos.history) {
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.reliability = PyLong_FromUnsignedLong(qos_profile->reliability);
  if (!rclpy_qos.reliability) {
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.durability = PyLong_FromUnsignedLong(qos_profile->durability);
  if (!rclpy_qos.durability) {
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.lifespan = _convert_rmw_time_to_py_duration(&qos_profile->lifespan);
  if (!rclpy_qos.lifespan) {
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.deadline = _convert_rmw_time_to_py_duration(&qos_profile->deadline);
  if (!rclpy_qos.deadline) {
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.liveliness = PyLong_FromUnsignedLong(qos_profile->liveliness);
  if (!rclpy_qos.liveliness) {
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.liveliness_lease_duration = _convert_rmw_time_to_py_duration(
    &qos_profile->liveliness_lease_duration);
  if (!rclpy_qos.liveliness_lease_duration) {
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  rclpy_qos.avoid_ros_namespace_conventions =
    PyBool_FromLong(qos_profile->avoid_ros_namespace_conventions);
  if (!rclpy_qos.avoid_ros_namespace_conventions) {
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  // Create dictionary to hold QoSProfile keyword arguments
  PyObject * pyqos_kwargs = PyDict_New();
  if (!pyqos_kwargs) {
    cleanup_rclpy_qos_profile(&rclpy_qos);
    return NULL;
  }

  // Populate keyword arguments for QoSProfile object
  // A success returns 0, and a failure returns -1
  int set_result = 0;
  set_result += PyDict_SetItemString(pyqos_kwargs, "depth", rclpy_qos.depth);
  set_result += PyDict_SetItemString(pyqos_kwargs, "history", rclpy_qos.history);
  set_result += PyDict_SetItemString(pyqos_kwargs, "reliability", rclpy_qos.reliability);
  set_result += PyDict_SetItemString(pyqos_kwargs, "durability", rclpy_qos.durability);
  set_result += PyDict_SetItemString(pyqos_kwargs, "lifespan", rclpy_qos.lifespan);
  set_result += PyDict_SetItemString(pyqos_kwargs, "deadline", rclpy_qos.deadline);
  set_result += PyDict_SetItemString(pyqos_kwargs, "liveliness", rclpy_qos.liveliness);
  set_result += PyDict_SetItemString(
    pyqos_kwargs, "liveliness_lease_duration", rclpy_qos.liveliness_lease_duration);
  set_result += PyDict_SetItemString(
    pyqos_kwargs, "avoid_ros_namespace_conventions", rclpy_qos.avoid_ros_namespace_conventions);
  cleanup_rclpy_qos_profile(&rclpy_qos);
  if (0 != set_result) {
    Py_DECREF(pyqos_kwargs);
    return NULL;
  }

  return pyqos_kwargs;
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
  if (!(*destroy_ros_message)) {
    return NULL;
  }

  void * message = create_ros_message();
  if (!message) {
    PyErr_NoMemory();
    return NULL;
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

PyObject *
_rclpy_convert_to_py_topic_endpoint_info(const rmw_topic_endpoint_info_t * topic_endpoint_info)
{
  PyObject * py_node_name = NULL;
  PyObject * py_node_namespace = NULL;
  PyObject * py_topic_type = NULL;
  PyObject * py_endpoint_type = NULL;
  PyObject * py_endpoint_gid = NULL;
  PyObject * py_qos_profile = NULL;
  PyObject * py_endpoint_info_dict = NULL;

  py_node_name = PyUnicode_FromString(topic_endpoint_info->node_name);
  if (!py_node_name) {
    goto fail;
  }
  py_node_namespace = PyUnicode_FromString(topic_endpoint_info->node_namespace);
  if (!py_node_namespace) {
    goto fail;
  }
  py_topic_type = PyUnicode_FromString(topic_endpoint_info->topic_type);
  if (!py_topic_type) {
    goto fail;
  }
  py_endpoint_type = PyLong_FromUnsignedLong(topic_endpoint_info->endpoint_type);
  if (!py_endpoint_type) {
    goto fail;
  }

  py_endpoint_gid = PyList_New(RMW_GID_STORAGE_SIZE);
  if (!py_endpoint_gid) {
    goto fail;
  }
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; i++) {
    PyObject * py_val_at_index = PyLong_FromUnsignedLong(topic_endpoint_info->endpoint_gid[i]);
    if (!py_val_at_index) {
      goto fail;
    }
    PyList_SET_ITEM(py_endpoint_gid, i, py_val_at_index);
  }

  py_qos_profile = rclpy_common_convert_to_qos_dict(&topic_endpoint_info->qos_profile);
  if (!py_qos_profile) {
    goto fail;
  }

  // Create dictionary that represents rmw_topic_endpoint_info_t
  py_endpoint_info_dict = PyDict_New();
  if (!py_endpoint_info_dict) {
    goto fail;
  }
  // Populate keyword arguments
  // A success returns 0, and a failure returns -1
  int set_result = 0;
  set_result += PyDict_SetItemString(py_endpoint_info_dict, "node_name", py_node_name);
  set_result += PyDict_SetItemString(py_endpoint_info_dict, "node_namespace", py_node_namespace);
  set_result += PyDict_SetItemString(py_endpoint_info_dict, "topic_type", py_topic_type);
  set_result += PyDict_SetItemString(py_endpoint_info_dict, "endpoint_type", py_endpoint_type);
  set_result += PyDict_SetItemString(py_endpoint_info_dict, "endpoint_gid", py_endpoint_gid);
  set_result += PyDict_SetItemString(py_endpoint_info_dict, "qos_profile", py_qos_profile);
  if (set_result != 0) {
    goto fail;
  }
  return py_endpoint_info_dict;

fail:
  Py_XDECREF(py_node_name);
  Py_XDECREF(py_node_namespace);
  Py_XDECREF(py_topic_type);
  Py_XDECREF(py_endpoint_type);
  Py_XDECREF(py_endpoint_gid);
  Py_XDECREF(py_qos_profile);
  Py_XDECREF(py_endpoint_info_dict);
  return NULL;
}

PyObject *
rclpy_convert_to_py_topic_endpoint_info_list(const rmw_topic_endpoint_info_array_t * info_array)
{
  if (!info_array) {
    return NULL;
  }

  PyObject * py_info_array = PyList_New(info_array->size);
  if (!py_info_array) {
    return NULL;
  }

  for (size_t i = 0; i < info_array->size; ++i) {
    rmw_topic_endpoint_info_t topic_endpoint_info = info_array->info_array[i];
    PyObject * py_endpoint_info_dict = _rclpy_convert_to_py_topic_endpoint_info(
      &topic_endpoint_info);
    if (!py_endpoint_info_dict) {
      Py_DECREF(py_info_array);
      return NULL;
    }
    // add this dict to the list
    PyList_SET_ITEM(py_info_array, i, py_endpoint_info_dict);
  }
  return py_info_array;
}
