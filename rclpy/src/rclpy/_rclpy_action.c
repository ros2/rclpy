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

#include <rcl/error_handling.h>
#include <rcl_action/rcl_action.h>

/// Destroy an rcl_action entity
/**
 * Raises RuntimeError on failure
 *
 * \param[in] pyentity Capsule pointing to the entity to destroy
 * \param[in] pynode Capsule pointing to the node the action client was added to
 */
static PyObject *
rclpy_action_destroy_entity(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyentity;
  PyObject * pynode;

  if (!PyArg_ParseTuple(args, "OO", &pyentity, &pynode)) {
    return NULL;
  }

  if (!PyCapsule_CheckExact(pyentity)) {
    PyErr_Format(PyExc_ValueError, "Object is not a capsule");
    return NULL;
  }

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");

  rcl_ret_t ret;
  if (PyCapsule_IsValid(pyentity, "rcl_action_client_t")) {
    rcl_action_client_t * action_client =
      (rcl_action_client_t *)PyCapsule_GetPointer(pyentity, "rcl_action_client_t");
    ret = rcl_action_client_fini(action_client, node);
    PyMem_Free(action_client);
  } else if (PyCapsule_IsValid(pyentity, "rcl_action_server_t")) {
    rcl_action_server_t * action_server =
      (rcl_action_server_t *)PyCapsule_GetPointer(pyentity, "rcl_action_server_t");
    ret = rcl_action_server_fini(action_server, node);
    PyMem_Free(action_server);
  } else {
    ret = RCL_RET_ERROR;  // to avoid a linter warning
    PyErr_Format(PyExc_RuntimeError, "'%s' is not a known entity", PyCapsule_GetName(pyentity));
    return NULL;
  }

  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to fini '%s': %s", PyCapsule_GetName(pyentity), rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }

  if (PyCapsule_SetPointer(pyentity, Py_None)) {
    // exception set by PyCapsule_SetPointer
    return NULL;
  }

  Py_RETURN_NONE;
}


#define OPTIONS_COPY_QOS_PROFILE(Options, Profile) \
  if (PyCapsule_IsValid(py ## Profile, "rmw_qos_profile_t")) { \
    void * p = PyCapsule_GetPointer(py ## Profile, "rmw_qos_profile_t"); \
    rmw_qos_profile_t * qos_profile = (rmw_qos_profile_t *)p; \
    Options.Profile = *qos_profile; \
    PyMem_Free(p); \
    if (PyCapsule_SetPointer(py ## Profile, Py_None)) { \
      /* exception set by PyCapsule_SetPointer */ \
      return NULL; \
    } \
  }

/// Create an action client
/**
 * This function will create an action client for the given action name.
 * This client will use the typesupport defined in the action module
 * provided as pyaction_type to send messages over the wire.
 *
 * On a successful call a list with two elements is returned:
 *
 * - a Capsule pointing to the pointer of the created rcl_action_client_t * structure
 * - an integer representing the memory address of the created rcl_action_client_t
 *
 * Raises ValueError if the capsules are not the correct types
 * Raises RuntimeError if the action client could not be created
 *
 * \remark Call rclpy_action_destroy_entity() to destroy an action client
 * \param[in] pynode Capsule pointing to the node to add the action client to
 * \param[in] pyaction_type Action module associated with the action client
 * \param[in] pyaction_name Python object containing the action name
 * \param[in] pyqos_profile QoSProfile Python object for this action client
 * \return capsule and memory address, or
 * \return NULL on failure
 */
static PyObject *
rclpy_action_create_client(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pynode;
  PyObject * pyaction_type;
  PyObject * pyaction_name;
  PyObject * pygoal_service_qos;
  PyObject * pyresult_service_qos;
  PyObject * pycancel_service_qos;
  PyObject * pyfeedback_topic_qos;
  PyObject * pystatus_topic_qos;

  int parse_tuple_result = PyArg_ParseTuple(
    args,
    "OOOO",
    &pynode,
    &pyaction_type,
    &pyaction_name,
    &pygoal_service_qos,
    &pyresult_service_qos,
    &pycancel_service_qos,
    &pyfeedback_topic_qos,
    &pystatus_topic_qos);

  if (!parse_tuple_result) {
    return NULL;
  }

  if (!PyUnicode_Check(pyaction_name)) {
    PyErr_Format(PyExc_TypeError, "Argument pyaction_name is not a PyUnicode object");
    return NULL;
  }

  char * action_name = (char *)PyUnicode_1BYTE_DATA(pyaction_name);

  rcl_node_t * node = (rcl_node_t *)PyCapsule_GetPointer(pynode, "rcl_node_t");

  PyObject * pymetaclass = PyObject_GetAttrString(pyaction_type, "__class__");

  PyObject * pyts = PyObject_GetAttrString(pymetaclass, "_TYPE_SUPPORT");

  rosidl_action_type_support_t * ts =
    (rosidl_action_type_support_t *)PyCapsule_GetPointer(pyts, NULL);

  rcl_action_client_options_t action_client_ops = rcl_action_client_get_default_options();

  OPTIONS_COPY_QOS_PROFILE(action_client_ops, goal_service_qos);
  OPTIONS_COPY_QOS_PROFILE(action_client_ops, result_service_qos);
  OPTIONS_COPY_QOS_PROFILE(action_client_ops, cancel_service_qos);
  OPTIONS_COPY_QOS_PROFILE(action_client_ops, feedback_topic_qos);
  OPTIONS_COPY_QOS_PROFILE(action_client_ops, status_topic_qos);

  rcl_action_client_t * action_client =
    (rcl_action_client_t *)PyMem_Malloc(sizeof(rcl_action_client_t));
  *action_client = rcl_action_get_zero_initialized_client();
  rcl_ret_t ret = rcl_action_client_init(
    action_client,
    node,
    ts,
    action_name,
    &action_client_ops);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_ACTION_NAME_INVALID) {
      PyErr_Format(PyExc_ValueError,
        "Failed to create action client due to invalid topic name '%s': %s",
        action_name, rcl_get_error_string().str);
    } else {
      PyErr_Format(PyExc_RuntimeError,
        "Failed to create action client: %s", rcl_get_error_string().str);
    }
    PyMem_Free(action_client);
    rcl_reset_error();
    return NULL;
  }

  PyObject * pylist = PyList_New(2);
  PyList_SET_ITEM(pylist, 0, PyCapsule_New(action_client, "rcl_action_client_t", NULL));
  PyList_SET_ITEM(pylist, 1, PyLong_FromUnsignedLongLong((uint64_t)&action_client->impl));

  return pylist;
}

/// Define the public methods of this module
static PyMethodDef rclpy_action_methods[] = {
  {
    "rclpy_action_destroy_entity", rclpy_action_destroy_entity, METH_VARARGS,
    "Destroy a rclpy_action entity."
  },

  {
    "rclpy_action_create_client", rclpy_action_create_client, METH_VARARGS,
    "Create an action client."
  },

  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rclpy_action__doc__,
  "ROS 2 Python Action library.");

/// Define the Python module
static struct PyModuleDef _rclpy_action_module = {
  PyModuleDef_HEAD_INIT,
  "_rclpy_action",
  rclpy_action__doc__,
  -1,   /* -1 means that the module keeps state in global variables */
  rclpy_action_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy_action(void)
{
  return PyModule_Create(&_rclpy_action_module);
}
