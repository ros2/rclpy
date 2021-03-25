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

#include <rcl/error_handling.h>
#include <rcl/expand_topic_name.h>
#include <rcl/graph.h>
#include <rcl/logging.h>
#include <rcl/node.h>
#include <rcl/publisher.h>
#include <rcl/rcl.h>
#include <rcl/remap.h>
#include <rcl/time.h>
#include <rcl/validate_topic_name.h>
#include <rcl/init_options.h>
#include <rcl/context.h>
#include <rcl_interfaces/msg/parameter_type.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcutils/allocator.h>
#include <rcutils/format_string.h>
#include <rcutils/macros.h>
#include <rcutils/strdup.h>
#include <rcutils/types.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rmw/topic_endpoint_info_array.h>
#include <rmw/types.h>
#include <rmw/validate_full_topic_name.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include "./detail/thread_safe_logging_output_handler.h"
#include "./detail/execute_with_logging_mutex.h"
#include "rclpy_common/common.h"
#include "rclpy_common/handle.h"

typedef struct
{
  PyObject * NodeNameNonExistentError;
  PyObject * RCLError;
  PyObject * RCLInvalidROSArgsError;
  PyObject * UnknownROSArgsError;
  PyObject * UnsupportedEventTypeError;
} rclpy_module_state_t;

int pyobj_to_long(PyObject * obj, void * i)
{
  PY_LONG_LONG tmp;
  PY_LONG_LONG * val = (PY_LONG_LONG *)i;

  if (obj == Py_None) {
    return 1;  // Return success if object is None
  }

  if (PyLong_Check(obj)) {
    tmp = PyLong_AsLongLong(obj);
    if (PyErr_Occurred()) {
      return 0;  // Conversion failed.
    } else {
      *val = tmp;
      return 1;  // Successful conversion.
    }
  }

  PyErr_SetString(PyExc_TypeError, "PyObject must be long or None.");
  return 0;  // Conversion failed
}

/// Initialize rcl logging
/**
 * Raises RuntimeError if rcl logging could not be initialized
 */
static PyObject *
rclpy_logging_configure_impl(PyObject * module, PyObject * args)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }
  // Expect one argument, a context.
  PyObject * pycontext;
  if (!PyArg_ParseTuple(args, "O", &pycontext)) {
    // Exception raised
    return NULL;
  }
  rcl_context_t * context = rclpy_handle_get_pointer_from_capsule(pycontext, "rcl_context_t");
  if (!context) {
    return NULL;
  }
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_logging_configure_with_output_handler(
    &context->global_arguments,
    &allocator,
    rclpy_detail_thread_safe_logging_output_handler);
  if (RCL_RET_OK != ret) {
    PyErr_Format(
      module_state->RCLError,
      "Failed to initialize logging: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// See rclpy_logging_configure_impl above.
static PyObject *
rclpy_logging_configure(PyObject * self, PyObject * args)
{
  return rclpy_detail_execute_with_logging_mutex(rclpy_logging_configure_impl, self, args);
}

/// Finalize rcl logging
/**
 * Produces a RuntimeWarning if rcl logging could not be finalized
 */
static PyObject *
rclpy_logging_fini_impl(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  rcl_ret_t ret = rcl_logging_fini();
  if (RCL_RET_OK != ret) {
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning,
      stack_level,
      "Failed to fini logging: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
    return NULL;
  }
  Py_RETURN_NONE;
}

/// See rclpy_logging_fini_impl above.
static PyObject *
rclpy_logging_fini(PyObject * self, PyObject * args)
{
  return rclpy_detail_execute_with_logging_mutex(rclpy_logging_fini_impl, self, args);
}

/// Handle destructor for node
static void
_rclpy_destroy_node_impl(void * p)
{
  rcl_node_t * node = p;
  if (!node) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "_rclpy_destroy_node got a NULL pointer");
    return;
  }

  rcl_ret_t ret = rcl_node_fini(node);
  if (RCL_RET_OK != ret) {
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini node: %s",
      rcl_get_error_string().str);
  }
  PyMem_Free(node);
}

/// See _rclpy_destroy_node_impl above.
static void
_rclpy_destroy_node(void * p)
{
  rclpy_detail_execute_with_logging_mutex2(_rclpy_destroy_node_impl, p);
}

/// Create a node
/**
 * Raises ValueError if the node name or namespace is invalid
 * Raises RuntimeError if the node could not be initialized for an unexpected reason
 * Raises MemoryError if memory could not be allocated for the node
 *
 * \param[in] node_name string name of the node to be created
 * \param[in] namespace string namespace for the node
 * \return Capsule of the pointer to the created rcl_node_t * structure, or
 * \return NULL on failure
 */
static PyObject *
rclpy_create_node_impl(PyObject * module, PyObject * args)
{
  rcl_ret_t ret;
  const char * node_name;
  const char * namespace_;
  PyObject * pycontext;
  PyObject * py_cli_args;
  int use_global_arguments;
  int enable_rosout;

  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(module);
  if (!module_state) {
    // exception already raised
    return NULL;
  }

  if (!PyArg_ParseTuple(
      args, "ssOOpp",
      &node_name,
      &namespace_,
      &pycontext,
      &py_cli_args,
      &use_global_arguments,
      &enable_rosout))
  {
    return NULL;
  }

  rclpy_handle_t * context_handle = PyCapsule_GetPointer(pycontext, "rcl_context_t");
  if (!context_handle) {
    return NULL;
  }
  rcl_context_t * context = _rclpy_handle_get_pointer(context_handle);
  if (!context) {
    return NULL;
  }

  rcl_arguments_t arguments = rcl_get_zero_initialized_arguments();
  ret = _rclpy_parse_args(module_state, py_cli_args, &arguments);
  if (RCL_RET_OK != ret) {
    // exception set
    return NULL;
  }

  rcl_node_t * node = PyMem_Malloc(sizeof(rcl_node_t));
  if (!node) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for node");
    return NULL;
  }
  *node = rcl_get_zero_initialized_node();
  rcl_node_options_t options = rcl_node_get_default_options();
  options.use_global_arguments = use_global_arguments;
  options.arguments = arguments;
  options.enable_rosout = enable_rosout;
  ret = rcl_node_init(node, node_name, namespace_, context, &options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_BAD_ALLOC) {
      PyErr_Format(
        PyExc_MemoryError, "%s", rcl_get_error_string().str);
    } else if (ret == RCL_RET_NODE_INVALID_NAME) {
      PyErr_Format(
        PyExc_ValueError, "invalid node name: %s", rcl_get_error_string().str);
    } else if (ret == RCL_RET_NODE_INVALID_NAMESPACE) {
      PyErr_Format(
        PyExc_ValueError,
        "invalid node namespace: %s", rcl_get_error_string().str);
    } else {
      PyErr_Format(
        module_state->RCLError,
        "Unknown error creating node: %s", rcl_get_error_string().str);
    }
    rcl_reset_error();
    PyMem_Free(node);

    if (RCL_RET_OK != rcl_arguments_fini(&arguments)) {
      // Warn because an exception is already raised
      // Warning should use line number of the current stack frame
      int stack_level = 1;
      PyErr_WarnFormat(
        PyExc_RuntimeWarning, stack_level, "Failed to fini arguments during error handling: %s",
        rcl_get_error_string().str);
      rcl_reset_error();
    }
    return NULL;
  }
  if (RCL_RET_OK != rcl_arguments_fini(&arguments)) {
    // Warn because the node was successfully created
    // Warning should use line number of the current stack frame
    int stack_level = 1;
    PyErr_WarnFormat(
      PyExc_RuntimeWarning, stack_level, "Failed to fini arguments: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
  }

  rclpy_handle_t * node_handle = _rclpy_create_handle(node, _rclpy_destroy_node);
  if (!node_handle) {
    _rclpy_destroy_node(node);
    return NULL;
  }
  _rclpy_handle_add_dependency(node_handle, context_handle);
  if (PyErr_Occurred()) {
    _rclpy_handle_dec_ref(node_handle);
    return NULL;
  }
  PyObject * node_capsule = _rclpy_create_handle_capsule(node_handle, "rcl_node_t");
  if (!node_capsule) {
    _rclpy_handle_dec_ref(node_handle);
    return NULL;
  }
  return node_capsule;
}

/// See rclpy_create_node_impl above.
static PyObject *
rclpy_create_node(PyObject * self, PyObject * args)
{
  return rclpy_detail_execute_with_logging_mutex(rclpy_create_node_impl, self, args);
}

/// Fill a given rmw_time_t with python Duration info, if possible.
/**
  * \param[in] pyobject Python Object that should be a Duration, error checking is done
  * \param[out] out_time Valid destination for time data
  * \return Whether or not conversion succeeded.
  */
bool
_convert_py_duration_to_rmw_time(PyObject * pyobject, rmw_time_t * out_time)
{
  rcl_duration_t * duration = PyCapsule_GetPointer(pyobject, "rcl_duration_t");
  if (!duration) {
    return false;
  }
  *out_time = (rmw_time_t) {
    RCL_NS_TO_S(duration->nanoseconds),
    duration->nanoseconds % (1000LL * 1000LL * 1000LL)
  };
  return true;
}

/// Destructor for a QoSProfile
void
_rclpy_destroy_qos_profile(PyObject * pycapsule)
{
  PyMem_Free(PyCapsule_GetPointer(pycapsule, "rmw_qos_profile_t"));
}

/// Return a Python QoSProfile object
/**
 * This function creates a QoSProfile object from the QoS Policies provided
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
  PyObject * pyqos_lifespan;
  PyObject * pyqos_deadline;
  unsigned PY_LONG_LONG pyqos_liveliness;
  PyObject * pyqos_liveliness_lease_duration;
  int avoid_ros_namespace_conventions;

  if (!PyArg_ParseTuple(
      args, "KKKKOOKOp",
      &pyqos_history,
      &pyqos_depth,
      &pyqos_reliability,
      &pyqos_durability,
      &pyqos_lifespan,
      &pyqos_deadline,
      &pyqos_liveliness,
      &pyqos_liveliness_lease_duration,
      &avoid_ros_namespace_conventions))
  {
    return NULL;
  }

  rmw_qos_profile_t * qos_profile = PyMem_Malloc(sizeof(rmw_qos_profile_t));

  if (!qos_profile) {
    PyErr_Format(PyExc_MemoryError, "Failed to allocate memory for QoS profile");
    return NULL;
  }
  // Set to default so that we don't use uninitialized data if new fields are added
  *qos_profile = rmw_qos_profile_default;
  // Overwrite defaults with passed values
  qos_profile->history = pyqos_history;
  qos_profile->depth = pyqos_depth;
  qos_profile->reliability = pyqos_reliability;
  qos_profile->durability = pyqos_durability;

  if (!_convert_py_duration_to_rmw_time(pyqos_lifespan, &qos_profile->lifespan)) {
    return NULL;
  }
  if (!_convert_py_duration_to_rmw_time(pyqos_deadline, &qos_profile->deadline)) {
    return NULL;
  }

  qos_profile->liveliness = pyqos_liveliness;
  if (
    !_convert_py_duration_to_rmw_time(
      pyqos_liveliness_lease_duration,
      &qos_profile->liveliness_lease_duration))
  {
    return NULL;
  }

  qos_profile->avoid_ros_namespace_conventions = avoid_ros_namespace_conventions;
  PyObject * pyqos_profile = PyCapsule_New(
    qos_profile, "rmw_qos_profile_t", _rclpy_destroy_qos_profile);
  return pyqos_profile;
}

/// Convert a C rmw_qos_profile_t Capsule to a Python QoSProfile object.
/**
  * This function is exposed to facilitate testing profile type conversion.
  * \param[in] pyqos_profile Capsule of rmw_qos_profile_t from rclpy_convert_from_py_qos_policy
  * \return QoSProfile object
  */
static PyObject *
rclpy_convert_to_py_qos_policy(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * pyqos_profile = NULL;
  if (!PyArg_ParseTuple(args, "O", &pyqos_profile)) {
    return NULL;
  }

  rmw_qos_profile_t * profile = PyCapsule_GetPointer(
    pyqos_profile, "rmw_qos_profile_t");
  if (!profile) {
    return NULL;
  }
  return rclpy_common_convert_to_qos_dict(profile);
}

/// Fetch a predefined qos_profile from rmw and convert it to a dictionary with QoSProfile args
/**
 * Raises RuntimeError if there is an rcl error
 *
 * This function takes a string defining a rmw_qos_profile_t and returns the
 * corresponding QoSProfile args in a dictionary
 * \param[in] string with the name of the profile to load
 * \return dictionary with QoSProfile args
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
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_sensor_data);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_default")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_default);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_system_default")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_system_default);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_services_default")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_services_default);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_unknown")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_unknown);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameters")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_parameters);
  } else if (0 == strcmp(pyrmw_profile, "qos_profile_parameter_events")) {
    pyqos_profile = rclpy_common_convert_to_qos_dict(&rmw_qos_profile_parameter_events);
  } else {
    PyErr_Format(
      PyExc_RuntimeError,
      "Requested unknown rmw_qos_profile: '%s'", pyrmw_profile);
    return NULL;
  }
  return pyqos_profile;
}

/// Define the public methods of this module
static PyMethodDef rclpy_methods[] = {
  {
    "rclpy_logging_configure", rclpy_logging_configure, METH_VARARGS,
    "Initialize RCL logging."
  },
  {
    "rclpy_logging_fini", rclpy_logging_fini, METH_NOARGS,
    "Finalize RCL logging."
  },

  {
    "rclpy_create_node", rclpy_create_node, METH_VARARGS,
    "Create a Node."
  },

  {
    "rclpy_convert_from_py_qos_policy", rclpy_convert_from_py_qos_policy, METH_VARARGS,
    "Convert a QoSPolicy Python object into a rmw_qos_profile_t."
  },
  {
    "rclpy_convert_to_py_qos_policy", rclpy_convert_to_py_qos_policy, METH_VARARGS,
    "Convert a rmw_qos_profile_t into a QoSPolicy Python object."
  },
  {
    "rclpy_get_rmw_qos_profile", rclpy_get_rmw_qos_profile, METH_VARARGS,
    "Get QOS profile."
  },

  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rclpy__doc__, "ROS 2 Python client library.");

// Initialization function called after __spec__ and __package__ are set by loader
// Purpose: relative imports work in this function
static int rclpy_exec(PyObject * m)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(m);
  if (!module_state) {
    // exception already raised
    return -1;
  }

  PyObject * globals = PyModule_GetDict(m);
  if (NULL == globals) {
    return -1;
  }

  PyObject * exceptions_module = PyImport_ImportModuleLevel(
    "_rclpy_pybind11", globals, NULL, NULL, 1);
  if (NULL == exceptions_module) {
    return -1;
  }

  module_state->RCLError =
    PyObject_GetAttrString(exceptions_module, "RCLError");
  if (NULL == module_state->RCLError) {
    Py_DECREF(exceptions_module);
    return -1;
  }

  module_state->RCLInvalidROSArgsError =
    PyObject_GetAttrString(exceptions_module, "RCLInvalidROSArgsError");
  if (NULL == module_state->RCLInvalidROSArgsError) {
    Py_DECREF(module_state->RCLError);
    Py_DECREF(exceptions_module);
    return -1;
  }

  module_state->UnknownROSArgsError =
    PyObject_GetAttrString(exceptions_module, "UnknownROSArgsError");
  if (NULL == module_state->UnknownROSArgsError) {
    Py_DECREF(module_state->RCLInvalidROSArgsError);
    Py_DECREF(module_state->RCLError);
    Py_DECREF(exceptions_module);
    return -1;
  }

  module_state->NodeNameNonExistentError =
    PyObject_GetAttrString(exceptions_module, "NodeNameNonExistentError");
  if (NULL == module_state->NodeNameNonExistentError) {
    Py_DECREF(module_state->UnknownROSArgsError);
    Py_DECREF(module_state->RCLInvalidROSArgsError);
    Py_DECREF(module_state->RCLError);
    Py_DECREF(exceptions_module);
    return -1;
  }

  module_state->UnsupportedEventTypeError =
    PyObject_GetAttrString(exceptions_module, "UnsupportedEventTypeError");
  if (NULL == module_state->UnsupportedEventTypeError) {
    Py_DECREF(module_state->NodeNameNonExistentError);
    Py_DECREF(module_state->UnknownROSArgsError);
    Py_DECREF(module_state->RCLInvalidROSArgsError);
    Py_DECREF(module_state->RCLError);
    Py_DECREF(exceptions_module);
    return -1;
  }

  Py_DECREF(exceptions_module);

  // 0 is success
  return 0;
}

static int rclpy_traverse(PyObject * m, visitproc visit, void * arg)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(m);
  if (!module_state) {
    return -1;
  }
  Py_VISIT(module_state->NodeNameNonExistentError);
  Py_VISIT(module_state->UnknownROSArgsError);
  Py_VISIT(module_state->UnsupportedEventTypeError);
  Py_VISIT(module_state->RCLInvalidROSArgsError);
  Py_VISIT(module_state->RCLError);
  return 0;
}

static int rclpy_clear(PyObject * m)
{
  rclpy_module_state_t * module_state = (rclpy_module_state_t *)PyModule_GetState(m);
  if (!module_state) {
    return -1;
  }
  Py_CLEAR(module_state->NodeNameNonExistentError);
  Py_CLEAR(module_state->UnknownROSArgsError);
  Py_CLEAR(module_state->UnsupportedEventTypeError);
  Py_CLEAR(module_state->RCLInvalidROSArgsError);
  Py_CLEAR(module_state->RCLError);
  return 0;
}

static PyModuleDef_Slot rclpy_slots[] = {
  {Py_mod_exec, rclpy_exec},
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
