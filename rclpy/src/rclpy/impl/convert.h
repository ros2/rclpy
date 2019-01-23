#ifndef RCLPY_IMPL_CONVERT_H_
#define RCLPY_IMPL_CONVERT_H_

#include <Python.h>

#include <rmw/types.h>

/// Convert a C rmw_qos_profile_t into a Python QoSProfile object
/**
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
  PyObject_SetAttrString(pyqos_profile, "avoid_ros_namespace_conventions",
    PyBool_FromLong(qos_profile->avoid_ros_namespace_conventions));

  assert(pyqos_profile != NULL);
  return pyqos_profile;
}

static void * get_capsule_pointer(PyObject * pymetaclass, const char * attr)
{
  PyObject * pyattr = PyObject_GetAttrString(pymetaclass, attr);
  if (!pyattr) {
    return NULL;
  }
  void * ptr = PyCapsule_GetPointer(pyattr, NULL);
  Py_DECREF(pyattr);
  return ptr;
}
#endif  // RCLPY_IMPL_CONVERT_H_
