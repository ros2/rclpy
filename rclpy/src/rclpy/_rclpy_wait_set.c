// Copyright 2017 Open Source Robotics Foundation, Inc.
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
#include <structmember.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include "src/rclpy/sigint_gc.h"


/// Storage for custom python type _rclpy_wait_set.WaitSet
typedef struct
{
  PyObject_HEAD
  rcl_wait_set_t wait_set;
  /// Lists of pycapsules in the wait set
  PyObject * pysubs;
  PyObject * pytmrs;
  PyObject * pygcs;
  PyObject * pyclis;
  PyObject * pysrvs;
  /// List of pycapsule that are ready
  PyObject * pyready;
} rclpy_wait_set_t;

/// Destructor
static void
rclpy_wait_set_dealloc(rclpy_wait_set_t * self)
{
  rcl_ret_t ret = rcl_wait_set_fini(&(self->wait_set));
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to fini wait set: %s", rcl_get_error_string_safe());
    rcl_reset_error();
  }

  // X because it could be NULL if PyList_New failed during rclpy_wait_set_new
  Py_XDECREF(self->pysubs);
  Py_XDECREF(self->pytmrs);
  Py_XDECREF(self->pygcs);
  Py_XDECREF(self->pyclis);
  Py_XDECREF(self->pysrvs);
  Py_XDECREF(self->pyready);

  Py_TYPE(self)->tp_free((PyObject *)self);
}

/// Constructor
static PyObject *
rclpy_wait_set_new(PyTypeObject * type, PyObject * Py_UNUSED(args), PyObject * Py_UNUSED(kwds))
{
  rclpy_wait_set_t * self;

  self = (rclpy_wait_set_t *)type->tp_alloc(type, 0);
  if (self) {
    self->wait_set = rcl_get_zero_initialized_wait_set();
  }

  return (PyObject *)self;
}

/// Initializer
static int
rclpy_wait_set_init(
  rclpy_wait_set_t * self, PyObject * Py_UNUSED(args), PyObject * Py_UNUSED(kwds))
{
  // rclpy_wait_set_dealloc will take care of decref
#define MAKE_LIST_OR_BAIL(ELIST) \
  ELIST = PyList_New(0); \
  if (!(ELIST)) { \
    return -1; \
  }

  MAKE_LIST_OR_BAIL(self->pysubs);
  MAKE_LIST_OR_BAIL(self->pytmrs);
  MAKE_LIST_OR_BAIL(self->pygcs);
  MAKE_LIST_OR_BAIL(self->pyclis);
  MAKE_LIST_OR_BAIL(self->pysrvs);
  MAKE_LIST_OR_BAIL(self->pyready);
#undef MAKE_LIST_OR_BAIL

  rcl_ret_t ret = rcl_wait_set_init(&(self->wait_set), 0, 0, 0, 0, 0, rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to initialize wait set: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return -1;
  }

  return 0;
}

/// Return a pycapsule handle from an rclpy type
/*
 * Raises any exception raised by an object with a custom __getattr__
 *
 * \param[in] pyentity
 * \returns NULL on error else something that is probably a pycapsule
 */
static inline PyObject *
_rclpy_to_pycapsule(PyObject * pyentity)
{
  if (PyObject_HasAttrString(pyentity, "subscription_handle")) {
    pyentity = PyObject_GetAttrString(pyentity, "subscription_handle");
  } else if (PyObject_HasAttrString(pyentity, "guard_handle")) {
    pyentity = PyObject_GetAttrString(pyentity, "guard_handle");
  } else if (PyObject_HasAttrString(pyentity, "timer_handle")) {
    pyentity = PyObject_GetAttrString(pyentity, "timer_handle");
  } else if (PyObject_HasAttrString(pyentity, "client_handle")) {
    pyentity = PyObject_GetAttrString(pyentity, "client_handle");
  } else if (PyObject_HasAttrString(pyentity, "service_handle")) {
    pyentity = PyObject_GetAttrString(pyentity, "service_handle");
  }
  return pyentity;
}


/// Add entities of a known type to the correct list
/*
 * Raises ValueError if capsule is invalid
 * Handles adding entities of a given type to the wait set
 *
 * \param[in] pylist a list that the entities should be added to
 * \param[in] pyentities an iterable of (sub, pub, client, serv, guard)
 * \param[in] handle_attr an attribute of an entity where the handle is stored
 * \param[in] handle_type a pycapsule name this entity uses
 * \return RCL_RET_OK if everything succeded
 */
static inline rcl_ret_t
_rclpy_add_entity(
  PyObject * pylist, PyObject * pyentities, const char * handle_attr,
  const char * handle_type)
{
  // It's possible for arbitrary python code to be invoked
  Py_INCREF(pylist);
  Py_INCREF(pyentities);

  PyObject * pyiter = PyObject_GetIter(pyentities);
  if (!pyiter) {
    // exception set
    Py_DECREF(pylist);
    Py_DECREF(pyentities);
    return RCL_RET_ERROR;
  }

  PyObject * pyentity;
  while ((pyentity = PyIter_Next(pyiter))) {
    // Accept an instance of an rclpy type for convenience
    if (PyObject_HasAttrString(pyentity, handle_attr)) {
      pyentity = PyObject_GetAttrString(pyentity, handle_attr);
    }

    // No chance of arbitrary python code below, so decref early
    Py_DECREF(pyentity);

    if (!pyentity) {
      // Exception set
      break;
    }

    if (PyCapsule_IsValid(pyentity, handle_type)) {
      if (-1 == PyList_Append(pylist, pyentity)) {
        break;
      }
    } else {
      const char * entity_type = PyCapsule_GetName(pyentity);
      if (entity_type) {
        PyErr_Format(PyExc_ValueError, "Unknown capsule type: '%s'", entity_type);
      }  // else PyCapsule_GetName raised
      break;
    }
  }

  Py_DECREF(pyiter);
  Py_DECREF(pyentities);
  Py_DECREF(pylist);

  if (PyErr_Occurred()) {
    // Return down here after references have been cleaned dup
    return RCL_RET_ERROR;
  }
  return RCL_RET_OK;
}

/// Add subscriptions to be waited on to the appropriate set
/*
 * Raises ValueError if a capsule is invalid
 * Raises TypeError if the argument is not iterable
 *
 * \param[in] self instance of rclpy_wait_set_t
 * \param[in] pyentities Iterable of subscriptions
 */
static PyObject *
rclpy_wait_set_add_subscriptions(rclpy_wait_set_t * self, PyObject * args)
{
  PyObject * pyentities;
  if (!PyArg_ParseTuple(args, "O", &pyentities)) {
    return NULL;
  }

  if (RCL_RET_OK != _rclpy_add_entity(self->pysubs, pyentities, "subscription_handle",
    "rcl_subscription_t"))
  {
    // Exception set
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Add guard_conditions to be waited on to the appropriate set
/*
 * Raises ValueError if a capsule is invalid
 * Raises TypeError if the argument is not iterable
 *
 * \param[in] self instance of rclpy_wait_set_t
 * \param[in] pyentities Iterable of guard_conditions
 */
static PyObject *
rclpy_wait_set_add_guard_conditions(rclpy_wait_set_t * self, PyObject * args)
{
  PyObject * pyentities;
  if (!PyArg_ParseTuple(args, "O", &pyentities)) {
    return NULL;
  }

  if (RCL_RET_OK != _rclpy_add_entity(self->pygcs, pyentities, "guard_handle",
    "rcl_guard_condition_t"))
  {
    // Exception set
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Add timers to be waited on to the appropriate set
/*
 * Raises ValueError if a capsule is invalid
 * Raises TypeError if the argument is not iterable
 *
 * \param[in] self instance of rclpy_wait_set_t
 * \param[in] pyentities Iterable of timers
 */
static PyObject *
rclpy_wait_set_add_timers(rclpy_wait_set_t * self, PyObject * args)
{
  PyObject * pyentities;
  if (!PyArg_ParseTuple(args, "O", &pyentities)) {
    return NULL;
  }

  if (RCL_RET_OK != _rclpy_add_entity(self->pytmrs, pyentities, "timer_handle",
    "rcl_timer_t"))
  {
    // Exception set
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Add clients to be waited on to the appropriate set
/*
 * Raises ValueError if a capsule is invalid
 * Raises TypeError if the argument is not iterable
 *
 * \param[in] self instance of rclpy_wait_set_t
 * \param[in] pyentities Iterable of clients
 */
static PyObject *
rclpy_wait_set_add_clients(rclpy_wait_set_t * self, PyObject * args)
{
  PyObject * pyentities;
  if (!PyArg_ParseTuple(args, "O", &pyentities)) {
    return NULL;
  }

  if (RCL_RET_OK != _rclpy_add_entity(self->pyclis, pyentities, "client_handle",
    "rcl_client_t"))
  {
    // Exception set
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Add services to be waited on to the appropriate set
/*
 * Raises ValueError if a capsule is invalid
 * Raises TypeError if the argument is not iterable
 *
 * \param[in] self instance of rclpy_wait_set_t
 * \param[in] pyentities Iterable of services
 */
static PyObject *
rclpy_wait_set_add_services(rclpy_wait_set_t * self, PyObject * args)
{
  PyObject * pyentities;
  if (!PyArg_ParseTuple(args, "O", &pyentities)) {
    return NULL;
  }

  if (RCL_RET_OK != _rclpy_add_entity(self->pysrvs, pyentities, "service_handle",
    "rcl_service_t"))
  {
    // Exception set
    return NULL;
  }
  Py_RETURN_NONE;
}

/// Build a wait set
/*
 * Raises RuntimeError if an RCL error occurs
 *
 * This method adds all of the PyCapsule to the rcl_wait_set_t instance, resizing it in the process
 *
 * \return number of entities in the wait set
*/
static inline rcl_ret_t
_rclpy_build_wait_set(rclpy_wait_set_t * self)
{
  rcl_ret_t ret;
  ret = rcl_wait_set_fini(&(self->wait_set));
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to finalize wait set: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return ret;
  }

  Py_ssize_t num_subs = PyObject_Length(self->pysubs);
  if (-1 == num_subs) {
    return RCL_RET_ERROR;
  }
  Py_ssize_t num_gcs = PyObject_Length(self->pygcs);
  if (-1 == num_gcs) {
    return RCL_RET_ERROR;
  }
  Py_ssize_t num_tmrs = PyObject_Length(self->pytmrs);
  if (-1 == num_tmrs) {
    return RCL_RET_ERROR;
  }
  Py_ssize_t num_clis = PyObject_Length(self->pyclis);
  if (-1 == num_clis) {
    return RCL_RET_ERROR;
  }
  Py_ssize_t num_srvs = PyObject_Length(self->pysrvs);
  if (-1 == num_srvs) {
    return RCL_RET_ERROR;
  }

  ret = rcl_wait_set_init(&(self->wait_set), num_subs, num_gcs, num_tmrs, num_clis, num_srvs,
      rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to initialize wait set: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return ret;
  }

#define RCLPY_ADD_ENTITY(ETYPE, WSFUNC, ELIST) \
  do { \
    PyObject * pyiter = PyObject_GetIter(ELIST); \
    if (!pyiter) { \
      return RCL_RET_ERROR; \
    } \
    PyObject * pyentity; \
    while ((pyentity = PyIter_Next(pyiter))) { \
      ETYPE * entity = (ETYPE *)PyCapsule_GetPointer(pyentity, #ETYPE); \
      if (!entity) { \
        Py_DECREF(pyentity); \
        Py_DECREF(pyiter); \
        return RCL_RET_ERROR; \
      } \
      rcl_ret_t ret = WSFUNC(&(self->wait_set), entity); \
      if (ret != RCL_RET_OK) { \
        PyErr_Format(PyExc_RuntimeError, \
          "Failed to add entity '" #ETYPE "' to wait set: %s", rcl_get_error_string_safe()); \
        rcl_reset_error(); \
        Py_DECREF(pyentity); \
        Py_DECREF(pyiter); \
        return ret; \
      } \
      Py_DECREF(pyentity); \
    } \
    Py_DECREF(pyiter); \
  } while (false)

  RCLPY_ADD_ENTITY(rcl_subscription_t, rcl_wait_set_add_subscription, self->pysubs);
  RCLPY_ADD_ENTITY(rcl_guard_condition_t, rcl_wait_set_add_guard_condition, self->pygcs);
  RCLPY_ADD_ENTITY(rcl_timer_t, rcl_wait_set_add_timer, self->pytmrs);
  RCLPY_ADD_ENTITY(rcl_client_t, rcl_wait_set_add_client, self->pyclis);
  RCLPY_ADD_ENTITY(rcl_service_t, rcl_wait_set_add_service, self->pysrvs);
#undef RCLPY_ADD_ENTITY

  return RCL_RET_OK;
}

/// Fill pyready with entities that are ready
/*
 * \param[in] self an instance of _rclpy_wait_set.WaitSet
 * \param[in] pyset the set of entities to check
 * \param[in] type the name of the PyCapsule
 */
static inline rcl_ret_t
_rclpy_build_ready_entities(rclpy_wait_set_t * self)
{
  if (-1 == PySequence_DelSlice(self->pyready, 0, PySequence_Length(self->pyready))) {
    return RCL_RET_ERROR;
  }

#define GET_READY_ENTITIES(ETYPE, ELIST) \
  do { \
    PyObject * pyiter = PyObject_GetIter((ELIST)); \
    if (!pyiter) { \
      return RCL_RET_ERROR; \
    } \
    PyObject * pyentity; \
    while ((pyentity = PyIter_Next(pyiter))) { \
      rcl_ ## ETYPE ## _t * entity = \
        (rcl_ ## ETYPE ## _t *)PyCapsule_GetPointer(pyentity, "rcl_" #ETYPE "_t"); \
      if (!entity) { \
        Py_DECREF(pyentity); \
        Py_DECREF(pyiter); \
        return RCL_RET_ERROR; \
      } \
      size_t idx; \
      size_t idx_max; \
      idx_max = self->wait_set.size_of_ ## ETYPE ## s; \
      const rcl_ ## ETYPE ## _t ** struct_ptr = self->wait_set.ETYPE ## s; \
      for (idx = 0; idx < idx_max; ++idx) { \
        if (struct_ptr[idx] == entity) { \
          if (-1 == PyList_Append(self->pyready, pyentity)) { \
            Py_DECREF(pyentity); \
            Py_DECREF(pyiter); \
            return RCL_RET_ERROR; \
          } \
        } \
      } \
      Py_DECREF(pyentity); \
    } \
    Py_DECREF(pyiter); \
  } while (false)

  GET_READY_ENTITIES(subscription, self->pysubs);
  GET_READY_ENTITIES(guard_condition, self->pygcs);
  GET_READY_ENTITIES(timer, self->pytmrs);
  GET_READY_ENTITIES(client, self->pyclis);
  GET_READY_ENTITIES(service, self->pysrvs);
#undef GET_READY_ENTITIES
  return RCL_RET_OK;
}

/// Wait until timeout is reached or an event happened
/**
 * Raises RuntimeError if there was an error while waiting
 *
 * This function will wait for an event to happen or for the timeout to expire.
 * A negative timeout means wait forever, a timeout of 0 means no wait
 * \param[in] timeout optional time to wait before waking up (in nanoseconds)
 * \return NULL
 */
static PyObject *
rclpy_wait_set_wait(rclpy_wait_set_t * self, PyObject * args)
{
  PY_LONG_LONG timeout = -1;

  if (!PyArg_ParseTuple(args, "|K", &timeout)) {
    return NULL;
  }

  if (RCL_RET_OK != _rclpy_build_wait_set(self)) {
    // exception set
    return NULL;
  }

#ifdef _WIN32
  _crt_signal_t
#else
  sig_t
#endif  // _WIN32
  previous_handler = signal(SIGINT, rclpy_catch_function);
  rcl_ret_t ret;

  // Reference could be invalidated by arbitrary python code running while GIL is released
  Py_INCREF(self);
  Py_BEGIN_ALLOW_THREADS;
  ret = rcl_wait(&(self->wait_set), timeout);
  Py_END_ALLOW_THREADS;
  Py_DECREF(self);

  signal(SIGINT, previous_handler);
  if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
    PyErr_Format(PyExc_RuntimeError,
      "Failed to wait on wait set: %s", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  if (RCL_RET_OK != _rclpy_build_ready_entities(self)) {
    // exception set
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Remove all stored entities from the wait set
/**
 * Raises RuntimeError if there was an error while clearing
 *
 * \return None
 */
static PyObject *
rclpy_wait_set_clear(rclpy_wait_set_t * self)
{
  if (-1 == PySequence_DelSlice(self->pysubs, 0, PySequence_Length(self->pysubs)) ||
    -1 == PySequence_DelSlice(self->pytmrs, 0, PySequence_Length(self->pytmrs)) ||
    -1 == PySequence_DelSlice(self->pygcs, 0, PySequence_Length(self->pygcs)) ||
    -1 == PySequence_DelSlice(self->pyclis, 0, PySequence_Length(self->pyclis)) ||
    -1 == PySequence_DelSlice(self->pysrvs, 0, PySequence_Length(self->pysrvs)))
  {
    return NULL;
  }

  Py_RETURN_NONE;
}

/// Return True if an entity is ready
/**
 * Raises RuntimeError if there was an error while clearing
 * Raises ValueError if the capsule is invalid
 *
 * \return True if the entity is ready
 */
static PyObject *
rclpy_wait_set_is_ready(rclpy_wait_set_t * self, PyObject * args)
{
  PyObject * pyentity;
  if (!PyArg_ParseTuple(args, "O", &pyentity)) {
    return NULL;
  }

  pyentity = _rclpy_to_pycapsule(pyentity);

  int contains = PySequence_Contains(self->pyready, pyentity);
  if (-1 == contains) {
    // Exception set
    return NULL;
  }
  if (contains) {
    Py_RETURN_TRUE;
  }
  Py_RETURN_FALSE;
}

static PyMemberDef rclpy_wait_set_members[] = {
  {"_pysubs", T_OBJECT_EX, offsetof(rclpy_wait_set_t, pysubs), 0,
    "subscription capsules"},
  {"_pygcs", T_OBJECT_EX, offsetof(rclpy_wait_set_t, pygcs), 0,
    "guard_condition capsules"},
  {"_pytmrs", T_OBJECT_EX, offsetof(rclpy_wait_set_t, pytmrs), 0,
    "timer capsules"},
  {"_pyclis", T_OBJECT_EX, offsetof(rclpy_wait_set_t, pyclis), 0,
    "client capsules"},
  {"_pysrvs", T_OBJECT_EX, offsetof(rclpy_wait_set_t, pysrvs), 0,
    "service capsules"},
  {"_pyready", T_OBJECT_EX, offsetof(rclpy_wait_set_t, pyready), 0,
    "ready capsules"},
  {NULL}  /* Sentinel */
};

/// Define methods of _rclpy_wait_set.WaitSet
static PyMethodDef rclpy_wait_set_methods[] = {
  {"add_subscriptions", (PyCFunction)rclpy_wait_set_add_subscriptions, METH_VARARGS,
    "Add a bunch of subscription instances or handles to the wait_set."},
  {"add_guard_conditions", (PyCFunction)rclpy_wait_set_add_guard_conditions, METH_VARARGS,
    "Add a bunch of guard_condition instances or handles to the wait_set."},
  {"add_timers", (PyCFunction)rclpy_wait_set_add_timers, METH_VARARGS,
    "Add a bunch of timer instances or handles to the wait_set."},
  {"add_clients", (PyCFunction)rclpy_wait_set_add_clients, METH_VARARGS,
    "Add a bunch of client instances or handles to the wait_set."},
  {"add_services", (PyCFunction)rclpy_wait_set_add_services, METH_VARARGS,
    "Add a bunch of service instances or handles to the wait_set."},
  {"wait", (PyCFunction)rclpy_wait_set_wait, METH_VARARGS,
    "Wait until timeout is reached or an event happened."},
  {"clear", (PyCFunction)rclpy_wait_set_clear, METH_NOARGS,
    "Remove all entities from the wait set."},
  {"is_ready", (PyCFunction)rclpy_wait_set_is_ready, METH_VARARGS,
    "Return True if an entity is ready."},
  {NULL}  /* Sentinel */
};


// Partially initializing is recommended in the python docs
// I don't see a way to complete the initializer without duplicating #ifdef in cpython header
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
/// Python type _rclpy_wait_set.WaitSet
static PyTypeObject rclpy_wait_set_type_t = {
  PyVarObject_HEAD_INIT(NULL, 0)
  "_rclpy_wait_set.WaitSet",    /* tp_name */
  sizeof(rclpy_wait_set_t),     /* tp_basicsize */
  0,                            /* tp_itemsize */
  (destructor)rclpy_wait_set_dealloc,  /* tp_dealloc */
  0,                            /* tp_print */
  0,                            /* tp_getattr */
  0,                            /* tp_setattr */
  0,                            /* tp_reserved */
  0,                            /* tp_repr */
  0,                            /* tp_as_number */
  0,                            /* tp_as_sequence */
  0,                            /* tp_as_mapping */
  0,                            /* tp_hash  */
  0,                            /* tp_call */
  0,                            /* tp_str */
  0,                            /* tp_getattro */
  0,                            /* tp_setattro */
  0,                            /* tp_as_buffer */
  Py_TPFLAGS_DEFAULT,           /* tp_flags */
  "Interface to a wait set.",   /* tp_doc */
  0,                            /* tp_traverse */
  0,                            /* tp_clear */
  0,                            /* tp_richcompare */
  0,                            /* tp_weaklistoffset */
  0,                            /* tp_iter */
  0,                            /* tp_iternext */
  rclpy_wait_set_methods,       /* tp_methods */
  rclpy_wait_set_members,       /* tp_members */
  0,                            /* tp_getset */
  0,                            /* tp_base */
  0,                            /* tp_dict */
  0,                            /* tp_descr_get */
  0,                            /* tp_descr_set */
  0,                            /* tp_dictoffset */
  (initproc)rclpy_wait_set_init,  /* tp_init */
  0,                            /* tp_alloc */
  rclpy_wait_set_new,           /* tp_new */
};
#pragma GCC diagnostic pop

static PyModuleDef wait_set_module = {
  PyModuleDef_HEAD_INIT,
  "_rclpy_wait_set",
  "Extention module for a wait set class.",
  -1,
  NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC
PyInit__rclpy_wait_set(void)
{
  PyObject * m;

  rclpy_wait_set_type_t.tp_new = PyType_GenericNew;
  if (PyType_Ready(&rclpy_wait_set_type_t) < 0) {
    return NULL;
  }

  m = PyModule_Create(&wait_set_module);
  if (m == NULL) {
    return NULL;
  }

  Py_INCREF(&rclpy_wait_set_type_t);
  PyModule_AddObject(m, "WaitSet", (PyObject *)&rclpy_wait_set_type_t);
  return m;
}
