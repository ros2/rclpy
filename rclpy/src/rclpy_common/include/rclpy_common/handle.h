// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLPY_COMMON__HANDLE_H_
#define RCLPY_COMMON__HANDLE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <Python.h>
#include <stddef.h>

#include "rcutils/types/rcutils_ret.h"

/// Wrapper that manages the lifetime of another object,
/// allowing to establish dependencies between them.
typedef struct rclpy_handle_t rclpy_handle_t;

/// Signature of rclpy_handle_t destructors.
typedef void (* rclpy_handle_destructor_t)(void *);

/// Create a PyCapsule wrapping a rclpy_handle_t object.
/**
 * \sa _rclpy_create_handle
 * \param name Name of the PyCapsule.
 */
PyObject *
rclpy_create_handle_capsule(void * ptr, const char * name, rclpy_handle_destructor_t destructor);

/// Returns the object managed by the rclpy_handle_t wrapped in a PyCapsule.
void *
rclpy_handle_get_pointer_from_capsule(PyObject * capsule, const char * name);

/// Creates a rclpy_handle_t object.
/**
 * \param ptr Opaque pointer to the object being wrapped.
 * \param destructor Function that will be called when the handle is destructed.
 */
rclpy_handle_t *
_rclpy_create_handle(void * ptr, rclpy_handle_destructor_t destructor);

/// Returns the object managed by the rclpy_handle_t.
void *
_rclpy_handle_get_pointer(rclpy_handle_t * handle);

/// Adds a dependency to a handle.
/**
 * \param dependency Handle object which its reference count will be incremented.
 * \param dependent Handle object that keeps a reference to the `dependency`.
 */
rcutils_ret_t
_rclpy_handle_add_dependency(rclpy_handle_t * dependent, rclpy_handle_t * dependency);

/// Decrements the reference count of a handle.
/**
 * The reference count of `handle` is decremented.
 * If it reaches zero:
 * - `rclpy_handle_dec_ref` is called on `handle` dependencies.
 * - `handle` is deallocated.
 *
 * \param handle Object which reference count will be decremented.
 */
void
_rclpy_handle_dec_ref(rclpy_handle_t * handle);

#ifdef __cplusplus
}
#endif

#endif  // RCLPY_COMMON__HANDLE_H_
