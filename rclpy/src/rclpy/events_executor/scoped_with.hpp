// Copyright 2024-2025 Brad Martin
// Copyright 2024 Merlin Labs, Inc.
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

#ifndef RCLPY__EVENTS_EXECUTOR__SCOPED_WITH_HPP_
#define RCLPY__EVENTS_EXECUTOR__SCOPED_WITH_HPP_

#include <pybind11/pybind11.h>
#include <Python.h>

#include <utility>

namespace rclpy
{
namespace events_executor
{

inline bool interpreter_is_finalizing()
{
#if PY_VERSION_HEX >= 0x030D0000
  return Py_IsFinalizing() != 0;
#else
  return _Py_IsFinalizing() != 0;   // private but present on 3.7–3.12
#endif
}

/// Enters a python context manager for the scope of this object instance.
class ScopedWith
{
public:
  explicit ScopedWith(pybind11::handle object)
  : object_(pybind11::cast<pybind11::object>(object))
  {
    object_.attr("__enter__")();
  }

  ~ScopedWith() noexcept
  {
    // Move object_ out so the member destructor (~py::object) is a
    // no-op after this body completes.
    pybind11::object object = std::move(object_);
    if (!object) {
      return;
    }

    if (interpreter_is_finalizing()) {
      // During interpreter shutdown, reacquiring thread state from
      // arbitrary threads is forbidden.  Leak the handle rather
      // than crash.  __exit__ is skipped — acceptable because the
      // process is terminating.
      object.release();
      return;
    }

    // Acquire the GIL.  owned_object is declared *after* gil so that
    // it is destroyed (Py_DECREF) *before* the GIL is released — C++
    // destroys locals in reverse declaration order.
    pybind11::gil_scoped_acquire gil;
    pybind11::object owned_object = std::move(object);
    try {
      owned_object.attr("__exit__")(
        pybind11::none(), pybind11::none(), pybind11::none());
    } catch (pybind11::error_already_set & e) {
      e.discard_as_unraisable("ScopedWith::~ScopedWith");
    }
  }

private:
  pybind11::object object_;
};

}  // namespace events_executor
}  // namespace rclpy

#endif  // RCLPY__EVENTS_EXECUTOR__SCOPED_WITH_HPP_
