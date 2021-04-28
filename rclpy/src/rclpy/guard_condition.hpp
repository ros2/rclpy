// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLPY__GUARD_CONDITION_HPP_
#define RCLPY__GUARD_CONDITION_HPP_

#include <pybind11/pybind11.h>

#include <rcl/error_handling.h>
#include <rcl/guard_condition.h>

#include <memory>

#include "context.hpp"
#include "destroyable.hpp"
#include "rclpy_common/exceptions.hpp"
#include "rclpy_common/handle.h"
#include "utils.hpp"

namespace py = pybind11;

namespace rclpy
{
/// Create a general purpose guard condition
class GuardCondition : public Destroyable, public std::enable_shared_from_this<GuardCondition>
{
public:
  /**
   * Raises RuntimeError if initializing the guard condition fails
   */
  explicit GuardCondition(Context & context);

  /// Trigger a general purpose guard condition
  /**
   * Raises ValueError if pygc is not a guard condition capsule
   * Raises RCLError if the guard condition could not be triggered
   */
  void
  trigger_guard_condition();

  /// Get rcl_guard_condition_t pointer
  rcl_guard_condition_t * rcl_ptr() const
  {
    return rcl_guard_condition_.get();
  }

  // TODO(ahcorde): Remove the pycapsule method when #728 is in
  /// Return a pycapsule object to be able to handle the signal in C.
  py::capsule
  pycapsule() const
  {
    PyObject * capsule = rclpy_create_handle_capsule(
      rcl_guard_condition_.get(), "rcl_guard_condition_t", _rclpy_destroy_guard_condition);
    if (!capsule) {
      throw py::error_already_set();
    }
    return py::reinterpret_steal<py::capsule>(capsule);
  }

  /// Force an early destruction of this object
  void destroy() override;

private:
  Context context_;
  std::shared_ptr<rcl_guard_condition_t> rcl_guard_condition_;

  /// Handle destructor for guard condition
  static void
  _rclpy_destroy_guard_condition(void * p)
  {
    (void)p;
    // Empty destructor, the class should take care of the lifecycle.
  }
};

/// Define a pybind11 wrapper for an rclpy::Service
void define_guard_condition(py::object module);
}  // namespace rclpy

#endif  // RCLPY__GUARD_CONDITION_HPP_
