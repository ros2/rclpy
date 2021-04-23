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

#ifndef RCLPY__CONTEXT_HPP_
#define RCLPY__CONTEXT_HPP_

#include <pybind11/pybind11.h>

#include <functional>
#include <memory>

#include "destroyable.hpp"

namespace py = pybind11;

namespace rclpy
{
class Context : public Destroyable, public std::enable_shared_from_this<Context>
{
public:
  /// Create a Context instance.
  /**
   * Raises MemoryError if allocating memory fails.
   * Raises RuntimeError if creating the context fails.
   */
  Context();

  /// Retrieves domain id from init_options of context
  /**
   * \param[in] pycontext Capsule containing rcl_context_t
   * \return domain id
   */
  size_t
  get_domain_id();

  /// Status of the the client library
  /**
   * \return True if rcl is running properly, False otherwise
   */
  bool
  ok();

  void
  shutdown();

  /// Get rcl_context_t pointer
  rcl_context_t * rcl_ptr() const
  {
    return rcl_context_.get();
  }

  /// Force an early destruction of this object
  void destroy() override;

private:
  std::shared_ptr<rcl_context_t> rcl_context_;
};

/// Define a pybind11 wrapper for an rclpy::Service
void define_context(py::object module);
}  // namespace rclpy

#endif  // RCLPY__CONTEXT_HPP_
