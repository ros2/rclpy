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

#include <rcl/context.h>
#include <rcl/error_handling.h>
#include <rcl/init_options.h>

#include <memory>

#include "destroyable.hpp"
#include "exceptions.hpp"

namespace py = pybind11;

namespace rclpy
{
struct InitOptions
{
  explicit InitOptions(rcl_allocator_t allocator)
  {
    rcl_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = rcl_init_options_init(&rcl_options, allocator);
    if (RCL_RET_OK != ret) {
      throw RCLError("Failed to initialize init options");
    }
  }

  ~InitOptions()
  {
    rcl_ret_t ret = rcl_init_options_fini(&rcl_options);
    if (RCL_RET_OK != ret) {
      int stack_level = 1;
      PyErr_WarnFormat(
        PyExc_RuntimeWarning, stack_level,
        "[rclpy| %s : %s ]: failed to fini rcl_init_options_t in destructor: %s",
        RCUTILS_STRINGIFY(__FILE__), RCUTILS_STRINGIFY(__LINE__), rcl_get_error_string().str);
      rcl_reset_error();
    }
  }

  rcl_init_options_t rcl_options;
};

void shutdown_contexts();

class Context : public Destroyable, public std::enable_shared_from_this<Context>
{
public:
  /// Create a Context instance.
  /**
   * Raises MemoryError if allocating memory fails.
   * Raises RuntimeError if creating the context fails.
   *
   * \param[in] pyargs List of command line arguments
   * \param[in] domain_id domain id to be set in this context
   */
  Context(py::list pyargs, size_t domain_id);

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
