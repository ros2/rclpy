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

#ifndef RCLPY__LOGGING_HPP_
#define RCLPY__LOGGING_HPP_

#include <pybind11/pybind11.h>

#include <mutex>

#include "context.hpp"

namespace py = pybind11;

namespace rclpy
{
/// Acquire the global logging mutex when constructed, and release it when destructed
class LoggingGuard
{
public:
  LoggingGuard();

private:
  static std::recursive_mutex logging_mutex_;
  std::lock_guard<std::recursive_mutex> guard_;
};

/// Initialize rcl logging
/**
 * Raises RuntimeError if rcl logging could not be initialized
 * \param[in] _context A context instance to use to retrieve global CLI arguments.
 */
void
logging_configure(Context & _context);

/// Finalize rcl logging
void
logging_fini(void);
}  // namespace rclpy

#endif  // RCLPY__LOGGING_HPP_
