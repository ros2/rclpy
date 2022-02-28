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

#include <pybind11/pybind11.h>

#include <rcl/allocator.h>
#include <rcl/error_handling.h>
#include <rcl/logging.h>
#include <rcl/types.h>
#include <rcutils/logging.h>
#include <rcutils/time.h>

#include <mutex>
#include <stdexcept>

#include "exceptions.hpp"
#include "logging.hpp"

namespace rclpy
{
LoggingGuard::LoggingGuard()
: guard_(logging_mutex_)
{
}

// Initialize logging mutex
std::recursive_mutex LoggingGuard::logging_mutex_;

static
void
rclpy_thread_safe_logging_output_handler(
  const rcutils_log_location_t * location,
  int severity,
  const char * name,
  rcutils_time_point_value_t timestamp,
  const char * format,
  va_list * args)
{
  try {
    rclpy::LoggingGuard scoped_logging_guard;
    rcl_logging_multiple_output_handler(location, severity, name, timestamp, format, args);
  } catch (const std::exception & ex) {
    RCUTILS_SAFE_FWRITE_TO_STDERR("rclpy failed to get the global logging mutex: ");
    RCUTILS_SAFE_FWRITE_TO_STDERR(ex.what());
    RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
  } catch (...) {
    RCUTILS_SAFE_FWRITE_TO_STDERR("rclpy failed to get the global logging mutex\n");
  }
}

void
logging_configure(Context & context)
{
  rcl_allocator_t allocator = rcl_get_default_allocator();

  rclpy::LoggingGuard scoped_logging_guard;
  rcl_ret_t ret = rcl_logging_configure_with_output_handler(
    &context.rcl_ptr()->global_arguments,
    &allocator,
    rclpy_thread_safe_logging_output_handler);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to initialize logging");
  }
}

void
logging_fini(void)
{
  rclpy::LoggingGuard scoped_logging_guard;
  rcl_ret_t ret = rcl_logging_fini();
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to fini logging");
  }
}
}  // namespace rclpy
