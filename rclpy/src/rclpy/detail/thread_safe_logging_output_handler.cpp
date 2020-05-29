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

#include "./logging_mutex.hpp"
#include "./thread_safe_logging_output_handler.h"

#include "rcl/logging.h"

extern "C"
{

void
rclpy_detail_thread_safe_logging_output_handler(
  const rcutils_log_location_t * location,
  int severity,
  const char * name,
  rcutils_time_point_value_t timestamp,
  const char * format,
  va_list * args)
{
  try {
    std::shared_ptr<std::recursive_mutex> mutex = rclpy::detail::get_global_logging_mutex();
    std::lock_guard<std::recursive_mutex> guard(*mutex);
    rcl_logging_multiple_output_handler(location, severity, name, timestamp, format, args);
  } catch (std::exception & ex) {
    RCUTILS_SAFE_FWRITE_TO_STDERR("rclpy failed to get the global logging mutex: ");
    RCUTILS_SAFE_FWRITE_TO_STDERR(ex.what());
    RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
  } catch (...) {
    RCUTILS_SAFE_FWRITE_TO_STDERR("rclpy failed to get the global logging mutex\n");
  }
}

}  // extern "C"
