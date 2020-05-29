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

#ifndef RCLPY__DETAIL__THREAD_SAFE_LOGGING_OUTPUT_HANDLER_H_
#define RCLPY__DETAIL__THREAD_SAFE_LOGGING_OUTPUT_HANDLER_H_

#include "rcutils/logging.h"
#include "rcutils/visibility_control_macros.h"

#ifdef __cplusplus
extern "C"
{
#endif

/// A thread-safe logging output handler.
RCUTILS_LOCAL
void
rclpy_detail_thread_safe_logging_output_handler(
  const rcutils_log_location_t * location,
  int severity,
  const char * name,
  rcutils_time_point_value_t timestamp,
  const char * format,
  va_list * args);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // RCLPY__DETAIL__THREAD_SAFE_LOGGING_OUTPUT_HANDLER_H_