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

#include <pybind11/pybind11.h>

namespace py = pybind11;

#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>
#include <rcutils/logging.h>

#include <rcl_logging_interface/rcl_logging_interface.h>

#include <stdexcept>
#include <string>

#include "logging_api.hpp"

/// Initialize the logging system.
/**
 * \return None
 */
void
rclpy_logging_initialize()
{
  rcutils_ret_t ret = rcutils_logging_initialize();
  if (ret != RCUTILS_RET_OK) {
    rcutils_reset_error();
    throw std::runtime_error("Failed to initialize logging system");
  }
}

/// Shutdown the logging system.
/**
 * \return None
 */
void
rclpy_logging_shutdown()
{
  // TODO(dhood): error checking
  rcutils_ret_t ret = rcutils_logging_shutdown();
  if (ret != RCUTILS_RET_OK) {
    rcutils_reset_error();
    throw std::runtime_error("Failed to shutdown logging system");
  }
}

/// Set the level of a logger.
/**
 *
 * \param[in] name Fully-qualified name of logger.
 * \param[in] level to set
 * \return None
 */
void
rclpy_logging_set_logger_level(const char * name, int level)
{
  rcutils_ret_t ret = rcutils_logging_set_logger_level(name, level);
  if (ret != RCUTILS_RET_OK) {
    rcutils_reset_error();
    throw std::runtime_error("Failed to set level for logger");
  }
}

/// Get the effective level of a logger.
/**
 * The "effective" logger level is determined as the logger level if it has been set explicitly,
 * otherwise it defers to the level of the logger's ancestors, and if all are unset
 * the default level is used.
 *
 * \param[in] name Fully-qualified name of logger.
 * \return The effective level
 */
int
rclpy_logging_get_logger_effective_level(const char * name)
{
  int logger_level = rcutils_logging_get_logger_effective_level(name);

  if (logger_level < 0) {
    rcutils_reset_error();
    throw std::runtime_error("Failed to get effective level for logger");
  }
  return logger_level;
}

/// Determine if the logger is enabled for a severity.
/**
 *
 * \param[in] name Fully-qualified name of logger.
 * \param[in] severity Logging severity to compare against.
 * \return True if the logger is enabled for the severity,
 * \return False otherwise.
 */
bool
rclpy_logging_logger_is_enabled_for(const char * name, int severity)
{
  return rcutils_logging_logger_is_enabled_for(name, severity);
}

/// Log a message through rcutils with the specified severity.
/**
 *
 * \param[in] severity Enum of type RCUTILS_LOG_SEVERITY.
 * \param[in] name Name of logger.
 * \param[in] message String to log.
 * \param[in] function_name String with the function name of the caller.
 * \param[in] file_name String with the file name of the caller.
 * \param[in] line_number Line number of the calling function.
 * \return None
 */
void
rclpy_logging_rcutils_log(
  int severity,
  const char * name,
  const char * message,
  const char * function_name,
  const char * file_name,
  uint64_t line_number)
{
  RCUTILS_LOGGING_AUTOINIT;
  rcutils_log_location_t logging_location = {function_name, file_name, line_number};
  rcutils_log(&logging_location, severity, name, "%s", message);
}

/// Get the log severity based on the log level string representation
/**
 * Raises RuntimeError if an invalid log level name is given.
 * Raises ValueError if log level is not a string.
 *
 * \param[in] log_level Name of the log level.
 * \return NULL on failure,
 * \return Log level associated with the string representation otherwise.
 */
int
rclpy_logging_severity_level_from_string(const char * log_level)
{
  int severity;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcutils_ret_t ret = rcutils_logging_severity_level_from_string(log_level, allocator, &severity);
  if (ret != RCUTILS_RET_OK) {
    rcutils_reset_error();
    throw std::runtime_error("Failed to get severity name");
  }
  return severity;
}

/// Get the current logging directory from rcutils.
/// \return Unicode UTF8 object containing the current logging directory.
std::string
rclpy_logging_get_logging_directory()
{
  char * log_dir = NULL;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_logging_ret_t ret = rcl_logging_get_logging_directory(allocator, &log_dir);
  if (RCL_LOGGING_RET_OK != ret) {
    rcutils_reset_error();
    throw std::runtime_error("Failed to get current logging directory");
  }
  std::string s_log_dir{log_dir};
  allocator.deallocate(log_dir, allocator.state);
  return s_log_dir;
}

namespace rclpy
{
void
define_logging_api(py::module m)
{
  py::enum_<RCUTILS_LOG_SEVERITY>(m, "RCUTILS_LOG_SEVERITY")
  .value("RCUTILS_LOG_SEVERITY_UNSET", RCUTILS_LOG_SEVERITY_UNSET)
  .value("RCUTILS_LOG_SEVERITY_DEBUG", RCUTILS_LOG_SEVERITY_DEBUG)
  .value("RCUTILS_LOG_SEVERITY_INFO", RCUTILS_LOG_SEVERITY_INFO)
  .value("RCUTILS_LOG_SEVERITY_WARN", RCUTILS_LOG_SEVERITY_WARN)
  .value("RCUTILS_LOG_SEVERITY_ERROR", RCUTILS_LOG_SEVERITY_ERROR)
  .value("RCUTILS_LOG_SEVERITY_FATAL", RCUTILS_LOG_SEVERITY_FATAL);

  m.def("rclpy_logging_initialize", &rclpy_logging_initialize);
  m.def("rclpy_logging_shutdown", &rclpy_logging_shutdown);
  m.def("rclpy_logging_set_logger_level", &rclpy_logging_set_logger_level);
  m.def("rclpy_logging_get_logger_effective_level", &rclpy_logging_get_logger_effective_level);
  m.def("rclpy_logging_logger_is_enabled_for", &rclpy_logging_logger_is_enabled_for);
  m.def("rclpy_logging_rcutils_log", &rclpy_logging_rcutils_log);
  m.def("rclpy_logging_severity_level_from_string", &rclpy_logging_severity_level_from_string);
  m.def("rclpy_logging_get_logging_directory", &rclpy_logging_get_logging_directory);
}
}  // namespace rclpy
