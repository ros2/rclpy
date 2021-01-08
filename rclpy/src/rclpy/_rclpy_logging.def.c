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

/* _rclpy_logging Definitions
 *
 * This is not a true C file.
 * It is parsed by python-cffi and used to generate code for a CPython extension.
 * Everything in this file will be made accessible to Python.
 */

typedef int rcutils_ret_t;

#define RCUTILS_RET_OK 0
#define RCUTILS_RET_WARN 1
#define RCUTILS_RET_ERROR 2
#define RCUTILS_RET_BAD_ALLOC 10
#define RCUTILS_RET_INVALID_ARGUMENT 11
#define RCUTILS_RET_NOT_ENOUGH_SPACE 12
#define RCUTILS_RET_NOT_INITIALIZED 13
#define RCUTILS_RET_NOT_FOUND 14
#define RCUTILS_RET_STRING_MAP_ALREADY_INIT 30
#define RCUTILS_RET_STRING_MAP_INVALID 31
#define RCUTILS_RET_STRING_KEY_NOT_FOUND 32
#define RCUTILS_RET_LOGGING_SEVERITY_MAP_INVALID 40
#define RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID 41
#define RCUTILS_RET_HASH_MAP_NO_MORE_ENTRIES 50
#define RCUTILS_ERROR_MESSAGE_MAX_LENGTH 1024

typedef struct rcutils_allocator_t
{
  void * (*allocate)(size_t size, void * state);
  void (* deallocate)(void * pointer, void * state);
  void * (*reallocate)(void * pointer, size_t size, void * state);
  void * (*zero_allocate)(size_t number_of_elements, size_t size_of_element, void * state);
  void * state;
} rcutils_allocator_t;

rcutils_allocator_t rcutils_get_default_allocator();

typedef struct rcutils_error_string_t
{
  char str[RCUTILS_ERROR_MESSAGE_MAX_LENGTH];
} rcutils_error_string_t;

void rcutils_reset_error();

enum RCUTILS_LOG_SEVERITY
{
  RCUTILS_LOG_SEVERITY_UNSET = 0,
  RCUTILS_LOG_SEVERITY_DEBUG = 10,
  RCUTILS_LOG_SEVERITY_INFO = 20,
  RCUTILS_LOG_SEVERITY_WARN = 30,
  RCUTILS_LOG_SEVERITY_ERROR = 40,
  RCUTILS_LOG_SEVERITY_FATAL = 50,
};

typedef struct rcutils_log_location_t
{
  const char * function_name;
  const char * file_name;
  size_t line_number;
} rcutils_log_location_t;

rcutils_ret_t rcutils_logging_initialize();
rcutils_ret_t rcutils_logging_shutdown();
rcutils_ret_t rcutils_logging_set_logger_level(const char * name, int level);
int rcutils_logging_get_logger_effective_level(const char * name);
bool rcutils_logging_logger_is_enabled_for(const char * name, int severity);
void rcutils_log(
  const rcutils_log_location_t * location,
  int severity,
  const char * name,
  const char * format,
  ...);
rcutils_ret_t
rcutils_logging_severity_level_from_string(
  const char * severity_string, rcutils_allocator_t allocator, int * severity);

typedef enum
{
  RCL_LOGGING_RET_OK = 0,
  RCL_LOGGING_RET_ERROR = 2,
  RCL_LOGGING_RET_INVALID_ARGUMENT = 11,
  RCL_LOGGING_RET_CONFIG_FILE_DOESNT_EXIST = 21,
  RCL_LOGGING_RET_CONFIG_FILE_INVALID = 22,
} rcl_logging_ret_t;

rcl_logging_ret_t
rcl_logging_get_logging_directory(rcutils_allocator_t allocator, char ** directory);
