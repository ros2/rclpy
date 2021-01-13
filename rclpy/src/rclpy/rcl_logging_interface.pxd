# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rcutils cimport rcutils_allocator_t

cdef extern from "<rcl_logging_interface/rcl_logging_interface.h>":
    ctypedef int rcl_logging_ret_t
    int RCL_LOGGING_RET_OK
    int RCL_LOGGING_RET_ERROR
    int RCL_LOGGING_RET_INVALID_ARGUMENT
    int RCL_LOGGING_RET_CONFIG_FILE_DOESNT_EXIST
    int RCL_LOGGING_RET_CONFIG_FILE_INVALID

    rcl_logging_ret_t rcl_logging_get_logging_directory(rcutils_allocator_t allocator, char ** directory)
