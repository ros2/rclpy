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

#include <rcl/context.h>
#include <rcl/types.h>

#include <stdexcept>

#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "context.hpp"

namespace rclpy
{
size_t
context_get_domain_id(py::capsule pycontext)
{
  auto context = static_cast<rcl_context_t *>(
    rclpy_handle_get_pointer_from_capsule(pycontext.ptr(), "rcl_context_t"));
  if (!context) {
    throw py::error_already_set();
  }

  size_t domain_id;
  rcl_ret_t ret = rcl_context_get_domain_id(context, &domain_id);
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to get domain id");
  }

  return domain_id;
}
}  // namespace rclpy
