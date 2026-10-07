// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/socket.h>
#endif

#include <cstdint>

#include "wakeup_socket.hpp"

namespace rclpy
{
void send_wakeup_byte(std::uintptr_t handle)
{
  const char byte = 1;
#ifdef _WIN32
  (void)::send(static_cast<SOCKET>(handle), &byte, 1, 0);
#else
  (void)::send(static_cast<int>(handle), &byte, 1, 0);
#endif
}
}  // namespace rclpy
