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

#ifndef RCLPY__WAKEUP_SOCKET_HPP_
#define RCLPY__WAKEUP_SOCKET_HPP_

#include <cstdint>

namespace rclpy
{
/// Write one byte to a socket.
/**
 * The socket must be non-blocking.
 * \param[in] handle POSIX file descriptor or windows SOCKET.
 */
void send_wakeup_byte(std::uintptr_t handle);
}  // namespace rclpy

#endif  // RCLPY__WAKEUP_SOCKET_HPP_
