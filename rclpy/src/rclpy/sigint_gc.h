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
#ifndef RCLPY__SIGINT_GC_H_
#define RCLPY__SIGINT_GC_H_

#include <signal.h>

#include <rcl/rcl.h>

/// Global variable with guard condition for sigint handler
extern rcl_guard_condition_t * g_rclpy_sigint_gc_handle;

/// Function that can be used as a sigint handler
extern void rclpy_catch_function(int signo);

#endif  // RCLPY__SIGINT_GC_H_
