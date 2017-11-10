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
#ifndef RCLPY__SIGINT_GC_VISIBILITY_H_
#define RCLPY__SIGINT_GC_VISIBILITY_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RCLPY_SIGINT_EXPORT __attribute__ ((dllexport))
    #define RCLPY_SIGINT_IMPORT __attribute__ ((dllimport))
  #else
    #define RCLPY_SIGINT_EXPORT __declspec(dllexport)
    #define RCLPY_SIGINT_IMPORT __declspec(dllimport)
  #endif
  #ifdef RCLPY_SIGINT_BUILDING_LIBRARY
    #define RCLPY_SIGINT_PUBLIC RCLPY_SIGINT_EXPORT
  #else
    #define RCLPY_SIGINT_PUBLIC RCLPY_SIGINT_IMPORT
  #endif
  #define RCLPY_SIGINT_PUBLIC_TYPE RCLPY_SIGINT_PUBLIC
  #define RCLPY_SIGINT_LOCAL
#else
  #define RCLPY_SIGINT_EXPORT __attribute__ ((visibility("default")))
  #define RCLPY_SIGINT_IMPORT
  #if __GNUC__ >= 4
    #define RCLPY_SIGINT_PUBLIC __attribute__ ((visibility("default")))
    #define RCLPY_SIGINT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RCLPY_SIGINT_PUBLIC
    #define RCLPY_SIGINT_LOCAL
  #endif
  #define RCLPY_SIGINT_PUBLIC_TYPE
#endif

#endif  // RCLPY__SIGINT_GC_VISIBILITY_H_
