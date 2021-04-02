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

#include <pybind11/embed.h>

#include <gtest/gtest.h>

#include <vector>

#include "python_allocator.hpp"

namespace py = pybind11;

TEST(test_allocator, vector) {
  py::scoped_interpreter guard{};  // Start a Python interpreter

  std::vector<int, rclpy::PythonAllocator<int>> container(42);

  EXPECT_EQ(42u, container.capacity());
  ASSERT_EQ(42u, container.size());

  for (int i = 0; i < 42; ++i) {
    container[i] = i;
  }
}

TEST(test_allocator, equality) {
  py::scoped_interpreter guard{};  // Start a Python interpreter

  rclpy::PythonAllocator<int> int_alloc;
  rclpy::PythonAllocator<float> float_alloc;

  EXPECT_TRUE(int_alloc == float_alloc);
  EXPECT_FALSE(int_alloc != float_alloc);
}

TEST(test_allocator, make_1) {
  py::scoped_interpreter guard{};  // Start a Python interpreter

  rclpy::PythonAllocator<int> int_alloc;

  int * i = int_alloc.allocate(1);

  ASSERT_NE(nullptr, i);

  int_alloc.deallocate(i, 1);
}

TEST(test_allocator, copy_construct_make_1) {
  py::scoped_interpreter guard{};  // Start a Python interpreter

  rclpy::PythonAllocator<float> float_alloc;
  rclpy::PythonAllocator<int> int_alloc(float_alloc);

  int * i = int_alloc.allocate(1);

  ASSERT_NE(nullptr, i);

  int_alloc.deallocate(i, 1);
}
