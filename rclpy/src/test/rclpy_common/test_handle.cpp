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

#include <gtest/gtest.h>
#include <memory>

#include "rclpy_common/handle.h"

void int_destructor(void * p)
{
  *reinterpret_cast<int *>(p) += 1;
}

TEST(test_handle, basic_usage) {
  auto test_obj = std::make_unique<int>(0);
  rclpy_handle_t * handle =
    _rclpy_create_handle(test_obj.get(), int_destructor);
  ASSERT_NE(nullptr, handle);
  ASSERT_EQ(test_obj.get(), _rclpy_handle_get_pointer(handle));
  EXPECT_EQ(0, *test_obj);
  _rclpy_handle_dec_ref(handle);
  EXPECT_EQ(1, *test_obj);
}

TEST(test_handle, destroy_dependency_first) {
  // Construct two handles.
  auto dependent_obj = std::make_unique<int>(0);
  auto dependency_obj = std::make_unique<int>(0);
  rclpy_handle_t * dependent_handle =
    _rclpy_create_handle(dependent_obj.get(), int_destructor);
  rclpy_handle_t * dependency_handle =
    _rclpy_create_handle(dependency_obj.get(), int_destructor);
  ASSERT_NE(nullptr, dependent_handle);
  ASSERT_EQ(dependent_obj.get(), _rclpy_handle_get_pointer(dependent_handle));
  ASSERT_NE(nullptr, dependency_handle);
  ASSERT_EQ(dependency_obj.get(), _rclpy_handle_get_pointer(dependency_handle));
  EXPECT_EQ(0, *dependent_obj);
  EXPECT_EQ(0, *dependency_obj);

  // Add dependency.
  _rclpy_handle_add_dependency(dependent_handle, dependency_handle);

  // Decrement reference of dependency, nothing is destructed.
  _rclpy_handle_dec_ref(dependency_handle);
  ASSERT_EQ(dependent_obj.get(), _rclpy_handle_get_pointer(dependent_handle));
  ASSERT_EQ(dependency_obj.get(), _rclpy_handle_get_pointer(dependency_handle));
  EXPECT_EQ(0, *dependent_obj);
  EXPECT_EQ(0, *dependency_obj);

  // Decrement reference of dependent, both are destructed.
  _rclpy_handle_dec_ref(dependent_handle);
  EXPECT_EQ(1, *dependent_obj);
  EXPECT_EQ(1, *dependency_obj);
}

TEST(test_handle, destroy_dependent_first) {
  // Construct two handles.
  auto dependent_obj = std::make_unique<int>(0);
  auto dependency_obj = std::make_unique<int>(0);
  rclpy_handle_t * dependent_handle =
    _rclpy_create_handle(dependent_obj.get(), int_destructor);
  rclpy_handle_t * dependency_handle =
    _rclpy_create_handle(dependency_obj.get(), int_destructor);
  ASSERT_NE(nullptr, dependent_handle);
  ASSERT_EQ(dependent_obj.get(), _rclpy_handle_get_pointer(dependent_handle));
  ASSERT_NE(nullptr, dependency_handle);
  ASSERT_EQ(dependency_obj.get(), _rclpy_handle_get_pointer(dependency_handle));
  EXPECT_EQ(0, *dependent_obj);
  EXPECT_EQ(0, *dependency_obj);

  // Add dependency.
  _rclpy_handle_add_dependency(dependent_handle, dependency_handle);

  // Decrement reference of dependent, destructed right away.
  _rclpy_handle_dec_ref(dependent_handle);
  ASSERT_EQ(dependency_obj.get(), _rclpy_handle_get_pointer(dependency_handle));
  EXPECT_EQ(1, *dependent_obj);
  EXPECT_EQ(0, *dependency_obj);

  // Decrement reference of dependency, destructed right away.
  _rclpy_handle_dec_ref(dependency_handle);
  EXPECT_EQ(1, *dependent_obj);
  EXPECT_EQ(1, *dependency_obj);
}
