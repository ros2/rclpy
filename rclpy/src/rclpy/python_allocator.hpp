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

#ifndef RCLPY__PYTHON_ALLOCATOR_HPP_
#define RCLPY__PYTHON_ALLOCATOR_HPP_

#include <pybind11/pybind11.h>

#include <stdexcept>

namespace rclpy
{
//----------- Declaration ----------

/// Allocate memory using Python's allocator
/**
 * \warning The GIL must be held while the allocator is used.
 */
template<class T>
struct PythonAllocator
{
  using value_type = T;

  PythonAllocator() noexcept = default;

  template<class U>
  PythonAllocator(const PythonAllocator<U> & /* other */) noexcept {}

  T * allocate(std::size_t n);

  void deallocate(T * p, std::size_t n);
};

template<class T, class U>
constexpr bool operator==(const PythonAllocator<T> &, const PythonAllocator<U> &) noexcept;

template<class T, class U>
constexpr bool operator!=(const PythonAllocator<T> &, const PythonAllocator<U> &) noexcept;

//----------- Implementation ----------

template<class T>
T * PythonAllocator<T>::allocate(std::size_t n)
{
  T * ptr = static_cast<T *>(PyMem_Malloc(n * sizeof(T)));
  if (nullptr == ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}

template<class T>
void PythonAllocator<T>::deallocate(T * p, std::size_t /* n */)
{
  PyMem_Free(p);
}

template<class T, class U>
constexpr bool operator==(const PythonAllocator<T> &, const PythonAllocator<U> &) noexcept
{
  return true;
}

template<class T, class U>
constexpr bool operator!=(const PythonAllocator<T> &, const PythonAllocator<U> &) noexcept
{
  return false;
}
}  // namespace rclpy

#endif  // RCLPY__PYTHON_ALLOCATOR_HPP_
