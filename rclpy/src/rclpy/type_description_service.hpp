// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef RCLPY__TYPE_DESCRIPTION_SERVICE_HPP_
#define RCLPY__TYPE_DESCRIPTION_SERVICE_HPP_

#include <pybind11/pybind11.h>

#include <rmw/types.h>

#include <memory>

#include "destroyable.hpp"
#include "service.hpp"

namespace py = pybind11;

namespace rclpy
{

class TypeDescriptionService
  : public Destroyable, public std::enable_shared_from_this<TypeDescriptionService>
{
public:
  /// Initialize and contain the rcl implementation of ~/get_type_description
  /**
   * \param[in] node Node to add the service to
   */
  explicit TypeDescriptionService(Node & node);

  ~TypeDescriptionService() = default;

  /// Return the wrapped rcl service, so that it can be added to the node waitsets
  /**
   * \return The capsule containing the Service
   */
  Service
  get_impl();

  /// Handle an incoming request to the service
  /**
   * \param[in] pyrequest incoming request to handle
   * \param[in] pyresponse_type Python type of the response object to wrap the C message in
   * \param[in] node The node that this service belongs to
   * \return response message to send
   */
  py::object
  handle_request(py::object pyrequest, py::object pyresponse_type, Node & node);

  /// Force early cleanup of object
  void
  destroy() override;

private:
  std::shared_ptr<Service> service_;
};

/// Define a pybind11 wrapper for an rclpy::TypeDescriptionService
void
define_type_description_service(py::object module);
}  // namespace rclpy

#endif  // RCLPY__TYPE_DESCRIPTION_SERVICE_HPP_
