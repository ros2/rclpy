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

#ifndef RCLPY__SERVICE_HPP_
#define RCLPY__SERVICE_HPP_

#include <pybind11/pybind11.h>

#include <rmw/types.h>
#include <string>

namespace py = pybind11;

namespace rclpy
{
/// Create a service server
/**
 * This function will create a service server for the given service name.
 * This service will use the typesupport defined in the service module
 * provided as pysrv_type to send messages over the wire.
 *
 * On a successful call a Capsule pointing to the pointer of the created rcl_service_t *
 * is returned.
 *
 * Raises ValueError if the capsules are not the correct types
 * Raises RCLError if the service could not be created
 *
 * \param[in] pynode Capsule pointing to the node to add the service to
 * \param[in] pysrv_type Service module associated with the service
 * \param[in] service_name Python object for the service name
 * \param[in] pyqos_profile rmw_qos_profile_t object for this service
 * \return capsule containing the rcl_service_t
 */
py::capsule
service_create(
  py::capsule pynode, py::object pysrv_type, std::string service_name,
  py::object pyqos_profile);

/// Publish a response message
/**
 * Raises ValueError if the capsules are not the correct types
 * Raises RCLError if the response could not be sent
 *
 * \param[in] pyservice Capsule pointing to the service
 * \param[in] pyresponse reply message to send
 * \param[in] header Pointer to the rmw_request_id_t header of the request we respond to
 */
void
service_send_response(py::capsule pyservice, py::object pyresponse, rmw_request_id_t * header);

/// Take a request from a given service
/**
 * Raises ValueError if pyservice is not a service capsule
 * Raises RCLError if the take failed
 *
 * \param[in] pyservice Capsule pointing to the service to process the request
 * \param[in] pyrequest_type Instance of the message type to take
 * \return None if there was nothing to take, or
 * \return List with 2 elements:
 *            first element: a Python request message with all fields populated with received request
 *            second element: The rmw_request_id header of the processed request
 */
py::object
service_take_request(py::capsule pyservice, py::object pyrequest_type);
}  // namespace rclpy

#endif  // RCLPY__SERVICE_HPP_
