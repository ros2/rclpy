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

#ifndef RCLPY__CLIENT_HPP_
#define RCLPY__CLIENT_HPP_

#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

namespace rclpy
{
/// Create a client
/**
 * This function will create a client for the given service name.
 * This client will use the typesupport defined in the service module
 * provided as pysrv_type to send messages over the wire.
 *
 * On a successful call a Capsule pointing to the pointer of the created
 * rclpy_client_t * is returned.
 *
 * Raises ValueError if the capsules are not the correct types
 * Raises RuntimeError if the client could not be created
 *
 * \param[in] pynode Capsule pointing to the node to add the client to
 * \param[in] pysrv_type Service module associated with the client
 * \param[in] service_name Python object containing the service name
 * \param[in] pyqos_profile QoSProfile Python object for this client
 * \return capsule containing the rclpy_client_t
 */
py::capsule
client_create(
  py::capsule pynode, py::object pysrv_type, std::string service_name,
  py::capsule pyqos_profile);

/// Publish a request message
/**
 * Raises ValueError if pyclient is not a client capsule
 * Raises RuntimeError if the request could not be sent
 *
 * \param[in] pyclient Capsule pointing to the client
 * \param[in] pyrequest request message to send
 * \return sequence_number Index of the sent request
 */
int64_t
client_send_request(py::capsule pyclient, py::object pyrequest);

/// Check if a service server is available
/**
 * Raises ValueError if the arguments are not capsules
 *
 * \param[in] pyclient Capsule pointing to the client
 * \return True if the service server is available
 */
bool
client_service_server_is_available(py::capsule pyclient);

/// Take a response from a given client
/**
 * Raises ValueError if pyclient is not a client capsule
 *
 * \param[in] pyclient Capsule pointing to the client to process the response
 * \param[in] pyresponse_type Instance of the message type to take
 * \return 2-tuple sequence number and received response or None, None if there is no response
 */
py::tuple
client_take_response(py::capsule pyclient, py::object pyresponse_type);
}  // namespace rclpy

#endif  // RCLPY__CLIENT_HPP_
