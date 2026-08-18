# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Optional, TYPE_CHECKING
import weakref

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_services_default
from rclpy.type_support import check_is_valid_srv_type
from rclpy.validate_topic_name import TOPIC_SEPARATOR_STRING

from type_description_interfaces.srv import GetTypeDescription

if TYPE_CHECKING:
    from rclpy.node import BaseNode

START_TYPE_DESCRIPTION_SERVICE_PARAM = 'start_type_description_service'


class TypeDescriptionService:
    """
    Optionally initializes and contains the ~/get_type_description service.

    The service is implemented in rcl, but should be enabled via parameter and have its
    callbacks handled via end-client execution framework, such as callback groups and waitsets.

    This is not intended for use by end users, rather it is a component to be used by BaseNode.
    """

    def __init__(self, node: 'BaseNode'):
        """Initialize the service, if the parameter is set to true."""
        self._node_weak_ref = weakref.ref(node)
        node_name = node.get_name()
        self.service_name = TOPIC_SEPARATOR_STRING.join((node_name, 'get_type_description'))
        self._type_description_srv: Optional[_rclpy.TypeDescriptionService] = None

        self.enabled = False
        if not node.has_parameter(START_TYPE_DESCRIPTION_SERVICE_PARAM):
            descriptor = ParameterDescriptor(
                name=START_TYPE_DESCRIPTION_SERVICE_PARAM,
                type=ParameterType.PARAMETER_BOOL,
                description='If enabled, start the ~/get_type_description service.',
                read_only=True)
            node.declare_parameter(
                START_TYPE_DESCRIPTION_SERVICE_PARAM,
                True,
                descriptor)
        param = node.get_parameter(START_TYPE_DESCRIPTION_SERVICE_PARAM)
        if param.type_ != Parameter.Type.NOT_SET:
            if param.type_ == Parameter.Type.BOOL:
                self.enabled = param.value
            else:
                node.get_logger().error(
                    "Invalid type for parameter '{}' {!r} should be bool"
                    .format(START_TYPE_DESCRIPTION_SERVICE_PARAM, param.type_))
        else:
            node.get_logger().debug(
                'Parameter {} not set, defaulting to true.'
                .format(START_TYPE_DESCRIPTION_SERVICE_PARAM))

        if self.enabled:
            self._start_service()

    def _start_service(self) -> None:
        node = self._get_node()
        self._type_description_srv = _rclpy.TypeDescriptionService(node.handle)
        check_is_valid_srv_type(GetTypeDescription)
        node._create_service(
            self._type_description_srv.impl,
            GetTypeDescription,
            self.service_name,
            self._service_callback,
            qos_profile_services_default)

    def _service_callback(
        self,
        request: GetTypeDescription.Request,
        response: GetTypeDescription.Response
    ) -> GetTypeDescription.Response:
        if self._type_description_srv is None:
            raise RuntimeError('Cannot handle request if TypeDescriptionService is None.')
        return self._type_description_srv.handle_request(
            request, GetTypeDescription.Response, self._get_node().handle)

    def _get_node(self) -> 'BaseNode':
        node = self._node_weak_ref()
        if node is None:
            raise ReferenceError('Expected valid node weak reference')
        return node
