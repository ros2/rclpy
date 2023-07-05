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

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_services_default
from rclpy.validate_topic_name import TOPIC_SEPARATOR_STRING
from rclpy.service import Service
from type_description_interfaces.srv import GetTypeDescription

START_TYPE_DESCRIPTION_SERVICE_PARAM = 'start_type_description_service'


class TypeDescriptionService:
    """
    """

    def __init__(self, node):
        node_name = node.get_name()
        self.service_name = TOPIC_SEPARATOR_STRING.join((node_name, 'get_type_description'))

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
            self.start_service(node)

    def start_service(self, node):
        self._service_impl = _rclpy.TypeDescriptionService(node.handle)

    def _service_callback(self, request, response):
        self._service_impl.handle_request(request, response)
