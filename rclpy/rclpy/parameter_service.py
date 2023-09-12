# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import weakref

from rcl_interfaces.msg import ListParametersResult
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import DescribeParameters, GetParameters, GetParameterTypes
from rcl_interfaces.srv import ListParameters, SetParameters, SetParametersAtomically
from rclpy.exceptions import ParameterNotDeclaredException, ParameterUninitializedException
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_parameters
from rclpy.validate_topic_name import TOPIC_SEPARATOR_STRING


class ParameterService:

    def __init__(self, node):
        self._node_weak_ref = weakref.ref(node)
        nodename = node.get_name()

        describe_parameters_service_name = \
            TOPIC_SEPARATOR_STRING.join((nodename, 'describe_parameters'))
        node.create_service(
            DescribeParameters, describe_parameters_service_name,
            self._describe_parameters_callback, qos_profile=qos_profile_parameters
        )
        get_parameters_service_name = TOPIC_SEPARATOR_STRING.join((nodename, 'get_parameters'))
        node.create_service(
            GetParameters, get_parameters_service_name, self._get_parameters_callback,
            qos_profile=qos_profile_parameters
        )
        get_parameter_types_service_name = \
            TOPIC_SEPARATOR_STRING.join((nodename, 'get_parameter_types'))
        node.create_service(
            GetParameterTypes, get_parameter_types_service_name,
            self._get_parameter_types_callback, qos_profile=qos_profile_parameters
        )
        list_parameters_service_name = TOPIC_SEPARATOR_STRING.join((nodename, 'list_parameters'))
        node.create_service(
            ListParameters, list_parameters_service_name, self._list_parameters_callback,
            qos_profile=qos_profile_parameters
        )
        set_parameters_service_name = TOPIC_SEPARATOR_STRING.join((nodename, 'set_parameters'))
        node.create_service(
            SetParameters, set_parameters_service_name, self._set_parameters_callback,
            qos_profile=qos_profile_parameters
        )
        set_parameters_atomically_service_name = \
            TOPIC_SEPARATOR_STRING.join((nodename, 'set_parameters_atomically'))
        node.create_service(
            SetParametersAtomically, set_parameters_atomically_service_name,
            self._set_parameters_atomically_callback,
            qos_profile=qos_profile_parameters
        )

    def _describe_parameters_callback(self, request, response):
        node = self._get_node()
        for name in request.names:
            try:
                descriptor = node.describe_parameter(name)
            except ParameterNotDeclaredException:
                response.descriptors = node.describe_parameters([])
                return response
            response.descriptors.append(descriptor)
        return response

    def _get_parameters_callback(self, request, response):
        node = self._get_node()
        for name in request.names:
            try:
                param = node.get_parameter(name)
            except (ParameterNotDeclaredException, ParameterUninitializedException):
                response.values = node.get_parameters([])
                return response
            response.values.append(param.get_parameter_value())
        return response

    def _get_parameter_types_callback(self, request, response):
        node = self._get_node()
        for name in request.names:
            try:
                value = node.get_parameter_type(name)
            except ParameterNotDeclaredException:
                response.types = node.get_parameter_types([])
                return response
            response.types.append(value)
        return response

    def _list_parameters_callback(self, request, response):
        node = self._get_node()
        try:
            response.result = node.list_parameters(request.prefixes, request.depth)
        except (TypeError, ValueError):
            response.result = ListParametersResult()
        return response

    def _set_parameters_callback(self, request, response):
        node = self._get_node()
        for p in request.parameters:
            param = Parameter.from_parameter_msg(p)
            try:
                result = node.set_parameters_atomically([param])
            except ParameterNotDeclaredException as e:
                result = SetParametersResult(
                    successful=False,
                    reason=str(e)
                )
            response.results.append(result)
        return response

    def _set_parameters_atomically_callback(self, request, response):
        node = self._get_node()
        try:
            response.result = node.set_parameters_atomically([
                Parameter.from_parameter_msg(p) for p in request.parameters])
        except ParameterNotDeclaredException as e:
            response.result = SetParametersResult(
                    successful=False,
                    reason=str(e)
                )
        return response

    def _get_node(self):
        node = self._node_weak_ref()
        if node is None:
            raise ReferenceError('Expected valid node weak reference')
        return node
