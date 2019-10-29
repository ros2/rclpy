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

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import DescribeParameters, GetParameters, GetParameterTypes
from rcl_interfaces.srv import ListParameters, SetParameters, SetParametersAtomically
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.parameter import Parameter, PARAMETER_SEPARATOR_STRING
from rclpy.qos import qos_profile_parameters
from rclpy.validate_topic_name import TOPIC_SEPARATOR_STRING


class ParameterService:

    def __init__(self, node):
        self._node = node
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
        for name in request.names:
            try:
                descriptor = self._node.describe_parameter(name)
            except ParameterNotDeclaredException:
                descriptor = ParameterDescriptor()
            response.descriptors.append(descriptor)
        return response

    def _get_parameters_callback(self, request, response):
        for name in request.names:
            p = self._node.get_parameter_or(name)
            response.values.append(p.get_parameter_value())
        return response

    def _get_parameter_types_callback(self, request, response):
        for name in request.names:
            response.types.append(self._node.get_parameter_or(name).type_)
        return response

    def _list_parameters_callback(self, request, response):
        names_with_prefixes = []
        for name in self._node._parameters.keys():
            if PARAMETER_SEPARATOR_STRING in name:
                names_with_prefixes.append(name)
                continue
            elif request.prefixes:
                for prefix in request.prefixes:
                    if name.startswith(prefix):
                        response.result.names.append(name)
                continue
            else:
                response.result.names.append(name)
        if 1 == request.depth:
            return response

        if not request.DEPTH_RECURSIVE == request.depth:
            names_with_prefixes = filter(
                lambda name:
                    name.count(PARAMETER_SEPARATOR_STRING) < request.depth, names_with_prefixes
            )
        for name in names_with_prefixes:
            if request.prefixes:
                for prefix in request.prefixes:
                    if name.startswith(prefix + PARAMETER_SEPARATOR_STRING):
                        response.result.names.append(name)
                        full_prefix = PARAMETER_SEPARATOR_STRING.join(
                            name.split(PARAMETER_SEPARATOR_STRING)[0:-1])
                        if full_prefix not in response.result.prefixes:
                            response.result.prefixes.append(full_prefix)
                        if prefix not in response.result.prefixes:
                            response.result.prefixes.append(prefix)
            else:
                prefix = PARAMETER_SEPARATOR_STRING.join(
                    name.split(PARAMETER_SEPARATOR_STRING)[0:-1])
                if prefix not in response.result.prefixes:
                    response.result.prefixes.append(prefix)
                response.result.names.append(name)

        return response

    def _set_parameters_callback(self, request, response):
        for p in request.parameters:
            param = Parameter.from_parameter_msg(p)
            try:
                result = self._node.set_parameters_atomically([param])
            except ParameterNotDeclaredException as e:
                result = SetParametersResult(
                    successful=False,
                    reason=str(e)
                )
            response.results.append(result)
        return response

    def _set_parameters_atomically_callback(self, request, response):
        try:
            response.result = self._node.set_parameters_atomically([
                Parameter.from_parameter_msg(p) for p in request.parameters])
        except ParameterNotDeclaredException as e:
            response.result = SetParametersResult(
                    successful=False,
                    reason=str(e)
                )
        return response
