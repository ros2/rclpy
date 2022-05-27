# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import time
from typing import Callable
from typing import List
from typing import Optional
from typing import Sequence
from typing import Union

from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import GetParameterTypes
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.parameter import parameter_dict_from_yaml_file
from rclpy.qos import qos_profile_services_default
from rclpy.qos import QoSProfile
from rclpy.task import Future


class AsyncParameterClient(object):
    def __init__(
            self,
            node: Node,
            target_node_name: str,
            qos_profile: QoSProfile = qos_profile_services_default,
            callback_group: Optional[CallbackGroup] = None):
        # TODO: qos_profile, callback group
        """
        Create an AsyncParameterClient.

        For more on service names, see: `ROS 2 docs`_.


        .. _ROS 2 docs: https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Parameters.html#interacting-with-parameters # noqa E501

        :param node: Node for which the parameter clients will be added to
        :param target_node_name: Name of remote node for which the parameters will be managed
        """
        self.target_node = target_node_name
        self.node = node
        self.get_parameter_client_ = self.node.create_client(
            GetParameters, f'{target_node_name}/get_parameters',
            qos_profile=qos_profile, callback_group=callback_group
        )
        self.list_parameter_client_ = self.node.create_client(
            ListParameters, f'{target_node_name}/list_parameters',
            qos_profile=qos_profile, callback_group=callback_group
        )
        self.set_parameter_client_ = self.node.create_client(
            SetParameters, f'{target_node_name}/set_parameters',
            qos_profile=qos_profile, callback_group=callback_group
        )
        self.get_parameter_types_client_ = self.node.create_client(
            GetParameterTypes, f'{target_node_name}/get_parameter_types',
            qos_profile=qos_profile, callback_group=callback_group
        )
        self.describe_parameters_client_ = self.node.create_client(
            DescribeParameters, f'{target_node_name}/describe_parameters',
            qos_profile=qos_profile, callback_group=callback_group
        )
        self.set_parameters_atomically_client_ = self.node.create_client(
            SetParametersAtomically, f'{target_node_name}/set_parameters_atomically',
            qos_profile=qos_profile, callback_group=callback_group
        )

    def service_is_ready(self) -> bool:
        """
        Check if all services are ready.

        :return: ``True`` if all services are available, False otherwise.
        """
        return all([
            self.list_parameter_client_.service_is_ready(),
            self.set_parameter_client_.service_is_ready(),
            self.get_parameter_client_.service_is_ready(),
            self.get_parameter_types_client_.service_is_ready(),
            self.describe_parameters_client_.service_is_ready(),
            self.set_parameters_atomically_client_.service_is_ready(),
        ])

    def wait_for_service(self, timeout_sec: Optional[float] = None) -> bool:
        """
        Wait for all parameter services to be available.

        :param timeout_sec: Seconds to wait. If ``None``, then wait forever.
        :type timeout_sec: Union[float, None]
        :return: ``True`` if all services were waite , ``False`` otherwise.
        """
        client_wait_fns = [
                self.list_parameter_client_.wait_for_service,
                self.set_parameter_client_.wait_for_service,
                self.get_parameter_client_.wait_for_service,
                self.get_parameter_types_client_.wait_for_service,
                self.describe_parameters_client_.wait_for_service,
                self.set_parameters_atomically_client_.wait_for_service,
        ]

        if timeout_sec is None:
            return all([fn() for fn in client_wait_fns])

        prev = time.time()
        for wait_for_service in client_wait_fns:
            if timeout_sec < 0 or not wait_for_service(timeout_sec):
                return False
            timeout_sec -= time.time() - prev
            prev = time.time()
        return True

    def list_parameters(
        self,
        prefixes: Optional[List[str]] = None,
        depth: int = 1,
        callback: Optional[Callable] = None
    ) -> Future:
        """
        List all parameters with given prefixs.

        :param prefixes: List of prefixes to filter by.
        :param depth: Depth of the parameter tree to list.
        :param callback: Callback function to call when the request is complete.
        :return: ``Future`` with the result of the request.
        """
        request = ListParameters.Request()
        if prefixes:
            request.prefixes = prefixes
        request.depth = depth
        future = self.list_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def get_parameters(self, names: List[str], callback: Optional[Callable] = None) -> Future:
        """
        Get parameters given names.

        :param names: List of parameter names to get.
        :param callback: Callback function to call when the request is complete.
        :return: ``Future`` with the result of the request.
        """
        request = GetParameters.Request()
        request.names = names
        future = self.get_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def set_parameters(
        self,
        parameters: Sequence[Parameter],
        callback: Union[Callable, None] = None
    ) -> Future:
        """
        Set parameters given a list of parameters.

        The result after the returned future is complete
        will be of type ``rcl_interfaces.srv.SetParameters.Response``.

        :param parameters: Sequence of parameters to set.
        :param callback: Callback function to call when the request is complete.
        :return: ``Future`` with the result of the request.
        """
        request = SetParameters.Request()
        request.parameters = parameters
        future = self.set_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def describe_parameters(
        self,
        names: List[str],
        callback: Optional[Callable] = None
    ) -> Future:
        """
        Describe parameters given names.

        The result after the returned future is complete
        will be of type ``rcl_interfaces.srv.DescribeParameters.Response``.

        :param names: List of parameter names to describe
        :param callback: Callback function to call when the request is complete.
        :return: ``Future`` with the result of the request.
        """
        request = DescribeParameters.Request()
        request.names = names
        future = self.describe_parameters_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def get_parameter_types(
        self,
        names: List[str],
        callback: Optional[Callable] = None
    ) -> Future:
        """
        Get parameter types given names.

        The result after the returned future is complete
        will be of type ``rcl_interfaces.srv.GetParameterTypes.Response``.

        Parameter type definitions are given in rcl_interfaces.msg.ParameterType

        :param names: List of parameter names to get types for.
        :param callback: Callback function to call when the request is complete.
        :return: ``Future`` with the result of the request.
        """
        request = GetParameterTypes.Request()
        request.names = names
        future = self.get_parameter_types_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def set_parameters_atomically(
        self,
        parameters: Sequence[Parameter],
        callback: Optional[Callable] = None
    ) -> Future:
        """
        Set parameters atomically.

        The result after the returned future is complete
        will be of type ``rcl_interfaces.srv.SetParametersAtomically.Response``.

        :param parameters: Sequence of parameters to set.
        :param callback: Callback function to call when the request is complete.
        :return: ``Future`` with the result of the request.
        """
        request = SetParametersAtomically.Request()
        request.parameters = parameters
        future = self.set_parameters_atomically_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def delete_parameters(
        self, names: List[str], callback: Optional[Callable] = None
    ) -> Future:
        """
        Unset parameters with given names.

        The result after the returned future is complete
        will be of type ``rcl_interfaces.srv.SetParameters.Response``.

        :param names: List of parameter names to unset.
        :param callback: Callback function to call when the request is complete.
        :return: ``Future`` with the result of the request.
        """
        request = SetParameters.Request()
        request.parameters = [RclpyParameter(name=i).to_parameter_msg() for i in names]
        future = self.set_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def load_parameter_file(
        self,
        parameter_file: str,
        use_wildcard: bool = False,
    ) -> Future:
        """
        Load parameters from a yaml file.

        Wrapper around `parse_parameter_dict` and `load_dict`

        The result after the returned future is complete
        will be of type ``rcl_interfaces.srv.SetParameters.Response``.

        :param parameter_file: Path to the parameter file.
        :param use_wildcard: Whether to use wildcard expansion.
        :return: Future with the result from the set_parameter call.
        """
        param_dict = parameter_dict_from_yaml_file(parameter_file, use_wildcard)
        future = self.set_parameters(list(param_dict.values()))
        return future
