from typing import Callable, List, Sequence, Union

from rcl_interfaces.srv import (
    DescribeParameters,
    GetParameters,
    GetParameterTypes,
    ListParameters,
    SetParameters,
    SetParametersAtomically,
)
import yaml
from rcl_interfaces.msg import Parameter
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.parameter import get_parameter_value
from rclpy.parameter import PARAMETER_SEPARATOR_STRING
from rclpy.task import Future



class AsyncParameterClient(object):
    def __init__(self, node, target_node_name):
        # TODO: qos_profile, callback group
        """Create an AsyncParameterClient.

        :param node: Node for which the parameter clients will be added to
        :type node: rclpy.Node
        :param target_node_name: Name of remote node for which the parameters will be managed
        :type target_node_name: string
        """
        # Service names are defined in rclcpp/parameter_service_names.hpp
        # TODO: Link to them???
        self.target_node = target_node_name
        self.node = node
        self.get_parameter_client_ = self.node.create_client(
            GetParameters, f'{target_node_name}/get_parameters'
        )
        self.list_parameter_client_ = self.node.create_client(
            ListParameters, f'{target_node_name}/list_parameters'
        )
        self.set_parameter_client_ = self.node.create_client(
            SetParameters, f'{target_node_name}/set_parameters'
        )
        self.get_parameter_types_client_ = self.node.create_client(
            GetParameterTypes, f'{target_node_name}/get_parameter_types'
        )
        self.describe_parameters_client_ = self.node.create_client(
            DescribeParameters, f'{target_node_name}/describe_parameters'
        )
        self.set_parameters_atomically_client_ = self.node.create_client(
            SetParametersAtomically, f'{target_node_name}/set_parameters_atomically'
        )

    def service_is_ready(self, ) -> bool:
        """ 
        """
        return all([
            self.list_parameter_client_.service_is_ready(),
            self.set_parameter_client_.service_is_ready(),
            self.get_parameter_client_.service_is_ready(),
            self.get_parameter_types_client_.service_is_ready(),
            self.describe_parameters_client_.service_is_ready(),
            self.set_parameters_atomically_client_.service_is_ready(),
        ])



    def wait_for_service(self, timeout_sec: Union[float, None] = None) -> bool:
        """Wait for all parameter services to be available.

        :param timeout_sec: Seconds to wait. If ``None``, then wait forever.
        :type timeout_sec: Union[float, None]
        :return:
        :rtype:

        :return: ``True`` if all services are available, False otherwise.
        """
        return all([
            self.list_parameter_client_.wait_for_service(timeout_sec),
            self.set_parameter_client_.wait_for_service(timeout_sec),
            self.get_parameter_client_.wait_for_service(timeout_sec),
            self.get_parameter_types_client_.wait_for_service(timeout_sec),
            self.describe_parameters_client_.wait_for_service(timeout_sec),
            self.set_parameters_atomically_client_.wait_for_service(timeout_sec),
        ])

    def list_parameters(
        self, prefixes: Union[None, List[str]] = None, depth: int = 1, callback: Union[Callable, None] = None
    ) -> Future:
        # TODO: add typing/etc to handle listing all

        """ List all parameters with given prefixs.

        :param prefixes:
        :type prefixes: List[str]
        :param depth:
        :type depth: int
        :param callback:
        :type callback: Union[Callable, None]
        :return:
        :rtype: ``rclpy.task.Future``
        """
        request = ListParameters.Request()
        if prefixes:
            request.prefixes = prefixes
        request.depth = depth
        future = self.list_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def get_parameters(
        self, names: List[str], callback: Union[Callable, None] = None
    ) -> Future:
        """

        :param names:
        :type names: List[str]
        :param callback:
        :type callback: Union[Callable, None]
        :return:
        :rtype:
        """
        request = GetParameters.Request()
        request.names = names
        future = self.get_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def set_parameters(
        self, parameters: Sequence[Parameter], callback: Union[Callable, None] = None
    ) -> Future:
        """

        :param parameters:
        :type parameters: Sequence[Parameter]
        :param callback:
        :type callback: Union[Callable, None]
        :return:
        :rtype:
        """
        request = SetParameters.Request()
        request.parameters = parameters
        future = self.set_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def describe_parameters(
        self, names: List[str], callback: Union[Callable, None] = None
    ) -> Future:
        """

        :param names:
        :type names: List[str]
        :param callback:
        :type callback: Union[Callable, None]
        :return:
        :rtype:

        Parameter type definitions are given in rcl_interfaces.msg.ParameterType

        """
        request = DescribeParameters.Request()
        request.names = names
        future = self.describe_parameters_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def get_parameter_types(
        self, names, callback: Union[Callable, None] = None
    ) -> Future:
        """

        :param names:
        :type names:
        :param callback:
        :type callback: Union[Callable, None]
        :return:
        :rtype:
        """
        request = GetParameterTypes.Request()
        request.names = names
        future = self.get_parameter_types_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def set_parameters_atomically(
        self, parameters: Sequence[Parameter], callback: Union[Callable, None] = None
    ) -> Future:
        """

        :param parameters:
        :type parameters: Union[Set[Parameter], Parameter]
        :param callback:
        :type callback: Union[Callable, None]
        :return:
        :rtype:
        """
        request = SetParametersAtomically.Request()
        request.parameters = parameters
        future = self.set_parameters_atomically_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def delete_parameters(
        self, names: List[str], callback: Union[Callable, None] = None
    ) -> Future:
        # TODO: `Static parameter cannot be undeclared`
        """ Unset parameters with given names.

        :param names:
        :type names: List[str]
        :param callback:
        :type callback: Union[Callable, None]
        :return:
        :rtype:
        """
        request = SetParameters.Request()
        request.parameters = [RclpyParameter(name=i).to_parameter_msg() for i in names]
        future = self.set_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def load_parameter_file(self, parameter_file, use_wildcard, return_parameters=False):
        with open(parameter_file, 'r') as f:
            param_file = yaml.safe_load(f)
            param_keys = []
            if use_wildcard and '/**' in param_file:
                param_keys.append('/**')
            if self.target_node in param_file:
                param_keys.append(self.target_node)

            if param_keys == []:
                raise RuntimeError('Param file does not contain parameters for {}, '
                                   ' only for nodes: {}' .format(node_name, param_file.keys()))
            param_dict = {}
            for k in param_keys:
                value = param_file[k]
                if type(value) != dict or 'ros__parameters' not in value:
                    raise RuntimeError('Invalid structure of parameter file for node {}'
                                       'expected same format as provided by ros2 param dump'
                                       .format(k))
                param_dict.update(value['ros__parameters'])
            parameters = parse_parameter_dict(namespace='', parameter_dict=param_dict)
            future = self.set_parameters(parameters)
            if return_parameters:
                return (future, parameters)
            return future

def parse_parameter_dict(namespace, parameter_dict):
    """
    Builds a list of parameters from a dictionary.
    """
    parameters = []
    for param_name, param_value in parameter_dict.items():
        full_param_name = namespace + param_name
        # Unroll nested parameters
        if type(param_value) == dict:
            parameters += parse_parameter_dict(
                    namespace=full_param_name + PARAMETER_SEPARATOR_STRING,
                    parameter_dict=param_value)
        else:
            parameter = Parameter()
            parameter.name = full_param_name
            parameter.value = get_parameter_value(str(param_value))
            parameters.append(parameter)
    return parameters

