from typing import Callable, List, Sequence, Union

from rcl_interfaces.srv import (
    DescribeParameters,
    GetParameters,
    GetParameterTypes,
    ListParameters,
    SetParameters,
    SetParametersAtomically,
)
from rclpy.parameter import Parameter
from rclpy.task import Future


class AsyncParameterClient:
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
        self.list_parameter_client_ = self.node.create_client(
            ListParameters, f'{target_node_name}/list_parameters'
        )
        self.set_parameter_client_ = self.node.create_client(
            SetParameters, f'{target_node_name}/set_parameters'
        )
        self.get_parameter_client_ = self.node.create_client(
            GetParameters, f'{target_node_name}/get_parameters'
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
        return all(
            [
                self.list_parameter_client_.wait_for_service(timeout_sec),
                self.set_parameter_client_.wait_for_service(timeout_sec),
                self.get_parameter_client_.wait_for_service(timeout_sec),
                self.get_parameter_types_client_.wait_for_service(timeout_sec),
                self.describe_parameters_client_.wait_for_service(timeout_sec),
                self.set_parameters_atomically_client_.wait_for_service(timeout_sec),
            ]
        )

    def list_parameters(
        self, prefixes: List[str], depth: int, callback: Union[Callable, None] = None
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
        request.parameters = [i.to_parameter_msg() for i in parameters]
        future = self.set_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def describe_parameters(
        self, names: Union[List[str], str], callback: Union[Callable, None] = None
    ) -> Future:
        """

        :param names:
        :type names: Union[List[str], str]
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
        request.parameters = [i.to_parameter_msg() for i in parameters]
        # request.parameters = parameters[0].to_parameter_msg()
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
        request.parameters = [Parameter(name=i).to_parameter_msg() for i in names]
        future = self.set_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def load_parameters(
        self, yaml_filename: str, callback: Union[Callable, None] = None
    ) -> Future:
        """ See: https://github.com/ros2/ros2cli/blob/master/ros2param/ros2param/api/__init__.py.

        :param yaml_filename: Full name of the ``yaml`` file
        :type yaml_filename: str
        :param callback: Callback function to perform on future completion
        :type callback: Union[Callable, None]
        :return: Future of set ``set_parameter`` service used to load the parameters
        :rtype: Future
        """
        raise NotImplementedError
