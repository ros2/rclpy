import rclpy
from rclpy import Parameter



from typing import Union
from typing import List
from typing import Callable
from rclpy.task import Future

from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import GetParameterTypes
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import SetParametersAtomically

from rcl_interfaces.msg import ListParametersResult

class AsyncParameterClient:
    def __init__(self, target_node_name):
        """
        Creates an AsyncParameterClient

        :param target_node_name: 
        :type target_node_name: 
        """
        self.target_node = target_node_name
        self.node = rclpy.create_node(f'async_param_client__{target_node_name}')
        self.list_parameter_client_ = self.node.create_client(ListParameters, f'{target_node_name}/list_parameters')
        self.set_parameter_client_ = self.node.create_client(SetParameters, f'{target_node_name}/set_parameters')
        self.get_parameter_client_ = self.node.create_client(GetParameters, f'{target_node_name}/get_parameters')
        self.get_parameter_types_client_ = self.node.create_client(GetParameterTypes,  f'{target_node_name}/get_parameter_types')



    def wait_for_service(self, timeout_sec: Union[float, None] = None) -> bool:
        """
        Waits for all parameter services to be available.

        :param timeout_sec: Seconds to wait. If ``None``, then wait forever.
        :type timeout_sec: Union[float, None] 
        :return: 
        :rtype: 

        :return: ``True`` if all services are available, False otherwise.
        """

        # TODO(ihasdapie): Wait across all parameter services
        return self.list_parameter_client_.wait_for_service(timeout_sec)


    def list_parameters(self, prefixes: List[str], depth: int, callback: Union[Callable, None] = None) -> Future:
        # TODO: add typing/etc to handle listing all

        """
        Lists all parameters with given prefix,


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
        request.prefixes = prefixes
        request.depth = depth
        future = self.list_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future

    def get_parameters(self, names: List[str], callback: Union[Callable, None] = None) -> Future:
        request = GetParameters.Request()
        request.names = names
        future = self.get_parameter_client_.call_async(request)
        if callback:
            future.add_done_callback(callback)
        return future


    # def set_parameters(self, parameters: Union[List[Parameter], Parameter], callback: Union[Callable, None] = None) -> Future:
        # NOTE: With list of strs instead
    #     request = SetParameters.Request()
    #     request.parameters = parameters
    #     future = self.set_parameter_client_.call_async(request)
    #     if callable:
    #         future.add_done_callback(callback)
    #     return future







        

























