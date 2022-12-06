# Copyright 2022 Sony Group Corporation.
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

import unittest

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import GetParameters
import rclpy
import rclpy.context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_services_default


class TestParameterService(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.test_node = rclpy.create_node(
            'test_parameter_service',
            namespace='/rclpy',
            context=self.context)

        self.get_parameter_client = self.test_node.create_client(
            GetParameters, '/rclpy/test_parameter_service/get_parameters',
            qos_profile=qos_profile_services_default
        )

        self.describe_parameters_client = self.test_node.create_client(
            DescribeParameters, '/rclpy/test_parameter_service/describe_parameters',
            qos_profile=qos_profile_services_default
        )

        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.test_node)

    def tearDown(self):
        self.executor.shutdown()
        self.test_node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_get_uninitialized_parameter(self):
        self.test_node.declare_parameter('uninitialized_parameter', Parameter.Type.STRING)

        # The type in description should be STRING
        request = DescribeParameters.Request()
        request.names = ['uninitialized_parameter']
        future = self.describe_parameters_client.call_async(request)
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert len(results.descriptors) == 1
        assert results.descriptors[0].type == ParameterType.PARAMETER_STRING
        assert results.descriptors[0].name == 'uninitialized_parameter'

        # The value should be empty
        request = GetParameters.Request()
        request.names = ['uninitialized_parameter']
        future = self.get_parameter_client.call_async(request)
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert results.values == []


if __name__ == '__main__':
    unittest.main()
