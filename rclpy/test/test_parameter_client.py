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

import os
from tempfile import NamedTemporaryFile
import unittest

import rcl_interfaces.msg
from rcl_interfaces.msg import ParameterType
import rcl_interfaces.srv
import rclpy
import rclpy.context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


class TestParameterClient(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.client_node = rclpy.create_node(
            'test_parameter_client',
            namespace='/rclpy',
            context=self.context)
        self.target_node = rclpy.create_node(
            'test_parameter_client_target',
            namespace='/rclpy',
            allow_undeclared_parameters=True,
            context=self.context)
        self.target_node.declare_parameter('int_arr_param', [1, 2, 3])
        self.target_node.declare_parameter('float.param..', 3.14)

        self.client = AsyncParameterClient(self.client_node, 'test_parameter_client_target')
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.client_node)
        self.executor.add_node(self.target_node)

    def tearDown(self):
        self.executor.shutdown()
        self.client_node.destroy_node()
        self.target_node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_set_parameter(self):
        future = self.client.set_parameters([
            Parameter('int_param', Parameter.Type.INTEGER, 88).to_parameter_msg(),
            Parameter('string.param', Parameter.Type.STRING, 'hello world'),
        ])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert len(results.results) == 2
        res = [i.successful for i in results.results]
        assert all(res)

    def test_get_parameter(self):
        future = self.client.get_parameters(['int_arr_param', 'float.param..'])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert len(results.values) == 2
        assert list(results.values[0].integer_array_value) == [1, 2, 3]
        assert results.values[1].double_value == 3.14

    def test_list_parameters(self):
        future = self.client.list_parameters()
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert 'int_arr_param' in results.result.names
        assert 'float.param..' in results.result.names

    def test_describe_parameters(self):
        future = self.client.describe_parameters(['int_arr_param'])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert len(results.descriptors) == 1
        assert results.descriptors[0].type == ParameterType.PARAMETER_INTEGER_ARRAY
        assert results.descriptors[0].name == 'int_arr_param'

    def test_get_paramter_types(self):
        future = self.client.get_parameter_types(['int_arr_param'])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert len(results.types) == 1
        assert results.types[0] == ParameterType.PARAMETER_INTEGER_ARRAY

    def test_set_parameters_atomically(self):
        future = self.client.set_parameters_atomically([
            Parameter('int_param', Parameter.Type.INTEGER, 888),
            Parameter('string.param', Parameter.Type.STRING, 'Hello World').to_parameter_msg(),
        ])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert results.result.successful

    def test_delete_parameters(self):
        self.target_node.declare_parameter('delete_param', 10)
        descriptor = rcl_interfaces.msg.ParameterDescriptor(dynamic_typing=True)
        self.target_node.declare_parameter('delete_param_dynamic', 10, descriptor=descriptor)

        future = self.client.delete_parameters(['delete_param'])
        self.executor.spin_until_future_complete(future)
        result = future.result()
        assert result is not None
        assert len(result.results) == 1
        assert not result.results[0].successful
        assert result.results[0].reason == 'Static parameter cannot be undeclared'

        future = self.client.delete_parameters(['delete_param_dynamic'])
        self.executor.spin_until_future_complete(future)
        result = future.result()
        assert result is not None
        assert len(result.results) == 1
        assert result.results[0].successful

    def test_load_parameter_file(self):
        yaml_string = """/param_test_target:
            ros__parameters:
                param_1: 1
                param_str: "string"
            """
        try:
            with NamedTemporaryFile(mode='w', delete=False) as f:
                f.write(yaml_string)
                f.flush()
                f.close()
                future = self.client.load_parameter_file(f.name)
            self.executor.spin_until_future_complete(future)
            result = future.result()
            assert result is not None
            assert len(result.results) == 2
            assert all([i.successful for i in result.results])
        finally:
            if os.path.exists(f.name):
                os.unlink(f.name)

    def test_load_parameter_file_atomically(self):
        yaml_string = """/param_test_target:
            ros__parameters:
                param_1: 1
                param_str: "string"
            """
        try:
            with NamedTemporaryFile(mode='w', delete=False) as f:
                f.write(yaml_string)
                f.flush()
                f.close()
                future = self.client.load_parameter_file_atomically(f.name)
            self.executor.spin_until_future_complete(future)
            result = future.result()
            assert result is not None
            assert result.result.successful
        finally:
            if os.path.exists(f.name):
                os.unlink(f.name)

    def test_get_uninitialized_parameter(self):
        self.target_node.declare_parameter('uninitialized_parameter', Parameter.Type.STRING)

        # The type in description should be STRING
        future = self.client.describe_parameters(['uninitialized_parameter'])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert len(results.descriptors) == 1
        assert results.descriptors[0].type == ParameterType.PARAMETER_STRING
        assert results.descriptors[0].name == 'uninitialized_parameter'

        # The value should be empty
        future = self.client.get_parameters(['uninitialized_parameter'])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert results.values == []

        self.target_node.undeclare_parameter('uninitialized_parameter')

    def test_on_parameter_event_new(self):
        def on_new_parameter_event(msg):
            assert msg.node == '/rclpy/test_parameter_client_target'
            assert len(msg.new_parameters) == 1
            assert msg.new_parameters[0].name == 'int_param'
            assert msg.new_parameters[0].value.integer_value == 88
            assert len(msg.changed_parameters) == 0
            assert len(msg.deleted_parameters) == 0

        param_event_sub = self.client.on_parameter_event(on_new_parameter_event)

        future = self.client.set_parameters([
            Parameter('int_param', Parameter.Type.INTEGER, 88).to_parameter_msg(),
        ])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert len(results.results) == 1
        res = [i.successful for i in results.results]
        assert all(res)

        param_event_sub.destroy()

    def test_on_parameter_event_changed(self):
        future = self.client.set_parameters([
            Parameter('int_param', Parameter.Type.INTEGER, 88).to_parameter_msg(),
        ])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert len(results.results) == 1
        res = [i.successful for i in results.results]
        assert all(res)

        def on_changed_parameter_event(msg):
            assert msg.node == '/rclpy/test_parameter_client_target'
            assert len(msg.new_parameters) == 0
            assert len(msg.changed_parameters) == 1
            assert msg.changed_parameters[0].name == 'int_param'
            assert msg.changed_parameters[0].value.integer_value == 99
            assert len(msg.deleted_parameters) == 0

        param_event_sub = self.client.on_parameter_event(on_changed_parameter_event)

        future = self.client.set_parameters([
            Parameter('int_param', Parameter.Type.INTEGER, 99).to_parameter_msg(),
        ])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert len(results.results) == 1
        res = [i.successful for i in results.results]
        assert all(res)

        param_event_sub.destroy()

    def test_on_parameter_event_deleted(self):
        future = self.client.set_parameters([
            Parameter('int_param', Parameter.Type.INTEGER, 88).to_parameter_msg(),
        ])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert len(results.results) == 1
        res = [i.successful for i in results.results]
        assert all(res)

        def on_deleted_parameter_event(msg):
            assert msg.node == '/rclpy/test_parameter_client_target'
            assert len(msg.new_parameters) == 0
            assert len(msg.changed_parameters) == 0
            assert len(msg.deleted_parameters) == 1
            assert msg.deleted_parameters[0].name == 'int_param'

        param_event_sub = self.client.on_parameter_event(on_deleted_parameter_event)

        future = self.client.delete_parameters(['int_param'])
        self.executor.spin_until_future_complete(future)
        results = future.result()
        assert results is not None
        assert len(results.results) == 1
        res = [i.successful for i in results.results]
        assert all(res)

        param_event_sub.destroy()
