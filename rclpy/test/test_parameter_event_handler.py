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

from typing import Union
import unittest

import pytest

from rcl_interfaces.msg import ParameterEvent
import rclpy.context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from rclpy.parameter_event_handler import ParameterCallbackHandle
from rclpy.parameter_event_handler import ParameterEventCallbackHandle
from rclpy.parameter_event_handler import ParameterEventHandler


from rclpy.qos import qos_profile_parameter_events


class ParameterEventHandlerTester(ParameterEventHandler):

    def test_event(self, parameter_event: ParameterEvent):
        self._callbacks.event_callback(parameter_event)


class CallbackChecker:

    def __init__(self):
        self.received = False

    def callback(self, _: Union[Parameter, ParameterEvent]):
        self.received = True


class CallCounter:

    def __init__(self):
        self.counter = 0
        self.first_callback_call_order = 0
        self.second_callback_call_order = 0

    def first_callback(self, _: Union[Parameter, ParameterEvent]):
        self.counter += 1
        self.first_callback_call_order = self.counter

    def second_callback(self, _: Union[Parameter, ParameterEvent]):
        self.counter += 1
        self.second_callback_call_order = self.counter


class TestParameterEventHandler(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.handler_node = rclpy.create_node(
            'test_parameter_event_handler',
            namespace='/rclpy',
            context=self.context
        )
        self.target_node = rclpy.create_node(
            'test_parameter_event_handler_target',
            namespace='/rclpy',
            allow_undeclared_parameters=True,
            context=self.context
        )
        self.target_node.declare_parameter('int_arr_param', [1, 2, 3])
        self.target_node.declare_parameter('float.param..', 3.14)

        self.parameter_event_handler = ParameterEventHandlerTester(
            self.handler_node,
            qos_profile_parameter_events,
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.handler_node)
        self.executor.add_node(self.target_node)

    def tearDown(self):
        self.executor.shutdown()
        self.handler_node.destroy_node()
        self.target_node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_register_parameter_callback(self):
        self.parameter_event_handler._callbacks.parameter_callbacks.clear()

        parameter_name = 'double_param'
        node_name = self.target_node.get_fully_qualified_name()

        # Callback is not called in this test anyway
        handle = self.parameter_event_handler.add_parameter_callback(
            parameter_name, node_name, lambda: None
        )

        assert isinstance(handle, ParameterCallbackHandle)
        assert {(parameter_name, node_name): [handle]} ==\
            self.parameter_event_handler._callbacks.parameter_callbacks

    def test_get_parameter_from_event(self):
        int_param = Parameter('int_param', Parameter.Type.INTEGER, 1)
        str_param = Parameter('str_param', Parameter.Type.STRING, 'hello world')

        new_params_event = ParameterEvent(
            node=self.target_node.get_fully_qualified_name(),
            new_parameters=[int_param, str_param]
        )

        # Correct parameter name, corrent node name
        assert int_param == ParameterEventHandler.get_parameter_from_event(
            new_params_event, 'int_param', self.target_node.get_fully_qualified_name()
        )
        assert str_param == ParameterEventHandler.get_parameter_from_event(
            new_params_event, 'str_param', self.target_node.get_fully_qualified_name()
        )

        # Correct parameter name, incorrect node name
        assert None is ParameterEventHandler.get_parameter_from_event(
            new_params_event, 'int_param', '/wrong_node_name'
        )
        assert None is ParameterEventHandler.get_parameter_from_event(
            new_params_event, 'str_param', '/wrong_node_name'
        )

        # Incorrect parameter name, correct node name
        assert None is ParameterEventHandler.get_parameter_from_event(
            new_params_event, 'wrong_int_param', self.target_node.get_fully_qualified_name()
        )
        assert None is ParameterEventHandler.get_parameter_from_event(
            new_params_event, 'wrong_str_param', self.target_node.get_fully_qualified_name()
        )

        # Incorrect parameter name, incorrect node name
        assert None is ParameterEventHandler.get_parameter_from_event(
            new_params_event, 'wrong_int_param', '/wrong_node_name'
        )
        assert None is ParameterEventHandler.get_parameter_from_event(
            new_params_event, 'wrong_str_param', '/wrong_node_name'
        )

    def test_get_parameters_from_event(self):
        int_param = Parameter('int_param', Parameter.Type.INTEGER, 1)
        str_param = Parameter('str_param', Parameter.Type.STRING, 'hello world')

        event = ParameterEvent(
            node=self.target_node.get_fully_qualified_name(),
            changed_parameters=[int_param, str_param]
        )

        res = ParameterEventHandler.get_parameters_from_event(event)

        assert {int_param, str_param} == set(res)

    def test_register_parameter_event_callback(self):
        self.parameter_event_handler._callbacks.event_callbacks.clear()

        handle = self.parameter_event_handler.add_parameter_event_callback(
            lambda x: None
        )

        assert isinstance(handle, ParameterEventCallbackHandle)
        assert [handle] == self.parameter_event_handler._callbacks.event_callbacks

    def test_parameter_callback(self):
        callback_checker = CallbackChecker()

        parameter_name = 'int_param'
        node_name = self.target_node.get_fully_qualified_name()

        parameter = Parameter(parameter_name, Parameter.Type.INTEGER, 1)
        parameter_event = ParameterEvent(
            node=node_name,
            changed_parameters=[parameter]
        )

        callback_handle = self.parameter_event_handler.add_parameter_callback(
            parameter_name, node_name, callback_checker.callback
        )

        assert not callback_checker.received
        self.parameter_event_handler.test_event(parameter_event)
        assert callback_checker.received

        self.parameter_event_handler.remove_parameter_callback(callback_handle)
        callback_checker.received = False

        self.parameter_event_handler.test_event(parameter_event)
        assert not callback_checker.received

        with pytest.raises(RuntimeError):
            self.parameter_event_handler.remove_parameter_callback(callback_handle)

    def test_parameter_event_callback(self):
        callback_checker = CallbackChecker()

        parameter_name = 'int_param'
        node_name = self.target_node.get_fully_qualified_name()

        parameter = Parameter(parameter_name, Parameter.Type.INTEGER, 1)
        parameter_event = ParameterEvent(
            node=node_name,
            changed_parameters=[parameter]
        )

        callback_handle = self.parameter_event_handler.add_parameter_event_callback(
            callback_checker.callback
        )

        assert not callback_checker.received
        self.parameter_event_handler.test_event(parameter_event)
        assert callback_checker.received

        self.parameter_event_handler.remove_parameter_event_callback(callback_handle)
        callback_checker.received = False

        self.parameter_event_handler.test_event(parameter_event)
        assert not callback_checker.received

        with pytest.raises(RuntimeError):
            self.parameter_event_handler.remove_parameter_event_callback(callback_handle)

    def test_last_in_first_call_for_parameter_callbacks(self):
        call_counter = CallCounter()

        parameter_name = 'int_param'
        parameter = Parameter(parameter_name, Parameter.Type.INTEGER, 1)

        node_name = self.target_node.get_fully_qualified_name()
        parameter_event = ParameterEvent(
            node=node_name,
            changed_parameters=[parameter]
        )

        self.parameter_event_handler.add_parameter_callback(
            parameter_name, node_name, call_counter.first_callback
        )
        self.parameter_event_handler.add_parameter_callback(
            parameter_name, node_name, call_counter.second_callback
        )

        assert call_counter.first_callback_call_order == 0
        assert call_counter.second_callback_call_order == 0
        self.parameter_event_handler.test_event(parameter_event)

        # Last in first called
        assert call_counter.first_callback_call_order == 2
        assert call_counter.second_callback_call_order == 1

    def test_last_in_first_call_for_parameter_event_callbacks(self):
        call_counter = CallCounter()

        parameter_name = 'int_param'
        parameter = Parameter(parameter_name, Parameter.Type.INTEGER, 1)

        node_name = self.target_node.get_fully_qualified_name()
        parameter_event = ParameterEvent(
            node=node_name,
            changed_parameters=[parameter]
        )

        self.parameter_event_handler.add_parameter_event_callback(
            call_counter.first_callback
        )
        self.parameter_event_handler.add_parameter_event_callback(
            call_counter.second_callback
        )

        assert call_counter.first_callback_call_order == 0
        assert call_counter.second_callback_call_order == 0
        self.parameter_event_handler.test_event(parameter_event)

        # Last in first called
        assert call_counter.first_callback_call_order == 2
        assert call_counter.second_callback_call_order == 1

    def test_resolve_path_empty_path(self):
        assert '/rclpy/test_parameter_event_handler' ==\
               self.parameter_event_handler._resolve_path()

    def test_resolve_path_same_namespace(self):
        assert '/rclpy/test_node' == self.parameter_event_handler._resolve_path('test_node')

    def test_resolve_path_other_namespace(self):
        assert '/test_node' == self.parameter_event_handler._resolve_path('/test_node')


if __name__ == '__main__':
    unittest.main()
