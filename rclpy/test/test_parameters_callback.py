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

import unittest

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.parameter import Parameter


class TestParametersCallback(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown(context=cls.context)

    def setUp(self):
        self.node = rclpy.create_node('parameters_callback_node', context=self.context)

    def tearDown(self):
        self.node.destroy_node()

    def test_set_callback_accepting_all(self):
        callback_called = False

        def callback(parameter_list):
            nonlocal callback_called
            callback_called = True
            return SetParametersResult(successful=True)
        self.node.set_parameters_callback(callback)
        result = self.node.set_parameters_atomically(
            [Parameter('foo', Parameter.Type.STRING, 'Hello')]
        )
        assert callback_called
        assert result.successful
        assert 'Hello' == self.node.get_parameter('foo').value

    def test_set_callback_rejecting_all(self):
        callback_called = False

        def callback(parameter_list):
            nonlocal callback_called
            callback_called = True
            return SetParametersResult(successful=False)
        self.node.set_parameters_callback(callback)
        result = self.node.set_parameters_atomically(
            [Parameter('foo', Parameter.Type.STRING, 'Hello')]
        )
        assert callback_called
        assert not result.successful
        assert Parameter.Type.NOT_SET == self.node.get_parameter('foo').type_

    def test_set_callback_accepting_even_integers(self):
        callback_called = False

        def callback(parameter_list):
            nonlocal callback_called
            callback_called = True
            r = SetParametersResult(successful=True)
            for p in parameter_list:
                if p.type_ != Parameter.Type.INTEGER:
                    r.successful = False
                    r.reason = 'Integer parameters only'
                    return r
                if p.value % 2 != 0:
                    r.successful = False
                    r.reason = 'Integer must be even'
                    return r
            return r
        self.node.set_parameters_callback(callback)
        result = self.node.set_parameters_atomically(
            [Parameter('foo', Parameter.Type.STRING, 'Hello')]
        )
        assert callback_called
        assert not result.successful
        assert Parameter.Type.NOT_SET == self.node.get_parameter('foo').type_
        assert 'Integer parameters only' == result.reason

        callback_called = False
        result = self.node.set_parameters_atomically(
            [Parameter('foo', Parameter.Type.INTEGER, 7)]
        )
        assert callback_called
        assert not result.successful
        assert Parameter.Type.NOT_SET == self.node.get_parameter('foo').type_
        assert 'Integer must be even' == result.reason

        callback_called = False
        result = self.node.set_parameters_atomically([
            Parameter('foo', Parameter.Type.INTEGER, 7),
            Parameter('bar', Parameter.Type.INTEGER, 8)
        ])
        assert callback_called
        assert not result.successful
        assert Parameter.Type.NOT_SET == self.node.get_parameter('foo').type_
        assert Parameter.Type.NOT_SET == self.node.get_parameter('bar').type_
        assert 'Integer must be even' == result.reason

        callback_called = False
        result = self.node.set_parameters_atomically([
            Parameter('foo', Parameter.Type.INTEGER, 6),
            Parameter('bar', Parameter.Type.INTEGER, 8)
        ])
        assert callback_called
        assert result.successful
        assert 6 == self.node.get_parameter('foo').value
        assert 8 == self.node.get_parameter('bar').value


if __name__ == '__main__':
    unittest.main()
