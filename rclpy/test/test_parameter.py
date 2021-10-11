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

from array import array
import unittest

import pytest

from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rclpy.parameter import Parameter
from rclpy.parameter import parameter_value_to_python


class TestParameter(unittest.TestCase):

    def test_create_boolean_parameter(self):
        p = Parameter('myparam', Parameter.Type.BOOL, True)
        self.assertEqual(p.name, 'myparam')
        self.assertEqual(p.value, True)

        p = Parameter('myparam', value=True)
        self.assertEqual(p.name, 'myparam')
        self.assertEqual(p.value, True)

    def test_create_bytes_parameter(self):
        p = Parameter('myparam', Parameter.Type.BYTE_ARRAY, [b'p', b'v', b'a', b'l', b'u', b'e'])
        self.assertEqual(p.name, 'myparam')
        self.assertEqual(p.value, [b'p', b'v', b'a', b'l', b'u', b'e'])

        p = Parameter('myparam', value=[b'p', b'v', b'a', b'l', b'u', b'e'])
        self.assertEqual(p.name, 'myparam')
        self.assertEqual(p.type_, Parameter.Type.BYTE_ARRAY)
        self.assertEqual(p.value, [b'p', b'v', b'a', b'l', b'u', b'e'])

    def test_create_float_parameter(self):
        p = Parameter('myparam', Parameter.Type.DOUBLE, 2.41)
        self.assertEqual(p.name, 'myparam')
        self.assertEqual(p.value, 2.41)

        p = Parameter('myparam', value=2.41)
        self.assertEqual(p.name, 'myparam')
        self.assertEqual(p.type_, Parameter.Type.DOUBLE)
        self.assertEqual(p.value, 2.41)

    def test_create_integer_parameter(self):
        p = Parameter('myparam', Parameter.Type.INTEGER, 42)
        self.assertEqual(p.name, 'myparam')
        self.assertEqual(p.value, 42)

        p = Parameter('myparam', value=42)
        self.assertEqual(p.name, 'myparam')
        self.assertEqual(p.type_, Parameter.Type.INTEGER)
        self.assertEqual(p.value, 42)

    def test_create_string_parameter(self):
        p = Parameter('myparam', Parameter.Type.STRING, 'pvalue')
        self.assertEqual(p.name, 'myparam')
        self.assertEqual(p.value, 'pvalue')

        p = Parameter('myparam', value='pvalue')
        self.assertEqual(p.name, 'myparam')
        self.assertEqual(p.type_, Parameter.Type.STRING)
        self.assertEqual(p.value, 'pvalue')

    def test_create_boolean_array_parameter(self):
        p = Parameter('myparam', Parameter.Type.BOOL_ARRAY, [True, False, True])
        self.assertEqual(p.value, [True, False, True])

        p = Parameter('myparam', value=[True, False, True])
        self.assertEqual(p.type_, Parameter.Type.BOOL_ARRAY)
        self.assertEqual(p.value, [True, False, True])

    def test_create_float_array_parameter(self):
        p = Parameter('myparam', Parameter.Type.DOUBLE_ARRAY, [2.41, 6.28])
        self.assertEqual(p.value, [2.41, 6.28])

        p = Parameter('myparam', value=[2.41, 6.28])
        self.assertEqual(p.type_, Parameter.Type.DOUBLE_ARRAY)
        self.assertEqual(p.value, [2.41, 6.28])

    def test_create_integer_array_parameter(self):
        p = Parameter('myparam', Parameter.Type.INTEGER_ARRAY, [1, 2, 3])
        self.assertEqual(p.value, [1, 2, 3])

        p = Parameter('myparam', value=[1, 2, 3])
        self.assertEqual(p.type_, Parameter.Type.INTEGER_ARRAY)
        self.assertEqual(p.value, [1, 2, 3])

    def test_create_string_array_parameter(self):
        p = Parameter('myparam', Parameter.Type.STRING_ARRAY, ['hello', 'world'])
        self.assertEqual(p.value, ['hello', 'world'])

        p = Parameter('myparam', value=['hello', 'world'])
        self.assertEqual(p.type_, Parameter.Type.STRING_ARRAY)
        self.assertEqual(p.value, ['hello', 'world'])

    def test_create_not_set_parameter(self):
        p = Parameter('myparam', Parameter.Type.NOT_SET)
        self.assertIsNone(p.value)

        p = Parameter('myparam')
        self.assertIsNone(p.value)
        self.assertEqual(p.type_, Parameter.Type.NOT_SET)

        p = Parameter('myparam', value=None)
        self.assertIsNone(p.value)
        self.assertEqual(p.type_, Parameter.Type.NOT_SET)

    def test_value_and_type_must_agree(self):
        with self.assertRaises(ValueError):
            Parameter('myparam', Parameter.Type.NOT_SET, 42)
        with self.assertRaises(ValueError):
            Parameter('myparam', Parameter.Type.BOOL_ARRAY, 42)

    def test_error_on_illegal_value_type(self):
        with self.assertRaises(TypeError):
            Parameter('illegaltype', 'mytype', 'myvalue')

        with self.assertRaises(TypeError):
            Parameter('illegaltype', value={'invalid': 'type'})

    def test_integer_tuple_array(self):
        # list
        int_list = [1, 2, 3]
        self.assertEqual(
            Parameter.Type.INTEGER_ARRAY, Parameter.Type.from_parameter_value(int_list))
        self.assertTrue(Parameter.Type.check(Parameter.Type.INTEGER_ARRAY, int_list))

        # tuple
        int_tuple = (1, 2, 3)
        self.assertEqual(
            Parameter.Type.INTEGER_ARRAY, Parameter.Type.from_parameter_value(int_tuple))
        self.assertTrue(Parameter.Type.check(Parameter.Type.INTEGER_ARRAY, int_tuple))

    def test_integer_array(self):
        int_array = array('i', [1, 2, 3])
        self.assertEqual(
            Parameter.Type.INTEGER_ARRAY, Parameter.Type.from_parameter_value(int_array))
        # test that it doesn't raise
        Parameter.from_parameter_msg(ParameterMsg(
            name='int_array',
            value=ParameterValue(type=7, integer_array_value=[1, 2, 3])
        ))

    def test_double_array(self):
        double_array = array('d', [1.0, 2.0, 3.0])
        self.assertEqual(
            Parameter.Type.DOUBLE_ARRAY, Parameter.Type.from_parameter_value(double_array))
        # test that it doesn't raise
        Parameter.from_parameter_msg(ParameterMsg(
            name='double_array',
            value=ParameterValue(type=8, double_array_value=[1.0, 2.0, 3.0])
        ))

    def test_parameter_value_to_python(self):
        """Test the parameter_value_to_python conversion function."""
        test_cases = [
            (ParameterValue(type=int(ParameterType.PARAMETER_NOT_SET)), None),
            (ParameterValue(type=int(ParameterType.PARAMETER_INTEGER), integer_value=42), 42),
            (ParameterValue(type=int(ParameterType.PARAMETER_DOUBLE), double_value=3.5), 3.5),
            (ParameterValue(type=int(ParameterType.PARAMETER_STRING), string_value='foo'), 'foo'),
            (
                ParameterValue(
                    type=int(ParameterType.PARAMETER_BYTE_ARRAY),
                    byte_array_value=[b'J', b'P']
                ),
                [b'J', b'P']
            ),
            (
                ParameterValue(
                    type=int(ParameterType.PARAMETER_INTEGER_ARRAY),
                    integer_array_value=[1, 2, 3]),
                [1, 2, 3]
            ),
            (
                ParameterValue(
                    type=int(ParameterType.PARAMETER_DOUBLE_ARRAY),
                    double_array_value=[1.0, 2.0, 3.0]),
                [1.0, 2.0, 3.0]
            ),
            (
                ParameterValue(
                    type=int(ParameterType.PARAMETER_STRING_ARRAY),
                    string_array_value=['foo', 'bar']),
                ['foo', 'bar']
            ),
        ]

        for input_value, expected_value in test_cases:
            result_value = parameter_value_to_python(input_value)
            if isinstance(expected_value, list):
                assert len(result_value) == len(expected_value)
                # element-wise comparison for lists
                assert all(x == y for x, y in zip(result_value, expected_value))
            else:
                assert result_value == expected_value

        # Test invalid 'type' member
        parameter_value = ParameterValue(type=42)
        with pytest.raises(RuntimeError):
            parameter_value_to_python(parameter_value)


if __name__ == '__main__':
    unittest.main()
