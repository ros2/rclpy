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

from rclpy.parameter import Parameter


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
            Parameter('illegaltype', value=('invalid', 'type'))

        with self.assertRaises(TypeError):
            Parameter('illegaltype', value={'invalid': 'type'})


if __name__ == '__main__':
    unittest.main()
