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

import unittest

from rclpy.type_hash import TypeHash

# From std_msgs/msg/String.json
STD_MSGS_STRING_TYPE_HASH_DICT = {
    'version': 1,
    'value': b'\xdf\x66\x8c\x74\x04\x82\xbb\xd4\x8f\xb3\x9d\x76\xa7\x0d\xfd\x4b'
             b'\xd5\x9d\xb1\x28\x80\x21\x74\x35\x03\x25\x9e\x94\x8f\x6b\x1a\x18',
}
STD_MSGS_STRING_TYPE_HASH_STR = 'RIHS01_' \
                                'df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18'


class TestTypeHash(unittest.TestCase):

    def test_dict_constructor(self):
        type_hash = TypeHash(**STD_MSGS_STRING_TYPE_HASH_DICT)
        self.assertTrue(hasattr(type_hash, '__slots__'))
        self.assertEqual(STD_MSGS_STRING_TYPE_HASH_DICT['version'], type_hash.version)
        self.assertEqual(STD_MSGS_STRING_TYPE_HASH_DICT['value'], type_hash.value)

    def test_print_valid(self):
        actual_str = str(TypeHash(**STD_MSGS_STRING_TYPE_HASH_DICT))
        expected_str = STD_MSGS_STRING_TYPE_HASH_STR
        self.assertEqual(expected_str, actual_str)

    def test_print_invalid(self):
        actual_str = str(TypeHash())
        expected_str = 'INVALID'
        self.assertEqual(expected_str, actual_str)

    def test_equals(self):
        self.assertEqual(TypeHash(), TypeHash())
        self.assertNotEqual(TypeHash(version=5), TypeHash())
