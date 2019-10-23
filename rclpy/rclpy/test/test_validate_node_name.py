# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from rclpy.exceptions import InvalidNodeNameException
from rclpy.validate_node_name import validate_node_name


class TestValidateNodeName(unittest.TestCase):

    def test_validate_node_name(self):
        tests = [
            'my_node',
        ]
        for topic in tests:
            # Will raise if invalid
            validate_node_name(topic)

    def test_validate_node_name_failures(self):
        # node name must not be empty
        with self.assertRaisesRegex(InvalidNodeNameException, 'must not be empty'):
            validate_node_name('')
        # node name may not contain '.'
        with self.assertRaisesRegex(InvalidNodeNameException, 'must not contain characters'):
            validate_node_name('invalid_node.')
        # node name may not contain '?'
        with self.assertRaisesRegex(InvalidNodeNameException, 'must not contain characters'):
            validate_node_name('invalid_node?')
        # node name must not contain /
        with self.assertRaisesRegex(InvalidNodeNameException, 'must not contain characters'):
            validate_node_name('/invalid_node')


if __name__ == '__main__':
    unittest.main()
