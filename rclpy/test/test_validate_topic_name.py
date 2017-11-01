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

from rclpy.exceptions import InvalidServiceNameException
from rclpy.exceptions import InvalidTopicNameException
from rclpy.validate_topic_name import validate_topic_name


class TestValidateTopicName(unittest.TestCase):

    def test_validate_topic_name(self):
        tests = [
            'chatter',
            '{node}/chatter',
            '~/chatter',
        ]
        for topic in tests:
            # Will raise if invalid
            validate_topic_name(topic)

    def test_validate_topic_name_failures(self):
        # topic name may not contain '?'
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not contain characters'):
            validate_topic_name('/invalid_topic?')
        # topic name must start with number in any token
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not start with a number'):
            validate_topic_name('invalid/42topic')

    def test_validate_topic_name_failures_service(self):
        # service name may not contain '?'
        with self.assertRaisesRegex(InvalidServiceNameException, 'must not contain characters'):
            validate_topic_name('/invalid_service?', is_service=True)
        # service name must start with number in any token
        with self.assertRaisesRegex(InvalidServiceNameException, 'must not start with a number'):
            validate_topic_name('invalid/42service', is_service=True)


if __name__ == '__main__':
    unittest.main()
