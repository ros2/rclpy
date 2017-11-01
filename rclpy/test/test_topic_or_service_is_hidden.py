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

from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden


class TestTopicOrServiceIsHidden(unittest.TestCase):

    def test_topic_or_service_is_hidden(self):
        tests = [
            ('/chatter', False),
            ('chatter', False),
            ('/_chatter', True),
            ('_chatter', True),
            ('/more/complex/chatter', False),
            ('/_more/complex/chatter', True),
            ('/more/_complex/chatter', True),
            ('/more/complex/_chatter', True),
            ('/more/complex_/chatter', False),
            ('/more/complex/_/chatter', True),
            ('_/chatter', True),
            ('/chatter_', False),
            ('/more_/complex/chatter', False),
            ('/more/complex_/chatter', False),
            ('/more/complex/chatter_', False),
            ('/_more/_complex/_chatter', True),
            ('', False),
            ('_', True),
        ]
        for topic, expected in tests:
            self.assertEqual(expected, topic_or_service_is_hidden(topic))


if __name__ == '__main__':
    unittest.main()
