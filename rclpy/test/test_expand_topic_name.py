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

from rclpy.expand_topic_name import expand_topic_name


class TestExpandTopicName(unittest.TestCase):

    def test_expand_topic_name(self):
        tests = {
            # 'expected output': ['input topic', 'node', '/ns']
            '/ns/chatter': ['chatter', 'node_name', '/ns'],
            '/chatter': ['chatter', 'node_name', '/'],
            '/ns/node_name/chatter': ['~/chatter', 'node_name', '/ns'],
            '/node_name/chatter': ['~/chatter', 'node_name', '/'],
        }
        for expected_topic, input_tuple in tests.items():
            topic, node, namespace = input_tuple
            expanded_topic = expand_topic_name(topic, node, namespace)
            self.assertEqual(expanded_topic, expected_topic)

    def test_expand_topic_name_invalid_node_name(self):
        # node name may not contain '?'
        with self.assertRaisesRegex(ValueError, 'node name'):
            expand_topic_name('topic', 'invalid_node_name?', '/ns')

    def test_expand_topic_name_invalid_namespace_empty(self):
        # namespace may not be empty
        with self.assertRaisesRegex(ValueError, 'namespace'):
            expand_topic_name('topic', 'node_name', '')

    def test_expand_topic_name_invalid_namespace_relative(self):
        # namespace may not be relative
        with self.assertRaisesRegex(ValueError, 'namespace'):
            expand_topic_name('topic', 'node_name', 'ns')

    def test_expand_topic_name_invalid_topic(self):
        # topic may not contain '?'
        with self.assertRaisesRegex(ValueError, 'topic name'):
            expand_topic_name('invalid/topic?', 'node_name', '/ns')


if __name__ == '__main__':
    unittest.main()
