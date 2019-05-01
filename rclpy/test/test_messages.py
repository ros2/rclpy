# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import rclpy

from test_msgs.msg import BasicTypes, Strings


class TestMessages(unittest.TestCase):

    NODE_NAME = 'messages_tester'
    NAMESPACE = 'messages_test'

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node(
            TestMessages.NODE_NAME,
            namespace=TestMessages.NAMESPACE,
            context=cls.context
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_invalid_string_raises(self):
        msg = Strings()
        msg.string_value = 'ñu'
        pub = self.node.create_publisher(Strings, 'chatter')
        with self.assertRaises(UnicodeEncodeError):
            pub.publish(msg)

    def test_different_type_raises(self):
        pub = self.node.create_publisher(BasicTypes, 'chatter')
        with self.assertRaises(TypeError):
            pub.publish('different message type')
