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

from rclpy.serialization import serialize_message
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

    def test_unicode_string(self):
        msg = Strings()
        msg.string_value = 'ñu'
        pub = self.node.create_publisher(Strings, 'chatter', 1)
        pub.publish(msg)
        self.node.destroy_publisher(pub)

    def test_different_type_raises(self):
        # TODO(bmarchi): When a publisher is destroyed for opensplice,
        # the topic from the previous test is kept around and is cached, so
        # the new publisher can't be created because opensplice checks
        # if the parameters of the created topic are the same as the one
        # that has in memory. It's expected that if any participant
        # is not subscribed/publishing to a topic, this is destroyed.
        # Revert the topic name to 'chatter' once proper topic destruction
        # for opensplice is possible.
        pub = self.node.create_publisher(
            BasicTypes, 'chatter_different_message_type', 1)
        with self.assertRaises(TypeError):
            pub.publish('different message type')
        self.node.destroy_publisher(pub)

    def test_serialized_publish(self):
        msg = Strings()
        msg.string_value = 'ñu'
        pub = self.node.create_publisher(Strings, 'chatter', 1)
        pub.publish(serialize_message(msg))
        self.node.destroy_publisher(pub)
