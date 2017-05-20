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

import rclpy

from rclpy.exceptions import InvalidServiceNameException
from rclpy.exceptions import InvalidTopicNameException

from std_msgs.msg import String
from std_srvs.srv import Empty


class TestNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('my_node', namespace='/my_ns')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_accessors(self):
        self.assertIsNotNone(self.node.handle)
        with self.assertRaises(AttributeError):
            self.node.handle = 'garbage'
        self.assertEqual(self.node.get_name(), 'my_node')
        self.assertEqual(self.node.get_namespace(), '/my_ns')

    def test_create_publisher(self):
        self.node.create_publisher(String, 'chatter')
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not contain characters'):
            self.node.create_publisher(String, 'chatter?')
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not start with a number'):
            self.node.create_publisher(String, '/chatter/42_is_the_answer')
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_publisher(String, 'chatter/{bad_sub}')

    def test_create_subscription(self):
        self.node.create_subscription(String, 'chatter', lambda msg: print(msg))
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not contain characters'):
            self.node.create_subscription(String, 'chatter?', lambda msg: print(msg))
        with self.assertRaisesRegex(InvalidTopicNameException, 'must not start with a number'):
            self.node.create_subscription(String, '/chatter/42ish', lambda msg: print(msg))
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_subscription(String, 'foo/{bad_sub}', lambda msg: print(msg))

    def test_create_client(self):
        self.node.create_client(Empty, 'side/effect')
        with self.assertRaisesRegex(InvalidServiceNameException, 'must not contain characters'):
            self.node.create_client(Empty, 'side/effect?')
        with self.assertRaisesRegex(InvalidServiceNameException, 'must not start with a number'):
            self.node.create_client(Empty, '/side/42effect')
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_client(Empty, 'foo/{bad_sub}')

    def test_create_service(self):
        self.node.create_service(Empty, 'side/effect', lambda req: None)
        with self.assertRaisesRegex(InvalidServiceNameException, 'must not contain characters'):
            self.node.create_service(Empty, 'side/effect?', lambda req: None)
        with self.assertRaisesRegex(InvalidServiceNameException, 'must not start with a number'):
            self.node.create_service(Empty, '/side/42effect', lambda req: None)
        with self.assertRaisesRegex(ValueError, 'unknown substitution'):
            self.node.create_service(Empty, 'foo/{bad_sub}', lambda req: None)


if __name__ == '__main__':
    unittest.main()
