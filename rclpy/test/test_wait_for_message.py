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

import time
import threading
import unittest
import rclpy
from rclpy.wait_for_message import wait_for_message
from test_msgs.msg import BasicTypes

MSG_DATA = 100
TOPIC_NAME = 'wait_for_message_topic'


def publish_message():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('publisher_node', context=context)
    pub = node.create_publisher(BasicTypes, TOPIC_NAME, 1)
    msg = BasicTypes()
    msg.int32_value = MSG_DATA
    time.sleep(1)
    pub.publish(msg)
    pub.destroy()
    node.destroy_node()
    rclpy.shutdown(context=context)


class TestWaitForMessage(unittest.TestCase):

    def test_wait_for_message(self):
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node('wait_for_message_node', context=context)
        t = threading.Thread(target=publish_message)
        t.start()
        ret, msg = wait_for_message(BasicTypes, node, TOPIC_NAME)
        self.assertTrue(ret)
        self.assertEqual(msg.int32_value, MSG_DATA)
        t.join()
        node.destroy_node()
        rclpy.shutdown(context=context)


class TestWaitForMessageTimeout(unittest.TestCase):

    def test_wait_for_message_timeout(self):
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node('wait_for_message_node', context=context)
        ret, _ = wait_for_message(BasicTypes, node, TOPIC_NAME, 1)
        self.assertFalse(ret)
        node.destroy_node()
        rclpy.shutdown(context=context)


if __name__ == '__main__':
    unittest.main()
