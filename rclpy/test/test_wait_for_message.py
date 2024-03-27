# Copyright 2022 Sony Group Corporation.
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

import threading
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile
from rclpy.wait_for_message import wait_for_message
from test_msgs.msg import BasicTypes

MSG_DATA = 100
TOPIC_NAME = 'wait_for_message_topic'


class TestWaitForMessage(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('publisher_node', context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def _publish_message(self):
        pub = self.node.create_publisher(BasicTypes, TOPIC_NAME, 1)
        msg = BasicTypes()
        msg.int32_value = MSG_DATA
        while True:
            if self.node.count_subscribers(TOPIC_NAME) > 0:
                pub.publish(msg)
                break
            time.sleep(1)
        pub.destroy()

    def test_wait_for_message(self):
        t = threading.Thread(target=self._publish_message)
        t.start()
        ret, msg = wait_for_message(BasicTypes, self.node, TOPIC_NAME, qos_profile=1)
        self.assertTrue(ret)
        self.assertEqual(msg.int32_value, MSG_DATA)
        t.join()

    def test_wait_for_message_qos(self):
        t = threading.Thread(target=self._publish_message)
        t.start()
        ret, msg = wait_for_message(
            BasicTypes, self.node, TOPIC_NAME, qos_profile=QoSProfile(depth=1))
        self.assertTrue(ret)
        self.assertEqual(msg.int32_value, MSG_DATA)
        t.join()

    def test_wait_for_message_timeout(self):
        ret, _ = wait_for_message(BasicTypes, self.node, TOPIC_NAME, time_to_wait=1)
        self.assertFalse(ret)


if __name__ == '__main__':
    unittest.main()
