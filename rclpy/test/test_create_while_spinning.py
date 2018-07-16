# Copyright 2018 Open Source Robotics Foundation, Inc.
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
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


# Arbitrary sleep time
TIMEOUT = 0.5


class TestCreateWhileSpinning(unittest.TestCase):
    """
    Test that the executor wakes after an entity is created.

    This is a regression test for ros2/rclpy#188.
    """

    def setUp(self):
        print("setup")
        rclpy.init()
        self.node = rclpy.create_node('TestCreateWhileSpinning', namespace='/rclpy')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.exec_thread = threading.Thread(target=self.executor.spin)
        self.exec_thread.start()
        time.sleep(TIMEOUT)

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()
        self.exec_thread.join()

    def test_publish_subscribe(self):
        # Assert that a published message is received
        evt = threading.Event()
        sub = self.node.create_subscription(String, 'foo', lambda msg: evt.set())
        pub = self.node.create_publisher(String, 'foo')
        msg = String()
        msg.data = 'hello world'
        pub.publish(msg)
        assert evt.wait(TIMEOUT)
        self.node.destroy_subscription(sub)
        self.node.destroy_publisher(pub)


if __name__ == '__main__':
    unittest.main()
