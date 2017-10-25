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

from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.constants import S_TO_NS
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.wait_set import WaitSet
from std_msgs.msg import String


class TestWaitSet(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('TestWaitSet')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_guard_condition_ready(self):
        ws = WaitSet()
        gc_handle, gc_pointer = _rclpy.rclpy_create_guard_condition()
        try:
            ws.add_guard_condition(gc_handle, gc_pointer)
            self.assertFalse(ws.is_ready(gc_pointer))

            ws.wait(0)
            self.assertFalse(ws.is_ready(gc_pointer))

            _rclpy.rclpy_trigger_guard_condition(gc_handle)
            ws.wait(0)
            self.assertTrue(ws.is_ready(gc_pointer))

            ws.wait(0)
            self.assertFalse(ws.is_ready(gc_pointer))
        finally:
            _rclpy.rclpy_destroy_entity(gc_handle)

    def test_timer_ready(self):
        ws = WaitSet()
        timer_handle, timer_pointer = _rclpy.rclpy_create_timer(int(0.1 * S_TO_NS))
        try:
            ws.add_timer(timer_handle, timer_pointer)
            self.assertFalse(ws.is_ready(timer_pointer))

            ws.wait(0)
            self.assertFalse(ws.is_ready(timer_pointer))

            ws.wait(int(0.1 * S_TO_NS))
            self.assertTrue(ws.is_ready(timer_pointer))

            _rclpy.rclpy_call_timer(timer_handle)
            ws.wait(0)
            self.assertFalse(ws.is_ready(timer_pointer))
        finally:
            _rclpy.rclpy_destroy_entity(timer_handle)

    def test_subscriber_ready(self):
        ws = WaitSet()
        sub = self.node.create_subscription(String, 'chatter', lambda msg: print(msg))
        pub = self.node.create_publisher(String, 'chatter')

        try:
            ws.add_subscription(sub.subscription_handle, sub.subscription_pointer)
            self.assertFalse(ws.is_ready(sub.subscription_pointer))

            ws.wait(0)
            self.assertFalse(ws.is_ready(sub.subscription_pointer))

            msg = String()
            msg.data = 'Hello World'
            pub.publish(msg)

            ws.wait(5 * S_TO_NS)
            self.assertTrue(ws.is_ready(sub.subscription_pointer))

            _rclpy.rclpy_take(sub.subscription_handle, sub.msg_type)
            ws.wait(0)
            self.assertFalse(ws.is_ready(sub.subscription_pointer))
        finally:
            self.node.destroy_publisher(pub)
            self.node.destroy_subscription(sub)

    def test_server_ready(self):
        ws = WaitSet()
        cli = self.node.create_client(GetParameters, 'get/parameters')
        srv = self.node.create_service(
            GetParameters, 'get/parameters', lambda req: GetParameters.Response())

        try:
            ws.add_client(cli.client_handle, cli.client_pointer)
            ws.add_service(srv.service_handle, srv.service_pointer)
            self.assertFalse(ws.is_ready(cli.client_pointer))
            self.assertFalse(ws.is_ready(srv.service_pointer))

            ws.wait(0)
            self.assertFalse(ws.is_ready(cli.client_pointer))
            self.assertFalse(ws.is_ready(srv.service_pointer))

            cli.wait_for_service()
            cli.call(GetParameters.Request())

            ws.wait(5 * S_TO_NS)
            # TODO(sloretz) test client after it's wait_for_future() API is sorted out
            self.assertTrue(ws.is_ready(srv.service_pointer))

            _rclpy.rclpy_take_request(srv.service_handle, srv.srv_type.Request)
            ws.wait(0)
            self.assertFalse(ws.is_ready(srv.service_pointer))
        finally:
            self.node.destroy_client(cli)
            self.node.destroy_service(srv)


if __name__ == '__main__':
    unittest.main()
