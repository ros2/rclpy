# Copyright 2022 Open Source Robotics Foundation, Inc.
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
from typing import List
import unittest

import rclpy
import rclpy.executors
from rclpy.qos import qos_profile_system_default
from rclpy.service_introspection import ServiceIntrospectionState
from service_msgs.msg import ServiceEventInfo
from test_msgs.srv import BasicTypes


class TestServiceEvents(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

    def setUp(self):
        self.node = rclpy.create_node(
            'TestServiceIntrospection', context=self.context)
        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.srv = self.node.create_service(BasicTypes, 'test_service', self.srv_callback)
        self.cli = self.node.create_client(BasicTypes, 'test_service')
        self.sub = self.node.create_subscription(BasicTypes.Event, 'test_service/_service_event',
                                                 self.sub_callback, 10)
        self.event_messages: List[BasicTypes.Event] = []

    def tearDown(self):
        self.node.destroy_node()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown(context=cls.context)

    def sub_callback(self, msg):
        self.event_messages.append(msg)

    def srv_callback(self, req, resp):
        resp.bool_value = not req.bool_value
        resp.int64_value = req.int64_value
        return resp

    def test_service_introspection_metadata(self):
        req = BasicTypes.Request()
        req.bool_value = False
        req.int64_value = 12345

        self.cli.configure_introspection(self.node.get_clock(),
                                         qos_profile_system_default,
                                         ServiceIntrospectionState.METADATA)
        self.srv.configure_introspection(self.node.get_clock(),
                                         qos_profile_system_default,
                                         ServiceIntrospectionState.METADATA)

        future = self.cli.call_async(req)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        self.assertTrue(future.done())

        start = time.monotonic()
        end = start + 5.0
        while len(self.event_messages) < 4:
            self.executor.spin_once(timeout_sec=0.1)

            now = time.monotonic()
            self.assertTrue(now < end)

        self.assertEqual(len(self.event_messages), 4)

        result_dict = {}
        for msg in self.event_messages:
            result_dict[msg.info.event_type] = msg
        self.assertEqual(
            set(result_dict.keys()),
            {ServiceEventInfo.REQUEST_SENT, ServiceEventInfo.REQUEST_RECEIVED,
             ServiceEventInfo.RESPONSE_SENT, ServiceEventInfo.RESPONSE_RECEIVED})

        self.assertEqual(len(result_dict[ServiceEventInfo.REQUEST_SENT].request), 0)
        self.assertEqual(len(result_dict[ServiceEventInfo.REQUEST_SENT].response), 0)

        self.assertEqual(len(result_dict[ServiceEventInfo.RESPONSE_SENT].request), 0)
        self.assertEqual(len(result_dict[ServiceEventInfo.RESPONSE_SENT].response), 0)

        self.assertEqual(len(result_dict[ServiceEventInfo.REQUEST_RECEIVED].request), 0)
        self.assertEqual(len(result_dict[ServiceEventInfo.REQUEST_RECEIVED].response), 0)

        self.assertEqual(len(result_dict[ServiceEventInfo.RESPONSE_RECEIVED].request), 0)
        self.assertEqual(len(result_dict[ServiceEventInfo.RESPONSE_RECEIVED].response), 0)

    def test_service_introspection_contents(self):
        req = BasicTypes.Request()
        req.bool_value = False
        req.int64_value = 12345

        self.cli.configure_introspection(self.node.get_clock(),
                                         qos_profile_system_default,
                                         ServiceIntrospectionState.CONTENTS)
        self.srv.configure_introspection(self.node.get_clock(),
                                         qos_profile_system_default,
                                         ServiceIntrospectionState.CONTENTS)

        future = self.cli.call_async(req)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        self.assertTrue(future.done())

        start = time.monotonic()
        end = start + 5.0
        while len(self.event_messages) < 4:
            self.executor.spin_once(timeout_sec=0.1)

            now = time.monotonic()
            self.assertTrue(now < end)

        self.assertEqual(len(self.event_messages), 4)

        result_dict = {}
        for msg in self.event_messages:
            result_dict[msg.info.event_type] = msg
        self.assertEqual(
            set(result_dict.keys()),
            {ServiceEventInfo.REQUEST_SENT, ServiceEventInfo.REQUEST_RECEIVED,
             ServiceEventInfo.RESPONSE_SENT, ServiceEventInfo.RESPONSE_RECEIVED})

        self.assertEqual(len(result_dict[ServiceEventInfo.REQUEST_SENT].request), 1)
        self.assertEqual(result_dict[ServiceEventInfo.REQUEST_SENT].request[0].bool_value, False)
        self.assertEqual(result_dict[ServiceEventInfo.REQUEST_SENT].request[0].int64_value, 12345)
        self.assertEqual(len(result_dict[ServiceEventInfo.REQUEST_SENT].response), 0)

        self.assertEqual(len(result_dict[ServiceEventInfo.RESPONSE_SENT].request), 0)
        self.assertEqual(len(result_dict[ServiceEventInfo.RESPONSE_SENT].response), 1)
        self.assertEqual(result_dict[ServiceEventInfo.RESPONSE_SENT].response[0].bool_value, True)
        self.assertEqual(
            result_dict[ServiceEventInfo.RESPONSE_SENT].response[0].int64_value,
            12345)

        self.assertEqual(len(result_dict[ServiceEventInfo.REQUEST_RECEIVED].request), 1)
        self.assertEqual(
            result_dict[ServiceEventInfo.REQUEST_RECEIVED].request[0].bool_value,
            False)
        self.assertEqual(
            result_dict[ServiceEventInfo.REQUEST_RECEIVED].request[0].int64_value,
            12345)
        self.assertEqual(len(result_dict[ServiceEventInfo.REQUEST_RECEIVED].response), 0)

        self.assertEqual(len(result_dict[ServiceEventInfo.RESPONSE_RECEIVED].request), 0)
        self.assertEqual(len(result_dict[ServiceEventInfo.RESPONSE_RECEIVED].response), 1)
        self.assertEqual(
            result_dict[ServiceEventInfo.RESPONSE_RECEIVED].response[0].bool_value,
            True)
        self.assertEqual(
            result_dict[ServiceEventInfo.RESPONSE_RECEIVED].response[0].int64_value,
            12345)


if __name__ == '__main__':
    unittest.main()
