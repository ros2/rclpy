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

import platform
import time
import unittest

from rcl_interfaces.srv import GetParameters
import rclpy
import rclpy.executors
from rclpy.utilities import get_rmw_implementation_identifier

# TODO(sloretz) Reduce fudge once wait_for_service uses node graph events
TIME_FUDGE = 0.3


class TestClient(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('TestClient', context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_wait_for_service_5sec(self):
        cli = self.node.create_client(GetParameters, 'get/parameters')
        try:
            start = time.monotonic()
            self.assertFalse(cli.wait_for_service(timeout_sec=5.0))
            end = time.monotonic()
            self.assertGreater(5.0, end - start - TIME_FUDGE)
            self.assertLess(5.0, end - start + TIME_FUDGE)
        finally:
            self.node.destroy_client(cli)

    def test_wait_for_service_nowait(self):
        cli = self.node.create_client(GetParameters, 'get/parameters')
        try:
            start = time.monotonic()
            self.assertFalse(cli.wait_for_service(timeout_sec=0))
            end = time.monotonic()
            self.assertGreater(0, end - start - TIME_FUDGE)
            self.assertLess(0, end - start + TIME_FUDGE)
        finally:
            self.node.destroy_client(cli)

    def test_wait_for_service_exists(self):
        cli = self.node.create_client(GetParameters, 'test_wfs_exists')
        srv = self.node.create_service(GetParameters, 'test_wfs_exists', lambda request: None)
        try:
            start = time.monotonic()
            self.assertTrue(cli.wait_for_service(timeout_sec=1.0))
            end = time.monotonic()
            self.assertGreater(0, end - start - TIME_FUDGE)
            self.assertLess(0, end - start + TIME_FUDGE)
        finally:
            self.node.destroy_client(cli)
            self.node.destroy_service(srv)

    def test_concurrent_calls_to_service(self):
        cli = self.node.create_client(GetParameters, 'get/parameters')
        srv = self.node.create_service(
            GetParameters, 'get/parameters',
            lambda request, response: response)
        try:
            self.assertTrue(cli.wait_for_service(timeout_sec=20))
            future1 = cli.call_async(GetParameters.Request())
            future2 = cli.call_async(GetParameters.Request())
            executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
            rclpy.spin_until_future_complete(self.node, future1, executor=executor)
            rclpy.spin_until_future_complete(self.node, future2, executor=executor)
            self.assertTrue(future1.result() is not None)
            self.assertTrue(future2.result() is not None)
        finally:
            self.node.destroy_client(cli)
            self.node.destroy_service(srv)

    @unittest.skipIf(
        get_rmw_implementation_identifier() == 'rmw_connextdds' and platform.system() == 'Windows',
        reason='Source timestamp not implemented for Connext on Windows')
    def test_service_timestamps(self):
        cli = self.node.create_client(GetParameters, 'get/parameters')
        srv = self.node.create_service(
            GetParameters, 'get/parameters',
            lambda request, response: response)
        try:
            self.assertTrue(cli.wait_for_service(timeout_sec=20))
            cli.call_async(GetParameters.Request())
            for i in range(5):
                with srv.handle:
                    result = srv.handle.service_take_request(srv.srv_type.Request)
                if result != (None, None):
                    request, header = result
                    self.assertTrue(header is not None)
                    self.assertNotEqual(0, header.source_timestamp)
                    return
                else:
                    time.sleep(0.2)
            self.fail('Did not get a request in time')
        finally:
            self.node.destroy_client(cli)
            self.node.destroy_service(srv)

    def test_different_type_raises(self):
        cli = self.node.create_client(GetParameters, 'get/parameters')
        srv = self.node.create_service(
            GetParameters, 'get/parameters',
            lambda request, response: 'different response type')
        try:
            with self.assertRaises(TypeError):
                cli.call('different request type')
            with self.assertRaises(TypeError):
                cli.call_async('different request type')
            self.assertTrue(cli.wait_for_service(timeout_sec=20))
            future = cli.call_async(GetParameters.Request())
            executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
            with self.assertRaises(TypeError):
                rclpy.spin_until_future_complete(self.node, future, executor=executor)
        finally:
            self.node.destroy_client(cli)
            self.node.destroy_service(srv)


if __name__ == '__main__':
    unittest.main()
