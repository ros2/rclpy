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
import threading
import time
import traceback
import unittest

from rcl_interfaces.srv import GetParameters
import rclpy
import rclpy.executors
import rclpy.node
from rclpy.utilities import get_rmw_implementation_identifier
from test_msgs.srv import Empty

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

    @classmethod
    def do_test_service_name(cls, test_service_name_list):
        for service_name, ns, cli_args, target_service_name in test_service_name_list:
            node = rclpy.create_node(
                node_name='node_name',
                context=cls.context,
                namespace=ns,
                cli_args=cli_args,
                start_parameter_services=False)
            client = node.create_client(
                srv_type=Empty,
                srv_name=service_name
            )
            assert client.service_name == target_service_name
            client.destroy()
            node.destroy_node()

    @staticmethod
    def _spin_rclpy_node(
        rclpy_node: rclpy.node.Node,
        rclpy_executor: rclpy.executors.SingleThreadedExecutor
    ) -> None:
        try:
            rclpy_executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass
        except Exception as err:
            traceback.print_exc()
            print(rclpy_node.get_name() + ': ' + str(err))
        print(rclpy_node.get_name() + ': rclpy_node exit')
#        rclpy_node.destroy_node()

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
                if result is not None:
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

    def test_get_service_name(self):
        test_service_name_list = [
            # test_service_name, namespace, cli_args for remap, expected service name
            # No namespaces
            ('service', None, None, '/service'),
            ('example/service', None, None, '/example/service'),
            # Using service names with namespaces
            ('service', 'ns', None, '/ns/service'),
            ('example/service', 'ns', None, '/ns/example/service'),
            ('example/service', 'my/ns', None, '/my/ns/example/service'),
            ('example/service', '/my/ns', None, '/my/ns/example/service'),
            # Global service name
            ('/service', 'ns', None, '/service'),
            ('/example/service', 'ns', None, '/example/service')
        ]
        TestClient.do_test_service_name(test_service_name_list)

    def test_get_service_name_after_remapping(self):
        test_service_name_list = [
            ('service', None, ['--ros-args', '--remap', 'service:=new_service'], '/new_service'),
            ('service', 'ns', ['--ros-args', '--remap', 'service:=new_service'],
             '/ns/new_service'),
            ('service', 'ns', ['--ros-args', '--remap', 'service:=example/new_service'],
             '/ns/example/new_service'),
            ('example/service', 'ns', ['--ros-args', '--remap', 'example/service:=new_service'],
             '/ns/new_service')
        ]
        TestClient.do_test_service_name(test_service_name_list)

    def test_sync_call(self):
        def _service(request, response):
            return response
        cli = self.node.create_client(GetParameters, 'get/parameters')
        srv = self.node.create_service(GetParameters, 'get/parameters', _service)
        try:
            self.assertTrue(cli.wait_for_service(timeout_sec=20))
            executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
            executor.add_node(self.node)
            executor_thread = threading.Thread(
                target=TestClient._spin_rclpy_node, args=(self.node, executor))
            executor_thread.start()
            result = cli.call(GetParameters.Request(), 0.5)
            self.assertTrue(result is not None)
            executor.shutdown()
            executor_thread.join()
        finally:
            self.node.destroy_client(cli)
            self.node.destroy_service(srv)

    def test_sync_call_timeout(self):
        def _service(request, response):
            time.sleep(1)
            return response
        cli = self.node.create_client(GetParameters, 'get/parameters')
        srv = self.node.create_service(GetParameters, 'get/parameters', _service)
        try:
            self.assertTrue(cli.wait_for_service(timeout_sec=20))
            executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
            executor.add_node(self.node)
            executor_thread = threading.Thread(
                target=TestClient._spin_rclpy_node, args=(self.node, executor))
            executor_thread.start()
            with self.assertRaises(TimeoutError):
                cli.call(GetParameters.Request(), 0.5)
        finally:
            executor.shutdown()
            executor_thread.join()
            self.node.destroy_client(cli)
            self.node.destroy_service(srv)

    def test_sync_call_context_manager(self):
        def _service(request, response):
            return response
        with self.node.create_client(GetParameters, 'get/parameters') as cli:
            with self.node.create_service(GetParameters, 'get/parameters', _service):
                self.assertTrue(cli.wait_for_service(timeout_sec=20))
                executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
                executor.add_node(self.node)
                executor_thread = threading.Thread(
                    target=TestClient._spin_rclpy_node, args=(self.node, executor))
                executor_thread.start()
                result = cli.call(GetParameters.Request(), 0.5)
                self.assertTrue(result is not None)
                executor.shutdown()
                executor_thread.join()


if __name__ == '__main__':
    unittest.main()
