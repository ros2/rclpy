# Copyright 2023 Sony Group Corporation.
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

import unittest

from rcl_interfaces.msg import LoggerLevel
from rcl_interfaces.srv import GetLoggerLevels
from rcl_interfaces.srv import SetLoggerLevels
import rclpy
import rclpy.context
from rclpy.executors import SingleThreadedExecutor


class TestLoggingService(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.test_node_with_logger_service = rclpy.create_node(
            'test_node_with_logger_service_enabled',
            namespace='/rclpy',
            context=self.context,
            enable_logger_service=True)

        self.test_node = rclpy.create_node(
            'test_logger_service',
            namespace='/rclpy',
            context=self.context)

        self.executor1 = SingleThreadedExecutor(context=self.context)
        self.executor1.add_node(self.test_node_with_logger_service)

        self.executor2 = SingleThreadedExecutor(context=self.context)
        self.executor2.add_node(self.test_node)

        self.thread = threading.Thread(target=self.executor1.spin, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.executor1.shutdown()
        self.thread.join()
        self.executor2.shutdown()
        self.test_node.destroy_node()
        self.test_node_with_logger_service.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_connect_get_logging_service(self):
        client = self.test_node.create_client(
            GetLoggerLevels,
            '/rclpy/test_node_with_logger_service_enabled/get_logger_levels')
        try:
            self.assertTrue(client.wait_for_service(2))
        finally:
            self.test_node.destroy_client(client)

    def test_connect_set_logging_service(self):
        client = self.test_node.create_client(
            SetLoggerLevels,
            '/rclpy/test_node_with_logger_service_enabled/set_logger_levels'
        )
        try:
            self.assertTrue(client.wait_for_service(2))
        finally:
            self.test_node.destroy_client(client)

    def test_set_and_get_one_logging_level(self):
        test_log_name = 'rcl'
        test_log_level = 10

        # Set debug level
        set_client = self.test_node.create_client(
            SetLoggerLevels,
            '/rclpy/test_node_with_logger_service_enabled/set_logger_levels')
        self.assertTrue(set_client.wait_for_service(2))
        request = SetLoggerLevels.Request()
        set_level = LoggerLevel()
        set_level.name = test_log_name
        set_level.level = test_log_level
        request.levels.append(set_level)
        future = set_client.call_async(request)
        self.executor2.spin_until_future_complete(future, 10)
        self.assertTrue(future.done())
        response = future.result()
        self.assertEqual(len(response.results), 1)
        self.assertTrue(response.results[0].successful)
        self.assertEqual(response.results[0].reason, '')  # reason should be empty if successful
        self.test_node.destroy_client(set_client)

        # Get set level
        get_client = self.test_node.create_client(
            GetLoggerLevels,
            '/rclpy/test_node_with_logger_service_enabled/get_logger_levels')
        self.assertTrue(get_client.wait_for_service(2))
        request = GetLoggerLevels.Request()
        request.names = [test_log_name]
        future = get_client.call_async(request)
        self.executor2.spin_until_future_complete(future, 10)
        self.assertTrue(future.done())
        response = future.result()
        self.assertEqual(len(response.levels), 1)
        self.assertEqual(response.levels[0].name, test_log_name)
        self.assertEqual(response.levels[0].level, test_log_level)
        self.test_node.destroy_client(get_client)

    def test_set_and_get_multi_logging_level(self):
        test_log_name1 = 'rcl'
        test_log_level1 = 20

        test_log_name2 = 'rclpy'
        test_log_level2 = 30

        test_log_name3 = 'test_node_with_logger_service_enabled'
        test_log_level3 = 40

        # Set multi log levels
        request = SetLoggerLevels.Request()
        set_level = LoggerLevel()
        set_level.name = test_log_name1
        set_level.level = test_log_level1
        request.levels.append(set_level)
        set_level = LoggerLevel()
        set_level.name = test_log_name2
        set_level.level = test_log_level2
        request.levels.append(set_level)
        set_level = LoggerLevel()
        set_level.name = test_log_name3
        set_level.level = test_log_level3
        request.levels.append(set_level)

        set_client = self.test_node.create_client(
            SetLoggerLevels,
            '/rclpy/test_node_with_logger_service_enabled/set_logger_levels')
        self.assertTrue(set_client.wait_for_service(2))

        future = set_client.call_async(request)
        self.executor2.spin_until_future_complete(future, 10)
        self.assertTrue(future.done())
        response = future.result()
        self.assertEqual(len(response.results), 3)
        for result in response.results:
            self.assertTrue(result.successful)
            self.assertEqual(result.reason, str())
        self.test_node.destroy_client(set_client)

        # Get multi log levels
        get_client = self.test_node.create_client(
            GetLoggerLevels,
            '/rclpy/test_node_with_logger_service_enabled/get_logger_levels')
        self.assertTrue(get_client.wait_for_service(2))
        request = GetLoggerLevels.Request()
        request.names = [test_log_name1, test_log_name2, test_log_name3]
        future = get_client.call_async(request)
        self.executor2.spin_until_future_complete(future, 10)
        self.assertTrue(future.done())
        response = future.result()
        self.assertEqual(len(response.levels), 3)
        self.assertEqual(response.levels[0].name, test_log_name1)
        self.assertEqual(response.levels[0].level, test_log_level1)
        self.assertEqual(response.levels[1].name, test_log_name2)
        self.assertEqual(response.levels[1].level, test_log_level2)
        self.assertEqual(response.levels[2].name, test_log_name3)
        self.assertEqual(response.levels[2].level, test_log_level3)
        self.test_node.destroy_client(get_client)

    def test_set_logging_level_with_invalid_param(self):
        set_client = self.test_node.create_client(
            SetLoggerLevels,
            '/rclpy/test_node_with_logger_service_enabled/set_logger_levels')
        self.assertTrue(set_client.wait_for_service(2))

        request = SetLoggerLevels.Request()
        set_level = LoggerLevel()
        set_level.name = 'test_node_with_logger_service_enabled'
        set_level.level = 22
        request.levels.append(set_level)
        set_level = LoggerLevel()
        set_level.name = 'rcl'
        set_level.level = 12
        request.levels.append(set_level)
        future = set_client.call_async(request)
        self.executor2.spin_until_future_complete(future, 10)
        self.assertTrue(future.done())
        response = future.result()
        self.assertEqual(len(response.results), 2)
        self.assertFalse(response.results[0].successful)
        self.assertEqual(response.results[0].reason, 'Failed reason: Invalid logger level.')
        self.assertFalse(response.results[1].successful)
        self.assertEqual(response.results[1].reason, 'Failed reason: Invalid logger level.')
        self.test_node.destroy_client(set_client)

    def test_set_logging_level_with_partial_invalid_param(self):
        set_client = self.test_node.create_client(
            SetLoggerLevels,
            '/rclpy/test_node_with_logger_service_enabled/set_logger_levels')
        self.assertTrue(set_client.wait_for_service(2))

        request = SetLoggerLevels.Request()
        set_level = LoggerLevel()
        set_level.name = 'rcl'
        set_level.level = 10
        request.levels.append(set_level)
        set_level = LoggerLevel()
        set_level.name = 'rclpy'
        set_level.level = 22  # Invalid logger level
        request.levels.append(set_level)
        set_level = LoggerLevel()
        set_level.name = 'test_node_with_logger_service_enabled'
        set_level.level = 30
        request.levels.append(set_level)
        future = set_client.call_async(request)
        self.executor2.spin_until_future_complete(future, 10)
        self.assertTrue(future.done())
        response = future.result()
        self.assertEqual(len(response.results), 3)
        self.assertTrue(response.results[0].successful)
        self.assertEqual(response.results[0].reason, '')
        self.assertFalse(response.results[1].successful)
        self.assertEqual(response.results[1].reason, 'Failed reason: Invalid logger level.')
        self.assertTrue(response.results[2].successful)
        self.assertEqual(response.results[2].reason, '')
        self.test_node.destroy_client(set_client)


if __name__ == '__main__':
    unittest.main()
