# Copyright 2023 Open Source Robotics Foundation, Inc.
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
import rclpy.context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_services_default
from test_msgs.msg import BasicTypes
from type_description_interfaces.srv import GetTypeDescription


class TestTypeDescriptionService(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)

        self.test_node = rclpy.create_node(
            'test_type_description_service',
            namespace='/rclpy',
            context=self.context)
        self.test_topic = '/rclpy/basic_types'
        self.test_pub = self.test_node.create_publisher(
            BasicTypes, self.test_topic, 10)

        self.get_type_description_client = self.test_node.create_client(
            GetTypeDescription, '/rclpy/test_parameter_service/get_type_description',
            qos_profile=qos_profile_services_default)

        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.test_node)

    def tearDown(self):
        self.executor.shutdown()
        self.test_node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_get_type_description(self):
        pub_infos = self.test_node.get_publishers_info_by_topic(self.test_topic)
        assert len(pub_infos)
        type_hash = pub_infos[0].topic_type_hash

        request = GetTypeDescription.Request(
            type_name='test_msgs/msg/BasicTypes',
            type_hash=type_hash,
            include_type_sources=True)
        future = self.get_type_description_client.call_async(request)
        self.executor.spin_until_future_complete(future)
        response = future.result()
        assert response is not None
        assert response.successful
        assert response.type_description.type_description.type_name == 'test_msgs/msg/BasicTypes'
        assert len(response.type_sources)
