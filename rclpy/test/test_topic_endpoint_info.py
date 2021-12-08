# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.topic_endpoint_info import TopicEndpointInfo, TopicEndpointTypeEnum


class TestQosProfile(unittest.TestCase):

    def test_node_name_only_constructor(self):
        test_node_name = 'test_string'

        info_for_ref = TopicEndpointInfo()
        info_for_ref.node_name = test_node_name

        info_from_ctor = TopicEndpointInfo(node_name=test_node_name)

        self.assertEqual(info_for_ref, info_from_ctor)
        self.assertEqual(test_node_name, info_from_ctor.node_name)

    def test_node_namespace_only_constructor(self):
        test_node_namespace = 'test_string'

        info_for_ref = TopicEndpointInfo()
        info_for_ref.node_namespace = test_node_namespace

        info_from_ctor = TopicEndpointInfo(node_namespace=test_node_namespace)

        self.assertEqual(info_for_ref, info_from_ctor)
        self.assertEqual(test_node_namespace, info_from_ctor.node_namespace)

    def test_topic_type_only_constructor(self):
        test_topic_type = 'test_string'

        info_for_ref = TopicEndpointInfo()
        info_for_ref.topic_type = test_topic_type

        info_from_ctor = TopicEndpointInfo(topic_type=test_topic_type)

        self.assertEqual(info_for_ref, info_from_ctor)
        self.assertEqual(test_topic_type, info_from_ctor.topic_type)

    def test_endpoint_type_only_constructor(self):
        test_endpoint_type = TopicEndpointTypeEnum.SUBSCRIPTION

        info_for_ref = TopicEndpointInfo()
        info_for_ref.endpoint_type = test_endpoint_type

        info_from_ctor = TopicEndpointInfo(endpoint_type=test_endpoint_type)

        self.assertEqual(info_for_ref, info_from_ctor)
        self.assertEqual(test_endpoint_type, info_from_ctor.endpoint_type)

    def test_endpoint_gid_only_constructor(self):
        test_endpoint_gid = [0, 0, 0, 0, 0]

        info_for_ref = TopicEndpointInfo()
        info_for_ref.endpoint_gid = test_endpoint_gid

        info_from_ctor = TopicEndpointInfo(endpoint_gid=test_endpoint_gid)

        self.assertEqual(info_for_ref, info_from_ctor)
        self.assertEqual(test_endpoint_gid, info_from_ctor.endpoint_gid)

    def test_qos_profile_only_constructor(self):
        c_qos_profile = _rclpy.rmw_qos_profile_t.predefined('qos_profile_default')
        test_qos_profile = QoSProfile(**c_qos_profile.to_dict())

        info_for_ref = TopicEndpointInfo()
        info_for_ref.qos_profile = test_qos_profile

        info_from_ctor = TopicEndpointInfo(qos_profile=test_qos_profile)

        self.assertEqual(info_for_ref, info_from_ctor)
        self.assertEqual(test_qos_profile, info_from_ctor.qos_profile)

    def test_print(self):
        actual_info_str = str(TopicEndpointInfo())
        expected_info_str = 'Node name: \n' \
            'Node namespace: \n' \
            'Topic type: \n' \
            'Endpoint type: INVALID\n' \
            'GID: \n' \
            'QoS profile:\n' \
            '  Reliability: UNKNOWN\n' \
            '  History (Depth): UNKNOWN\n' \
            '  Durability: UNKNOWN\n' \
            '  Lifespan: 0 nanoseconds\n' \
            '  Deadline: 0 nanoseconds\n' \
            '  Liveliness: UNKNOWN\n' \
            '  Liveliness lease duration: 0 nanoseconds'
        self.assertEqual(expected_info_str, actual_info_str)
