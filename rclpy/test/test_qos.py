# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

from rclpy.duration import Duration
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class TestQosProfile(unittest.TestCase):

    def convert_and_assert_equality(self, qos_profile):
        c_profile = qos_profile.get_c_qos_profile()
        converted_profile = _rclpy.rclpy_convert_to_py_qos_policy(c_profile)
        self.assertEqual(qos_profile, converted_profile)

    def test_eq_operator(self):
        profile_1 = QoSProfile()
        profile_same = QoSProfile()
        profile_different_number = QoSProfile(depth=1)
        profile_different_duration = QoSProfile(deadline=Duration(seconds=2))
        profile_equal_duration = QoSProfile(deadline=Duration(seconds=2))

        self.assertEqual(profile_1, profile_same)
        self.assertNotEqual(profile_1, profile_different_number)
        self.assertNotEqual(profile_1, profile_different_duration)
        self.assertEqual(profile_different_duration, profile_equal_duration)

    def test_simple_round_trip(self):
        source_profile = QoSProfile()
        self.convert_and_assert_equality(source_profile)

    def test_big_nanoseconds(self):
        # Under 31 bits
        no_problem = QoSProfile(lifespan=Duration(seconds=2))
        self.convert_and_assert_equality(no_problem)

        # Total nanoseconds in duration is too large to store in 32 bit signed int
        int32_problem = QoSProfile(lifespan=Duration(seconds=4))
        self.convert_and_assert_equality(int32_problem)

        # Total nanoseconds in duration is too large to store in 32 bit unsigned int
        uint32_problem = QoSProfile(lifespan=Duration(seconds=5))
        self.convert_and_assert_equality(uint32_problem)

    def test_alldata_round_trip(self):
        source_profile = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            depth=12,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            lifespan=Duration(seconds=4),
            deadline=Duration(nanoseconds=1e6),
            liveliness=QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE,
            liveliness_lease_duration=Duration(nanoseconds=12),
            avoid_ros_namespace_conventions=True
        )
        self.convert_and_assert_equality(source_profile)
