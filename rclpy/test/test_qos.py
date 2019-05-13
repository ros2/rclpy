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
import warnings

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

    def test_depth_only_constructor(self):
        qos = QoSProfile(depth=1)
        assert qos.depth == 1
        assert qos.history == QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST

    def test_eq_operator(self):
        profile_1 = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1)
        profile_same = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1)
        profile_different_depth = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=2)
        profile_different_duration = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
            deadline=Duration(seconds=2))
        profile_equal_duration = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
            deadline=Duration(seconds=2))

        self.assertEqual(profile_1, profile_same)
        self.assertNotEqual(profile_1, profile_different_depth)
        self.assertNotEqual(profile_1, profile_different_duration)
        self.assertEqual(profile_different_duration, profile_equal_duration)

    def test_simple_round_trip(self):
        source_profile = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
        self.convert_and_assert_equality(source_profile)

    def test_big_nanoseconds(self):
        # Under 31 bits
        no_problem = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            lifespan=Duration(seconds=2))
        self.convert_and_assert_equality(no_problem)

        # Total nanoseconds in duration is too large to store in 32 bit signed int
        int32_problem = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            lifespan=Duration(seconds=4))
        self.convert_and_assert_equality(int32_problem)

        # Total nanoseconds in duration is too large to store in 32 bit unsigned int
        uint32_problem = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            lifespan=Duration(seconds=5))
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

    def test_deprecation_warnings(self):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter('always')
            QoSProfile()
            assert len(w) == 1
            assert issubclass(w[0].category, UserWarning)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter('always')
            # No deprecation if history is supplied
            QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
            assert len(w) == 0, str(w[-1].message)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter('always')
            # Deprecation warning if KEEP_LAST, but no depth
            QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)
            assert len(w) == 1
            assert issubclass(w[0].category, UserWarning)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter('always')
            # No deprecation if 'depth' is present
            QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1)
            assert len(w) == 0, str(w[-1].message)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter('always')
            # No deprecation if only depth
            QoSProfile(depth=1)
            assert len(w) == 0, str(w[-1].message)
