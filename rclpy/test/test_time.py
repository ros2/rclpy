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

import unittest

from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.time import Time

from test_msgs.msg import Builtins


class TestTime(unittest.TestCase):

    def test_time_construction(self):
        time = Time()
        assert time.nanoseconds == 0

        time = Time(seconds=1, nanoseconds=5e8, clock_type=ClockType.SYSTEM_TIME)
        assert time.nanoseconds == 1500000000
        assert time.clock_type == ClockType.SYSTEM_TIME

        with self.assertRaises(ValueError):
            time = Time(seconds=-1)
        with self.assertRaises(ValueError):
            time = Time(nanoseconds=-1)
        with self.assertRaises(TypeError):
            time = Time(clock_type='SYSTEM_TIME')

    def test_time_operators(self):
        # Check that modifying the original doesn't affect a copy
        time1 = Time(nanoseconds=2, clock_type=ClockType.SYSTEM_TIME)
        time2 = time1
        time1 = Time(nanoseconds=1, clock_type=ClockType.STEADY_TIME)
        assert time2.nanoseconds == 2
        assert time2.clock_type == ClockType.SYSTEM_TIME

        # Addition/subtraction of time and duration
        duration = Duration(nanoseconds=1)
        time3 = time1 + duration
        assert isinstance(time3, Time)
        assert time3.nanoseconds == 2
        assert time3.clock_type == ClockType.STEADY_TIME

        time3 = duration + time1
        assert isinstance(time3, Time)
        assert time3.nanoseconds == 2
        assert time3.clock_type == ClockType.STEADY_TIME

        time3 = time1 - duration
        assert isinstance(time3, Time)
        assert time3.nanoseconds == 0
        assert time3.clock_type == ClockType.STEADY_TIME

        # Subtraction of times with the same clock type
        diff = time1 - time3
        assert isinstance(diff, Duration)
        assert diff.nanoseconds == 1

        # Subtraction of times with different clock types
        time4 = Time(nanoseconds=1, clock_type=ClockType.SYSTEM_TIME)
        with self.assertRaises(TypeError):
            diff = time1 - time4

        # Invalid combinations
        with self.assertRaises(TypeError):
            time2 = time1 + time2
        with self.assertRaises(TypeError):
            time2 = duration - time1

    def test_time_comparators(self):
        # Times with the same clock type
        time1 = Time(nanoseconds=1)
        time2 = Time(nanoseconds=2)
        self.assertFalse(time1 == time2)
        self.assertTrue(time1 != time2)
        self.assertFalse(time1 > time2)
        self.assertFalse(time1 >= time2)
        self.assertTrue(time1 < time2)
        self.assertTrue(time1 <= time2)

        time1 = Time(nanoseconds=5e9)
        time2 = Time(seconds=5)
        self.assertTrue(time1 == time2)

        # Supported combinations with other types
        time1 = Time(nanoseconds=1)
        self.assertFalse(time1 == 1)
        duration = Duration()
        self.assertFalse(time1 == duration)
        self.assertTrue(time1 != duration)

        # Times with different clock types
        time1 = Time(nanoseconds=1, clock_type=ClockType.SYSTEM_TIME)
        time2 = Time(nanoseconds=2, clock_type=ClockType.STEADY_TIME)
        with self.assertRaises(TypeError):
            time1 == time2
        with self.assertRaises(TypeError):
            time1 != time2
        with self.assertRaises(TypeError):
            time1 > time2
        with self.assertRaises(TypeError):
            time1 >= time2
        with self.assertRaises(TypeError):
            time1 < time2
        with self.assertRaises(TypeError):
            time1 <= time2

        # Invalid combinations
        with self.assertRaises(TypeError):
            time1 > duration
        with self.assertRaises(TypeError):
            time1 >= duration
        with self.assertRaises(TypeError):
            time1 < duration
        with self.assertRaises(TypeError):
            time1 <= duration

    def test_time_message_conversions(self):
        time1 = Time(nanoseconds=1, clock_type=ClockType.ROS_TIME)
        builtins_msg = Builtins()
        builtins_msg.time_value = time1.to_msg()

        # Default clock type resulting from from_msg will be ROS time
        time2 = Time.from_msg(builtins_msg.time_value)
        assert isinstance(time2, Time)
        assert time1 == time2
        # Clock type can be specified if appropriate
        time3 = Time.from_msg(builtins_msg.time_value, clock_type=ClockType.SYSTEM_TIME)
        assert time3.clock_type == ClockType.SYSTEM_TIME

        duration = Duration(nanoseconds=1)
        builtins_msg.duration_value = duration.to_msg()
        duration2 = Duration.from_msg(builtins_msg.duration_value)
        assert isinstance(duration2, Duration)
        assert duration2.nanoseconds == 1
