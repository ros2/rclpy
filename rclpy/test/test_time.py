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

        with self.assertRaises(OverflowError):
            time = Time(nanoseconds=2**63)
        time = Time(nanoseconds=2**63 - 1)
        assert time.nanoseconds == 2**63 - 1

        with self.assertRaises(ValueError):
            time = Time(seconds=-1)
        with self.assertRaises(ValueError):
            time = Time(nanoseconds=-1)
        with self.assertRaises(TypeError):
            time = Time(clock_type='SYSTEM_TIME')

    def test_duration_construction(self):
        duration = Duration()
        assert duration.nanoseconds == 0

        duration = Duration(seconds=1, nanoseconds=5e8)
        assert duration.nanoseconds == 1500000000

        with self.assertRaises(OverflowError):
            duration = Duration(nanoseconds=2**63)
        duration = Duration(nanoseconds=2**63 - 1)
        assert duration.nanoseconds == 2**63 - 1

        assert Duration(seconds=-1).nanoseconds == -1 * 1000 * 1000 * 1000
        assert Duration(nanoseconds=-1).nanoseconds == -1

        assert Duration(seconds=-2**63 / 1e9).nanoseconds == -2**63
        with self.assertRaises(OverflowError):
            # Much smaller number because float to integer conversion of seconds is imprecise
            duration = Duration(seconds=-2**63 / 1e9 - 1)

        assert Duration(nanoseconds=-2**63).nanoseconds == -2**63
        with self.assertRaises(OverflowError):
            Duration(nanoseconds=-2**63 - 1)

    def test_time_operators(self):
        time1 = Time(nanoseconds=1, clock_type=ClockType.STEADY_TIME)

        # Addition/subtraction of time and duration
        duration = Duration(nanoseconds=1)
        time2 = time1 + duration
        assert isinstance(time2, Time)
        assert time2.nanoseconds == 2
        assert time2.clock_type == ClockType.STEADY_TIME

        time2 = duration + time1
        assert isinstance(time2, Time)
        assert time2.nanoseconds == 2
        assert time2.clock_type == ClockType.STEADY_TIME

        time2 = time1 - duration
        assert isinstance(time2, Time)
        assert time2.nanoseconds == 0
        assert time2.clock_type == ClockType.STEADY_TIME

        with self.assertRaises(OverflowError):
            Duration(nanoseconds=1) + Time(nanoseconds=2**63 - 1)

        with self.assertRaises(ValueError):
            Time(nanoseconds=1) - Duration(nanoseconds=2)

        # Subtraction of times with the same clock type
        diff = time1 - time2
        assert isinstance(diff, Duration)
        assert diff.nanoseconds == 1

        # Subtraction resulting in a negative duration
        assert (Time(nanoseconds=1) - Time(nanoseconds=2)).nanoseconds == -1

        # Subtraction of times with different clock types
        with self.assertRaises(TypeError):
            Time(nanoseconds=2, clock_type=ClockType.SYSTEM_TIME) - \
                Time(nanoseconds=1, clock_type=ClockType.STEADY_TIME)

        # Invalid arithmetic combinations
        with self.assertRaises(TypeError):
            time1 + time2
        with self.assertRaises(TypeError):
            duration - time1

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
        time1 = Time(nanoseconds=1)
        with self.assertRaises(TypeError):
            time1 == 1
        duration = Duration()
        with self.assertRaises(TypeError):
            time1 == duration
        with self.assertRaises(TypeError):
            time1 != duration
        with self.assertRaises(TypeError):
            time1 > duration
        with self.assertRaises(TypeError):
            time1 >= duration
        with self.assertRaises(TypeError):
            time1 < duration
        with self.assertRaises(TypeError):
            time1 <= duration

    def test_duration_comparators(self):
        duration1 = Duration(nanoseconds=1)
        duration2 = Duration(nanoseconds=2)
        self.assertFalse(duration1 == duration2)
        self.assertTrue(duration1 != duration2)
        self.assertFalse(duration1 > duration2)
        self.assertFalse(duration1 >= duration2)
        self.assertTrue(duration1 < duration2)
        self.assertTrue(duration1 <= duration2)

        duration1 = Duration(nanoseconds=5e9)
        duration2 = Duration(seconds=5)
        self.assertTrue(duration1 == duration2)

        # Invalid combinations
        duration1 = Duration(nanoseconds=1)
        with self.assertRaises(TypeError):
            duration1 == 1
        time = Time()
        with self.assertRaises(TypeError):
            duration1 == time
        with self.assertRaises(TypeError):
            duration1 != time
        with self.assertRaises(TypeError):
            duration1 > time
        with self.assertRaises(TypeError):
            duration1 >= time
        with self.assertRaises(TypeError):
            duration1 < time
        with self.assertRaises(TypeError):
            duration1 <= time

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

    def test_time_message_conversions_big_nanoseconds(self):
        time1 = Time(nanoseconds=1553575413247045598, clock_type=ClockType.ROS_TIME)
        builtins_msg = Builtins()
        builtins_msg.time_value = time1.to_msg()

        # Default clock type resulting from from_msg will be ROS time
        time2 = Time.from_msg(builtins_msg.time_value)
        assert isinstance(time2, Time)
        assert time1 == time2

    def test_duration_message_conversions(self):
        duration = Duration(nanoseconds=1)
        builtins_msg = Builtins()
        builtins_msg.duration_value = duration.to_msg()
        duration2 = Duration.from_msg(builtins_msg.duration_value)
        assert isinstance(duration2, Duration)
        assert duration2.nanoseconds == 1

    def test_seconds_nanoseconds(self):
        assert (1, int(5e8)) == Time(seconds=1, nanoseconds=5e8).seconds_nanoseconds()
        assert (1, int(5e8)) == Time(seconds=0, nanoseconds=15e8).seconds_nanoseconds()
        assert (0, 0) == Time().seconds_nanoseconds()
