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

from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Header


class TestTime(unittest.TestCase):

    def test_time_construction(self):
        time = Time()
        assert time.nanoseconds == 0

        time = Time(seconds=1, nanoseconds=5e8)
        assert time.nanoseconds == 1500000000

        with self.assertRaises(ValueError):
            time = Time(seconds=-1)
        with self.assertRaises(ValueError):
            time = Time(nanoseconds=-1)

    def test_time_operators(self):
        time1 = Time(nanoseconds=1)
        time2 = time1
        assert time2.nanoseconds == 1

        with self.assertRaises(TypeError):
            time2 = time1 + time2
        duration = Duration(nanoseconds=1)
        with self.assertRaises(TypeError):
            time2 = time1 - duration
        with self.assertRaises(TypeError):
            time2 = duration - time1

        time3 = time1 + duration
        assert isinstance(time3, Time)
        assert time3.nanoseconds == 2
        time3 = duration + time1
        assert isinstance(time3, Time)
        assert time3.nanoseconds == 2

        diff = time3 - time1
        assert isinstance(diff, Duration)
        assert diff.nanoseconds == 1

    def test_time_message_conversions(self):
        time1 = Time(nanoseconds=1)
        time2 = Time(nanoseconds=2)
        header = Header()
        header.stamp = time1.to_msg()

        diff = time2 - Time.from_msg(header.stamp)
        assert isinstance(diff, Duration)
        assert diff.nanoseconds == 1
