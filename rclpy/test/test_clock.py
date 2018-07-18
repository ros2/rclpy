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

import time
import unittest

from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.time import Time


class TestClock(unittest.TestCase):

    def test_clock_construction(self):
        clock = Clock()

        with self.assertRaises(TypeError):
            clock = Clock(clock_type='STEADY_TIME')

        with self.assertRaises(NotImplementedError):
            clock = Clock(clock_type=ClockType.ROS_TIME)
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        assert clock.clock_type == ClockType.STEADY_TIME
        clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        assert clock.clock_type == ClockType.SYSTEM_TIME

    def test_clock_now(self):
        # System time should be roughly equal to time.time()
        # There will still be differences between them, with the bound depending on the scheduler.
        clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        now = clock.now()
        python_time_sec = time.time()
        assert isinstance(now, Time)
        assert abs(now.nanoseconds * 1e-9 - python_time_sec) < 5

        # Unless there is a date change during the test, system time have increased between these
        # calls.
        now2 = clock.now()
        # TODO(dhood): use comparators when implemented
        assert (now2 - now).nanoseconds > 0

        # Steady time should always return increasing values
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        now = clock.now()
        now2 = now
        for i in range(10):
            now2 = clock.now()
            assert (now2 - now).nanoseconds > 0
            now = now2
