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
from rclpy.clock import ClockChange
from rclpy.clock import ClockType
from rclpy.clock import JumpThreshold
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.time_source import TimeJump


def do_nothing(jump_info):
    pass


class TestClock(unittest.TestCase):

    def test_clock_construction(self):
        clock = Clock()

        with self.assertRaises(TypeError):
            clock = Clock(clock_type='STEADY_TIME')

        clock = Clock(clock_type=ClockType.STEADY_TIME)
        assert clock.clock_type == ClockType.STEADY_TIME
        clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        assert clock.clock_type == ClockType.SYSTEM_TIME
        # A subclass ROSClock is returned if ROS_TIME is specified.
        clock = Clock(clock_type=ClockType.ROS_TIME)
        assert clock.clock_type == ClockType.ROS_TIME
        assert isinstance(clock, ROSClock)

        # Direct instantiation of a ROSClock is also possible.
        clock = ROSClock()
        assert clock.clock_type == ClockType.ROS_TIME

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
        assert now2 > now

        # Steady time should always return increasing values
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        now = clock.now()
        now2 = now
        for i in range(10):
            now2 = clock.now()
            assert now2 > now
            now = now2

    def test_get_triggered_no_callbacks(self):
        jump_info = TimeJump(ClockChange.ROS_TIME_ACTIVATED, Duration())
        assert [] == list(ROSClock().get_triggered_callback_handlers(jump_info))

    def test_get_triggered_no_callbacks_match_clock_change(self):
        jump_info = TimeJump(ClockChange.ROS_TIME_ACTIVATED, Duration())
        assert [] == list(ROSClock().get_triggered_callback_handlers(jump_info))

    def test_get_triggered_callbacks_time_jump(self):
        one_second = Duration(seconds=1)
        half_second = Duration(seconds=0.5)
        negative_half_second = Duration(seconds=-0.5)
        negative_one_second = Duration(seconds=-1)

        threshold1 = JumpThreshold(
            min_forward=one_second, min_backward=negative_half_second, on_clock_change=False)
        threshold2 = JumpThreshold(
            min_forward=half_second, min_backward=negative_one_second, on_clock_change=False)

        forward_jump_info = TimeJump(ClockChange.ROS_TIME_NO_CHANGE, Duration(seconds=0.75))
        backward_jump_info = TimeJump(ClockChange.ROS_TIME_NO_CHANGE, Duration(seconds=-0.75))

        clock = ROSClock()
        handler1 = clock.create_jump_callback(threshold1, pre_callback=do_nothing)
        handler2 = clock.create_jump_callback(threshold2, pre_callback=do_nothing)

        assert [handler2] == list(clock.get_triggered_callback_handlers(forward_jump_info))
        assert [handler1] == list(clock.get_triggered_callback_handlers(backward_jump_info))

    def test_get_triggered_callbacks_clock_change(self):
        one_second = Duration(seconds=1)
        negative_one_second = Duration(seconds=-1)

        threshold1 = JumpThreshold(
            min_forward=one_second, min_backward=negative_one_second, on_clock_change=False)
        threshold2 = JumpThreshold(min_forward=None, min_backward=None, on_clock_change=True)
        threshold3 = JumpThreshold(
            min_forward=one_second, min_backward=negative_one_second, on_clock_change=True)

        clock_change1 = TimeJump(ClockChange.ROS_TIME_ACTIVATED, Duration())
        clock_change2 = TimeJump(ClockChange.ROS_TIME_DEACTIVATED, Duration())
        no_change1 = TimeJump(ClockChange.ROS_TIME_NO_CHANGE, Duration())
        no_change2 = TimeJump(ClockChange.SYSTEM_TIME_NO_CHANGE, Duration())

        clock = ROSClock()
        handler1 = clock.create_jump_callback(threshold1, pre_callback=do_nothing)  # noqa
        handler2 = clock.create_jump_callback(threshold2, pre_callback=do_nothing)
        handler3 = clock.create_jump_callback(threshold3, pre_callback=do_nothing)

        assert [handler2, handler3] == list(clock.get_triggered_callback_handlers(clock_change1))
        assert [handler2, handler3] == list(clock.get_triggered_callback_handlers(clock_change2))
        assert [] == list(clock.get_triggered_callback_handlers(no_change1))
        assert [] == list(clock.get_triggered_callback_handlers(no_change2))
