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
from unittest.mock import Mock

from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.clock import JumpThreshold
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.time import Time

from .mock_compat import __name__ as _  # noqa: ignore=F401


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

    def test_ros_time_is_active(self):
        clock = ROSClock()
        clock._set_ros_time_is_active(True)
        assert clock.ros_time_is_active
        clock._set_ros_time_is_active(False)
        assert not clock.ros_time_is_active

    def test_triggered_time_jump_callbacks(self):
        one_second = Duration(seconds=1)
        half_second = Duration(seconds=0.5)
        negative_half_second = Duration(seconds=-0.5)
        negative_one_second = Duration(seconds=-1)

        threshold1 = JumpThreshold(
            min_forward=one_second, min_backward=negative_half_second, on_clock_change=False)
        threshold2 = JumpThreshold(
            min_forward=half_second, min_backward=negative_one_second, on_clock_change=False)

        pre_callback1 = Mock()
        post_callback1 = Mock()
        pre_callback2 = Mock()
        post_callback2 = Mock()

        clock = ROSClock()
        handler1 = clock.create_jump_callback(
            threshold1, pre_callback=pre_callback1, post_callback=post_callback1)
        handler2 = clock.create_jump_callback(
            threshold2, pre_callback=pre_callback2, post_callback=post_callback2)

        clock.set_ros_time_override(Time(seconds=1))
        clock._set_ros_time_is_active(True)
        pre_callback1.assert_not_called()
        post_callback1.assert_not_called()
        pre_callback2.assert_not_called()
        post_callback2.assert_not_called()

        # forward jump
        clock.set_ros_time_override(Time(seconds=1.75))
        pre_callback1.assert_not_called()
        post_callback1.assert_not_called()
        pre_callback2.assert_called()
        post_callback2.assert_called()

        pre_callback1.reset_mock()
        post_callback1.reset_mock()
        pre_callback2.reset_mock()
        post_callback2.reset_mock()

        # backwards jump
        clock.set_ros_time_override(Time(seconds=1))
        pre_callback1.assert_called()
        post_callback1.assert_called()
        pre_callback2.assert_not_called()
        post_callback2.assert_not_called()

        handler1.unregister()
        handler2.unregister()

    def test_triggered_clock_change_callbacks(self):
        one_second = Duration(seconds=1)
        negative_one_second = Duration(seconds=-1)

        threshold1 = JumpThreshold(
            min_forward=one_second, min_backward=negative_one_second, on_clock_change=False)
        threshold2 = JumpThreshold(min_forward=None, min_backward=None, on_clock_change=True)
        threshold3 = JumpThreshold(
            min_forward=one_second, min_backward=negative_one_second, on_clock_change=True)

        pre_callback1 = Mock()
        post_callback1 = Mock()
        pre_callback2 = Mock()
        post_callback2 = Mock()
        pre_callback3 = Mock()
        post_callback3 = Mock()

        clock = ROSClock()
        handler1 = clock.create_jump_callback(
            threshold1, pre_callback=pre_callback1, post_callback=post_callback1)
        handler2 = clock.create_jump_callback(
            threshold2, pre_callback=pre_callback2, post_callback=post_callback2)
        handler3 = clock.create_jump_callback(
            threshold3, pre_callback=pre_callback3, post_callback=post_callback3)

        clock._set_ros_time_is_active(True)
        pre_callback1.assert_not_called()
        post_callback1.assert_not_called()
        pre_callback2.assert_called()
        post_callback2.assert_called()
        pre_callback3.assert_called()
        post_callback3.assert_called()

        pre_callback1.reset_mock()
        post_callback1.reset_mock()
        pre_callback2.reset_mock()
        post_callback2.reset_mock()
        pre_callback3.reset_mock()
        post_callback3.reset_mock()

        clock._set_ros_time_is_active(True)
        pre_callback1.assert_not_called()
        post_callback1.assert_not_called()
        pre_callback2.assert_not_called()
        post_callback2.assert_not_called()
        pre_callback3.assert_not_called()
        post_callback3.assert_not_called()

        handler1.unregister()
        handler2.unregister()
        handler3.unregister()
