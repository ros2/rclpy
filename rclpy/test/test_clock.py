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

import threading
import time
import unittest
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.clock import JumpHandle
from rclpy.clock import JumpThreshold
from rclpy.clock import ROSClock
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.exceptions import NotInitializedException
from rclpy.time import Time
from rclpy.utilities import get_default_context

from .mock_compat import __name__ as _  # noqa: ignore=F401


A_SMALL_AMOUNT_OF_TIME = Duration(seconds=0.5)


def test_invalid_jump_threshold():
    with pytest.raises(ValueError, match='.*min_forward.*'):
        JumpThreshold(
            min_forward=Duration(nanoseconds=0),
            min_backward=Duration(nanoseconds=-1))

    with pytest.raises(ValueError, match='.*min_forward.*'):
        JumpThreshold(
            min_forward=Duration(nanoseconds=-1),
            min_backward=Duration(nanoseconds=-1))

    with pytest.raises(ValueError, match='.*min_backward.*'):
        JumpThreshold(
            min_forward=Duration(nanoseconds=1),
            min_backward=Duration(nanoseconds=0))

    with pytest.raises(ValueError, match='.*min_backward.*'):
        JumpThreshold(
            min_forward=Duration(nanoseconds=1),
            min_backward=Duration(nanoseconds=1))

    with pytest.raises(ValueError, match='.*must be enabled.*'):
        JumpThreshold(
            min_forward=None,
            min_backward=None,
            on_clock_change=False)


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


@pytest.fixture()
def default_context():
    rclpy.init()
    yield get_default_context()
    rclpy.shutdown()


@pytest.fixture()
def non_default_context():
    context = Context()
    context.init()
    yield context
    context.try_shutdown()


def test_sleep_until_mismatched_clock_type(default_context):
    clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    with pytest.raises(ValueError, match='.*clock type does not match.*'):
        clock.sleep_until(Time(clock_type=ClockType.STEADY_TIME))


def test_sleep_until_non_default_context(non_default_context):
    clock = Clock()
    assert clock.sleep_until(clock.now() + Duration(seconds=0.1), context=non_default_context)


def test_sleep_for_non_default_context(non_default_context):
    clock = Clock()
    assert clock.sleep_for(Duration(seconds=0.1), context=non_default_context)


def test_sleep_until_invalid_context():
    clock = Clock()
    with pytest.raises(NotInitializedException):
        clock.sleep_until(clock.now() + Duration(seconds=0.1), context=Context())


def test_sleep_for_invalid_context():
    clock = Clock()
    with pytest.raises(NotInitializedException):
        clock.sleep_for(Duration(seconds=0.1), context=Context())


@pytest.mark.parametrize(
    'clock_type', (ClockType.SYSTEM_TIME, ClockType.STEADY_TIME, ClockType.ROS_TIME))
def test_sleep_until_basic(default_context, clock_type):
    clock = Clock(clock_type=clock_type)
    sleep_duration = Duration(seconds=0.1)
    start = clock.now()
    assert clock.sleep_until(clock.now() + sleep_duration)
    stop = clock.now()
    assert stop - start >= sleep_duration


@pytest.mark.parametrize(
    'clock_type', (ClockType.SYSTEM_TIME, ClockType.STEADY_TIME, ClockType.ROS_TIME))
def test_sleep_for_basic(default_context, clock_type):
    clock = Clock(clock_type=clock_type)
    sleep_duration = Duration(seconds=0.1)
    start = clock.now()
    assert clock.sleep_for(sleep_duration)
    stop = clock.now()
    assert stop - start >= sleep_duration


@pytest.mark.parametrize(
    'clock_type', (ClockType.SYSTEM_TIME, ClockType.STEADY_TIME, ClockType.ROS_TIME))
def test_sleep_until_time_in_past(default_context, clock_type):
    clock = Clock(clock_type=clock_type)
    sleep_duration = Duration(seconds=-1)
    start = clock.now()
    assert clock.sleep_until(clock.now() + sleep_duration)
    stop = clock.now()
    assert stop - start < A_SMALL_AMOUNT_OF_TIME


@pytest.mark.parametrize(
    'clock_type', (ClockType.SYSTEM_TIME, ClockType.STEADY_TIME, ClockType.ROS_TIME))
def test_sleep_for_negative_duration(default_context, clock_type):
    clock = Clock(clock_type=clock_type)
    sleep_duration = Duration(seconds=-1)
    start = clock.now()
    assert clock.sleep_for(sleep_duration)
    stop = clock.now()
    assert stop - start < A_SMALL_AMOUNT_OF_TIME


@pytest.mark.parametrize('ros_time_enabled', (True, False))
def test_sleep_until_ros_time_toggled(default_context, ros_time_enabled):
    clock = ROSClock()
    clock._set_ros_time_is_active(not ros_time_enabled)

    retval = None

    def run():
        nonlocal retval
        retval = clock.sleep_until(clock.now() + Duration(seconds=10))

    t = threading.Thread(target=run)
    t.start()

    # wait for thread to get inside sleep_until call
    time.sleep(0.2)

    clock._set_ros_time_is_active(ros_time_enabled)

    # wait for thread to exit
    start = clock.now()
    t.join()
    stop = clock.now()
    assert stop - start < A_SMALL_AMOUNT_OF_TIME

    assert retval is False


@pytest.mark.parametrize('ros_time_enabled', (True, False))
def test_sleep_for_ros_time_toggled(default_context, ros_time_enabled):
    clock = ROSClock()
    clock._set_ros_time_is_active(not ros_time_enabled)

    retval = None

    def run():
        nonlocal retval
        retval = clock.sleep_for(Duration(seconds=10))

    t = threading.Thread(target=run)
    t.start()

    # wait for thread to get inside sleep_for call
    time.sleep(0.2)

    clock._set_ros_time_is_active(ros_time_enabled)

    # wait for thread to exit
    start = clock.now()
    t.join()
    stop = clock.now()
    assert stop - start < A_SMALL_AMOUNT_OF_TIME

    assert retval is False


def test_sleep_until_context_shut_down(non_default_context):
    clock = Clock()
    retval = None

    def run():
        nonlocal retval
        retval = clock.sleep_until(
            clock.now() + Duration(seconds=10), context=non_default_context)

    t = threading.Thread(target=run)
    t.start()

    # wait for thread to get inside sleep_until call
    time.sleep(0.2)

    non_default_context.shutdown()

    # wait for thread to exit
    start = clock.now()
    t.join()
    stop = clock.now()
    assert stop - start < A_SMALL_AMOUNT_OF_TIME

    assert retval is False


def test_sleep_for_context_shut_down(non_default_context):
    clock = Clock()
    retval = None

    def run():
        nonlocal retval
        retval = clock.sleep_for(Duration(seconds=10), context=non_default_context)

    t = threading.Thread(target=run)
    t.start()

    # wait for thread to get inside sleep_for call
    time.sleep(0.2)

    non_default_context.shutdown()

    # wait for thread to exit
    start = clock.now()
    t.join()
    stop = clock.now()
    assert stop - start < A_SMALL_AMOUNT_OF_TIME

    assert retval is False


def test_sleep_until_ros_time_enabled(default_context):
    clock = ROSClock()
    clock._set_ros_time_is_active(True)

    start_time = Time(seconds=1, clock_type=ClockType.ROS_TIME)
    stop_time = start_time + Duration(seconds=10)
    clock.set_ros_time_override(start_time)

    retval = None

    def run():
        nonlocal retval
        retval = clock.sleep_until(stop_time)

    t = threading.Thread(target=run)
    t.start()

    # wait for thread to get inside sleep_until call
    time.sleep(0.2)

    clock.set_ros_time_override(stop_time)

    # wait for thread to exit
    start = clock.now()
    t.join()
    stop = clock.now()
    assert stop - start < A_SMALL_AMOUNT_OF_TIME

    assert retval


def test_sleep_for_ros_time_enabled(default_context):
    clock = ROSClock()
    clock._set_ros_time_is_active(True)

    start_time = Time(seconds=1, clock_type=ClockType.ROS_TIME)
    sleep_duration = Duration(seconds=10)
    stop_time = start_time + sleep_duration
    clock.set_ros_time_override(start_time)

    retval = None

    def run():
        nonlocal retval
        retval = clock.sleep_for(sleep_duration)

    t = threading.Thread(target=run)
    t.start()

    # wait for thread to get inside sleep_for call
    time.sleep(0.2)

    clock.set_ros_time_override(stop_time)

    # wait for thread to exit
    start = clock.now()
    t.join()
    stop = clock.now()
    assert stop - start < A_SMALL_AMOUNT_OF_TIME

    assert retval


def test_with_jump_handle():
    clock = ROSClock()
    clock._set_ros_time_is_active(False)

    post_callback = Mock()
    threshold = JumpThreshold(min_forward=None, min_backward=None, on_clock_change=True)

    with clock.create_jump_callback(threshold, post_callback=post_callback) as jump_handler:
        assert isinstance(jump_handler, JumpHandle)
        clock._set_ros_time_is_active(True)
        post_callback.assert_called_once()

    post_callback.reset_mock()
    clock._set_ros_time_is_active(False)
    post_callback.assert_not_called()
