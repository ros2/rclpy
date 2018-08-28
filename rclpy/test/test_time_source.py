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

import builtin_interfaces.msg
import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockChange
from rclpy.clock import ClockType
from rclpy.clock import JumpThreshold
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.time_source import CLOCK_TOPIC
from rclpy.time_source import TimeSource


class TestTimeSource(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('TestTimeSource', namespace='/rclpy')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def publish_clock_messages(self):
        clock_pub = self.node.create_publisher(builtin_interfaces.msg.Time, CLOCK_TOPIC)
        cycle_count = 0
        time_msg = builtin_interfaces.msg.Time()
        while rclpy.ok() and cycle_count < 5:
            time_msg.sec = cycle_count
            clock_pub.publish(time_msg)
            cycle_count += 1
            rclpy.spin_once(self.node, timeout_sec=1)
            # TODO(dhood): use rate once available
            time.sleep(1)

    def publish_reversed_clock_messages(self):
        clock_pub = self.node.create_publisher(builtin_interfaces.msg.Time, CLOCK_TOPIC)
        cycle_count = 0
        time_msg = builtin_interfaces.msg.Time()
        while rclpy.ok() and cycle_count < 5:
            time_msg.sec = 6 - cycle_count
            clock_pub.publish(time_msg)
            cycle_count += 1
            rclpy.spin_once(self.node, timeout_sec=1)
            time.sleep(1)

    def test_time_source_attach_clock(self):
        time_source = TimeSource(node=self.node)

        # ROSClock is a specialization of Clock with ROS time methods.
        time_source.attach_clock(ROSClock())

        # Other clock types are not supported.
        with self.assertRaises(ValueError):
            time_source.attach_clock(Clock(clock_type=ClockType.SYSTEM_TIME))

        with self.assertRaises(ValueError):
            time_source.attach_clock(Clock(clock_type=ClockType.STEADY_TIME))

    def test_time_source_not_using_sim_time(self):
        time_source = TimeSource(node=self.node)
        clock = ROSClock()
        time_source.attach_clock(clock)

        # When not using sim time, ROS time should look like system time
        now = clock.now()
        system_now = Clock(clock_type=ClockType.SYSTEM_TIME).now()
        assert (system_now.nanoseconds - now.nanoseconds) < 1e9

        # Presence of clock publisher should not affect the clock
        self.publish_clock_messages()
        self.assertFalse(clock.ros_time_is_active)
        now = clock.now()
        system_now = Clock(clock_type=ClockType.SYSTEM_TIME).now()
        assert (system_now.nanoseconds - now.nanoseconds) < 1e9

        # Whether or not an attached clock is using ROS time should be determined by the time
        # source managing it.
        self.assertFalse(time_source.ros_time_is_active)
        clock2 = ROSClock()
        clock2._set_ros_time_is_active(True)
        time_source.attach_clock(clock2)
        self.assertFalse(clock2.ros_time_is_active)
        assert time_source._clock_sub is None

    def test_time_source_using_sim_time(self):
        time_source = TimeSource(node=self.node)
        clock = ROSClock()
        time_source.attach_clock(clock)

        # Setting ROS time active on a time source should also cause attached clocks' use of ROS
        # time to be set to active.
        self.assertFalse(time_source.ros_time_is_active)
        self.assertFalse(clock.ros_time_is_active)
        time_source.ros_time_is_active = True
        self.assertTrue(time_source.ros_time_is_active)
        self.assertTrue(clock.ros_time_is_active)

        # A subscriber should have been created
        assert time_source._clock_sub is not None

        # Before any messages have been received on the /clock topic, now() should return 0
        assert clock.now() == Time(seconds=0, clock_type=ClockType.ROS_TIME)

        # When using sim time, ROS time should look like the messages received on /clock
        self.publish_clock_messages()
        assert clock.now() > Time(seconds=0, clock_type=ClockType.ROS_TIME)
        assert clock.now() <= Time(seconds=5, clock_type=ClockType.ROS_TIME)

        # Check that attached clocks get the cached message
        clock2 = Clock(clock_type=ClockType.ROS_TIME)
        time_source.attach_clock(clock2)
        assert clock2.now() > Time(seconds=0, clock_type=ClockType.ROS_TIME)
        assert clock2.now() <= Time(seconds=5, clock_type=ClockType.ROS_TIME)

        # Check detaching the node
        time_source.detach_node()
        node2 = rclpy.create_node('TestTimeSource2', namespace='/rclpy')
        time_source.attach_node(node2)
        node2.destroy_node()
        assert time_source._node == node2
        assert time_source._clock_sub is not None

    def test_forwards_jump(self):
        time_source = TimeSource(node=self.node)
        clock = ROSClock()
        time_source.attach_clock(clock)
        time_source.ros_time_is_active = True

        pre_cb = Mock()
        post_cb = Mock()
        threshold = JumpThreshold(
            min_forward=Duration(seconds=0.5), min_backward=None, on_clock_change=False)
        handler = clock.create_jump_callback(
            threshold, pre_callback=pre_cb, post_callback=post_cb)

        self.publish_clock_messages()

        pre_cb.assert_called()
        post_cb.assert_called()
        assert post_cb.call_args[0][0].clock_change == ClockChange.ROS_TIME_NO_CHANGE
        handler.unregister()

    def test_backwards_jump(self):
        time_source = TimeSource(node=self.node)
        clock = ROSClock()
        time_source.attach_clock(clock)
        time_source.ros_time_is_active = True

        pre_cb = Mock()
        post_cb = Mock()
        threshold = JumpThreshold(
            min_forward=None, min_backward=Duration(seconds=-0.5), on_clock_change=False)
        handler = clock.create_jump_callback(
            threshold, pre_callback=pre_cb, post_callback=post_cb)

        self.publish_reversed_clock_messages()

        pre_cb.assert_called()
        post_cb.assert_called()
        assert post_cb.call_args[0][0].clock_change == ClockChange.ROS_TIME_NO_CHANGE
        handler.unregister()

    def test_clock_change(self):
        time_source = TimeSource(node=self.node)
        clock = ROSClock()
        time_source.attach_clock(clock)
        time_source.ros_time_is_active = True

        pre_cb = Mock()
        post_cb = Mock()
        threshold = JumpThreshold(min_forward=None, min_backward=None, on_clock_change=True)
        handler = clock.create_jump_callback(
            threshold, pre_callback=pre_cb, post_callback=post_cb)

        time_source.ros_time_is_active = False
        pre_cb.assert_called()
        post_cb.assert_called()
        assert post_cb.call_args[0][0].clock_change == ClockChange.ROS_TIME_DEACTIVATED

        pre_cb.reset_mock()
        post_cb.reset_mock()

        time_source.ros_time_is_active = True
        pre_cb.assert_called()
        post_cb.assert_called()
        assert post_cb.call_args[0][0].clock_change == ClockChange.ROS_TIME_ACTIVATED
        handler.unregister()

    def test_no_pre_callback(self):
        time_source = TimeSource(node=self.node)
        clock = ROSClock()
        time_source.attach_clock(clock)
        time_source.ros_time_is_active = True

        post_cb = Mock()
        threshold = JumpThreshold(min_forward=None, min_backward=None, on_clock_change=True)
        handler = clock.create_jump_callback(
            threshold, pre_callback=None, post_callback=post_cb)

        time_source.ros_time_is_active = False
        post_cb.assert_called_once()
        assert post_cb.call_args[0][0].clock_change == ClockChange.ROS_TIME_DEACTIVATED
        handler.unregister()

    def test_no_post_callback(self):
        time_source = TimeSource(node=self.node)
        clock = ROSClock()
        time_source.attach_clock(clock)
        time_source.ros_time_is_active = True

        pre_cb = Mock()
        threshold = JumpThreshold(min_forward=None, min_backward=None, on_clock_change=True)
        handler = clock.create_jump_callback(
            threshold, pre_callback=pre_cb, post_callback=None)

        time_source.ros_time_is_active = False
        pre_cb.assert_called_once()
        handler.unregister()
