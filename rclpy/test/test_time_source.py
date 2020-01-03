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

import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockChange
from rclpy.clock import ClockType
from rclpy.clock import JumpThreshold
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.time_source import CLOCK_TOPIC
from rclpy.time_source import TimeSource
import rosgraph_msgs.msg

from .mock_compat import __name__ as _  # noqa: ignore=F401


class TestTimeSource(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            'TestTimeSource', namespace='/rclpy', context=self.context,
            allow_undeclared_parameters=True)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def publish_clock_messages(self):
        clock_pub = self.node.create_publisher(rosgraph_msgs.msg.Clock, CLOCK_TOPIC, 1)
        cycle_count = 0
        time_msg = rosgraph_msgs.msg.Clock()
        while rclpy.ok(context=self.context) and cycle_count < 5:
            time_msg.clock.sec = cycle_count
            clock_pub.publish(time_msg)
            cycle_count += 1
            executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
            rclpy.spin_once(self.node, timeout_sec=1, executor=executor)
            # TODO(dhood): use rate once available
            time.sleep(1)

    def publish_reversed_clock_messages(self):
        clock_pub = self.node.create_publisher(rosgraph_msgs.msg.Clock, CLOCK_TOPIC, 1)
        cycle_count = 0
        time_msg = rosgraph_msgs.msg.Clock()
        while rclpy.ok(context=self.context) and cycle_count < 5:
            time_msg.clock.sec = 6 - cycle_count
            clock_pub.publish(time_msg)
            cycle_count += 1
            executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
            rclpy.spin_once(self.node, timeout_sec=1, executor=executor)
            time.sleep(1)

    def set_use_sim_time_parameter(self, value):
        self.node.set_parameters(
            [Parameter('use_sim_time', Parameter.Type.BOOL, value)])
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        cycle_count = 0
        while rclpy.ok(context=self.context) and cycle_count < 5:
            use_sim_time_param = self.node.get_parameter('use_sim_time')
            cycle_count += 1
            if use_sim_time_param.type_ == Parameter.Type.BOOL:
                break

            rclpy.spin_once(self.node, timeout_sec=1, executor=executor)
            time.sleep(1)
        return use_sim_time_param.value == value

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
        assert self.set_use_sim_time_parameter(True)
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
        node2 = rclpy.create_node('TestTimeSource2', namespace='/rclpy', context=self.context)
        time_source.attach_node(node2)
        node2.destroy_node()
        assert time_source._get_node() == node2
        assert time_source._clock_sub is None

    def test_forwards_jump(self):
        time_source = TimeSource(node=self.node)
        clock = ROSClock()
        time_source.attach_clock(clock)
        assert self.set_use_sim_time_parameter(True)

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
        assert self.set_use_sim_time_parameter(True)

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
        assert self.set_use_sim_time_parameter(True)

        pre_cb = Mock()
        post_cb = Mock()
        threshold = JumpThreshold(min_forward=None, min_backward=None, on_clock_change=True)
        handler = clock.create_jump_callback(
            threshold, pre_callback=pre_cb, post_callback=post_cb)

        assert self.set_use_sim_time_parameter(False)
        pre_cb.assert_called()
        post_cb.assert_called()
        assert post_cb.call_args[0][0].clock_change == ClockChange.ROS_TIME_DEACTIVATED

        pre_cb.reset_mock()
        post_cb.reset_mock()

        assert self.set_use_sim_time_parameter(True)
        pre_cb.assert_called()
        post_cb.assert_called()
        assert post_cb.call_args[0][0].clock_change == ClockChange.ROS_TIME_ACTIVATED
        handler.unregister()

    def test_no_pre_callback(self):
        time_source = TimeSource(node=self.node)
        clock = ROSClock()
        time_source.attach_clock(clock)
        assert self.set_use_sim_time_parameter(True)

        post_cb = Mock()
        threshold = JumpThreshold(min_forward=None, min_backward=None, on_clock_change=True)
        handler = clock.create_jump_callback(
            threshold, pre_callback=None, post_callback=post_cb)

        assert self.set_use_sim_time_parameter(False)
        post_cb.assert_called_once()
        assert post_cb.call_args[0][0].clock_change == ClockChange.ROS_TIME_DEACTIVATED
        handler.unregister()

    def test_no_post_callback(self):
        time_source = TimeSource(node=self.node)
        clock = ROSClock()
        time_source.attach_clock(clock)
        assert self.set_use_sim_time_parameter(True)

        pre_cb = Mock()
        threshold = JumpThreshold(min_forward=None, min_backward=None, on_clock_change=True)
        handler = clock.create_jump_callback(
            threshold, pre_callback=pre_cb, post_callback=None)

        assert self.set_use_sim_time_parameter(False)
        pre_cb.assert_called_once()
        handler.unregister()


if __name__ == '__main__':
    unittest.main()
