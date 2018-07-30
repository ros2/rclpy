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

import builtin_interfaces.msg
import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType
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

    def test_time_source_not_using_sim_time(self):
        time_source = TimeSource(node=self.node)
        clock = Clock(clock_type=ClockType.ROS_TIME)
        time_source.attach_clock(clock)
        self.assertFalse(clock.ros_time_is_active)

        # When not using sim time, ROS time should look like system time
        now = clock.now()
        system_now = Clock(clock_type=ClockType.SYSTEM_TIME).now()
        assert (system_now.nanoseconds - now.nanoseconds) < 1e9

        # Presence of clock publisher should not affect the clock
        self.publish_clock_messages()
        now = clock.now()
        system_now = Clock(clock_type=ClockType.SYSTEM_TIME).now()
        assert (system_now.nanoseconds - now.nanoseconds) < 1e9

    def test_time_source_using_sim_time(self):
        time_source = TimeSource(node=self.node)
        clock = Clock(clock_type=ClockType.ROS_TIME)
        time_source.attach_clock(clock)

        # Check attribute getting/setting
        clock.ros_time_is_active = True
        self.assertTrue(clock.ros_time_is_active)

        # When using sim time, ROS time should look like the messages received on /clock
        self.publish_clock_messages()
        assert clock.now() > Time(seconds=0, clock_type=ClockType.ROS_TIME)
        assert clock.now() <= Time(seconds=5, clock_type=ClockType.ROS_TIME)
