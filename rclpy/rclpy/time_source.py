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

import builtin_interfaces.msg
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.time import Time

CLOCK_TOPIC = '/clock'


class TimeSource:

    def __init__(self, *, node=None):
        self._clock_sub = None
        self._node = None
        self._associated_clocks = []
        self._last_time_set = None
        if node is not None:
            self.attach_node(node)

    def attach_node(self, node):
        if not isinstance(node, Node):
            raise TypeError('Node must be of type rclpy.node.Node')
        # Remove an existing node.
        if self._node is not None:
            self.detach_node()

        # Create a subscription to the clock topic using the node.
        self._clock_sub = node.create_subscription(
            builtin_interfaces.msg.Time,
            CLOCK_TOPIC,
            self.clock_callback
        )
        self._node = node

    def detach_node(self):
        # Remove the subscription to the clock topic.
        if self._clock_sub is not None:
            if self._node is None:
                print('Unable to destroy previously created clock subscription')
            else:
                self._node.destroy_subscription(self._clock_sub)
        self._clock_sub = None
        self._node = None

    def attach_clock(self, clock):
        if not isinstance(clock, Clock):
            raise TypeError('Clock must be of type rclpy.clock.Clock')
        if not clock.clock_type == ClockType.ROS_TIME:
            raise ValueError('Only clocks with type ROS_TIME can be attached.')
        if self._last_time_set is not None:
            self._set_clock(self._last_time_set, clock)
        self._associated_clocks.append(clock)

    def clock_callback(self, msg):
        # Cache the last message in case a new clock is attached.
        time_from_msg = Time.from_msg(msg)
        self._last_time_set = time_from_msg
        for clock in self._associated_clocks:
            self._set_clock(time_from_msg, clock)

    def _set_clock(self, time, clock):
        # TODO(dhood): clock jump notifications
        if clock.ros_time_is_active:
            clock.set_ros_time_override(time)
