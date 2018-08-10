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
from rclpy.clock import ClockChange
from rclpy.clock import ClockType
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.time import Time

CLOCK_TOPIC = '/clock'


class TimeJump:

    def __init__(self, clock_change, delta):
        if not isinstance(clock_change, ClockChange):
            raise TypeError('clock_change must be an instance of rclpy.clock.ClockChange')
        # Access through read only properties because same instance is given to all clock callbacks
        self._clock_change = clock_change
        self._delta = delta

    @property
    def clock_change(self):
        return self._clock_change

    @property
    def delta(self):
        return self._delta


class TimeSource:

    def __init__(self, *, node=None):
        self._clock_sub = None
        self._node = None
        self._associated_clocks = []
        # Zero time is a special value that means time is uninitialzied
        self._last_time_set = Time(clock_type=ClockType.ROS_TIME)
        self._ros_time_is_active = False
        if node is not None:
            self.attach_node(node)

    @property
    def ros_time_is_active(self):
        return self._ros_time_is_active

    @ros_time_is_active.setter
    def ros_time_is_active(self, enabled):
        if self._ros_time_is_active == enabled:
            return
        self._ros_time_is_active = enabled
        clock_change = ClockChange.ROS_TIME_DEACTIVATED
        if enabled:
            clock_change = ClockChange.ROS_TIME_ACTIVATED
            self._subscribe_to_clock_topic()
        for clock in self._associated_clocks:
            # Set time and trigger any time jump callbacks on clock change
            self._set_clock(self._last_time_set, clock, TimeJump(clock_change, Duration()))

    def _subscribe_to_clock_topic(self):
        if self._clock_sub is None and self._node is not None:
            self._clock_sub = self._node.create_subscription(
                builtin_interfaces.msg.Time,
                CLOCK_TOPIC,
                self.clock_callback
            )

    def attach_node(self, node):
        from rclpy.node import Node
        if not isinstance(node, Node):
            raise TypeError('Node must be of type rclpy.node.Node')
        # Remove an existing node.
        if self._node is not None:
            self.detach_node()
        self._node = node
        if self.ros_time_is_active:
            self._subscribe_to_clock_topic()

    def detach_node(self):
        # Remove the subscription to the clock topic.
        if self._clock_sub is not None:
            if self._node is None:
                raise RuntimeError('Unable to destroy previously created clock subscription')
            self._node.destroy_subscription(self._clock_sub)
        self._clock_sub = None
        self._node = None

    def attach_clock(self, clock):
        if not isinstance(clock, ROSClock):
            raise ValueError('Only clocks with type ROS_TIME can be attached.')

        jump_info = None
        if self.ros_time_is_active:
            jump_info = TimeJump(ClockChange.ROS_TIME_NO_CHANGE, Duration())
        else:
            jump_info = TimeJump(ClockChange.SYSTEM_TIME_NO_CHANGE, Duration())
        self._set_clock(self._last_time_set, clock, jump_info)

        self._associated_clocks.append(clock)

    def clock_callback(self, msg):
        time_from_msg = Time.from_msg(msg)
        jump_info = TimeJump(ClockChange.ROS_TIME_NO_CHANGE, time_from_msg - self._last_time_set)
        # Cache the last message in case a new clock is attached.
        self._last_time_set = time_from_msg
        # Only notify clocks of new time if ROS time is active
        if self.ros_time_is_active:
            for clock in self._associated_clocks:
                self._set_clock(time_from_msg, clock, jump_info)

    def _set_clock(self, time, clock, jump_info):
        # Must not call pre jump handlers in executor because they are required to complete
        # before the clock time changes
        jump_handlers = tuple(clock.get_triggered_callback_handlers(jump_info))
        for jump_handler in jump_handlers:
            if jump_handler.pre_callback is not None:
                jump_handler.pre_callback(jump_info)
        clock._set_ros_time_is_active(self.ros_time_is_active)
        clock.set_ros_time_override(time)
        for jump_handler in jump_handlers:
            if jump_handler.post_callback is not None:
                jump_handler.post_callback(jump_info)
