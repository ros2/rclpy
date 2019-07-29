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

from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import ClockType
from rclpy.clock import ROSClock
from rclpy.parameter import Parameter
from rclpy.time import Time
import rosgraph_msgs.msg

CLOCK_TOPIC = '/clock'
USE_SIM_TIME_NAME = 'use_sim_time'


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
        for clock in self._associated_clocks:
            clock._set_ros_time_is_active(enabled)
        if enabled:
            self._subscribe_to_clock_topic()
        else:
            if self._clock_sub is not None and self._node is not None:
                self._node.destroy_subscription(self._clock_sub)
                self._clock_sub = None

    def _subscribe_to_clock_topic(self):
        if self._clock_sub is None and self._node is not None:
            self._clock_sub = self._node.create_subscription(
                rosgraph_msgs.msg.Clock,
                CLOCK_TOPIC,
                self.clock_callback,
                10
            )

    def attach_node(self, node):
        from rclpy.node import Node
        if not isinstance(node, Node):
            raise TypeError('Node must be of type rclpy.node.Node')
        # Remove an existing node.
        if self._node is not None:
            self.detach_node()
        self._node = node

        if not node.has_parameter(USE_SIM_TIME_NAME):
            node.declare_parameter(USE_SIM_TIME_NAME, False)

        use_sim_time_param = node.get_parameter(USE_SIM_TIME_NAME)
        if use_sim_time_param.type_ != Parameter.Type.NOT_SET:
            if use_sim_time_param.type_ == Parameter.Type.BOOL:
                self.ros_time_is_active = use_sim_time_param.value
            else:
                node.get_logger().error(
                    "Invalid type for parameter '{}' {!r} should be bool"
                    .format(USE_SIM_TIME_NAME, use_sim_time_param.type_))
        else:
            node.get_logger().debug(
                "'{}' parameter not set, using wall time by default"
                .format(USE_SIM_TIME_NAME))

        node.set_parameters_callback(self._on_parameter_event)

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

        clock.set_ros_time_override(self._last_time_set)
        clock._set_ros_time_is_active(self.ros_time_is_active)
        self._associated_clocks.append(clock)

    def clock_callback(self, msg):
        # Cache the last message in case a new clock is attached.
        time_from_msg = Time.from_msg(msg.clock)
        self._last_time_set = time_from_msg
        for clock in self._associated_clocks:
            clock.set_ros_time_override(time_from_msg)

    def _on_parameter_event(self, parameter_list):
        for parameter in parameter_list:
            if parameter.name == USE_SIM_TIME_NAME:
                if parameter.type_ == Parameter.Type.BOOL:
                    self.ros_time_is_active = parameter.value
                else:
                    self._node.get_logger().error(
                        '{} parameter set to something besides a bool'
                        .format(USE_SIM_TIME_NAME))
                break

        return SetParametersResult(successful=True)
