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

from typing import List, Optional, Set, TYPE_CHECKING
import weakref

from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import ROSClock
from rclpy.clock_type import ClockType
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
import rosgraph_msgs.msg

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.subscription import Subscription

CLOCK_TOPIC = '/clock'
USE_SIM_TIME_NAME = 'use_sim_time'


class TimeSource:

    def __init__(self, *, node: Optional['Node'] = None):
        self._clock_sub: Optional['Subscription'] = None
        self._node_weak_ref: Optional[weakref.ReferenceType['Node']] = None
        self._associated_clocks: Set[ROSClock] = set()
        # Zero time is a special value that means time is uninitialzied
        self._last_time_set = Time(clock_type=ClockType.ROS_TIME)
        self._ros_time_is_active = False
        if node is not None:
            self.attach_node(node)

    @property
    def ros_time_is_active(self) -> bool:
        return self._ros_time_is_active

    @ros_time_is_active.setter
    def ros_time_is_active(self, enabled: bool) -> None:
        if self._ros_time_is_active == enabled:
            return
        self._ros_time_is_active = enabled
        for clock in self._associated_clocks:
            clock._set_ros_time_is_active(enabled)
        if enabled:
            self._subscribe_to_clock_topic()
        else:
            if self._clock_sub is not None:
                node = self._get_node()
                if node is not None:
                    node.destroy_subscription(self._clock_sub)
                    self._clock_sub = None

    def _subscribe_to_clock_topic(self) -> None:
        if self._clock_sub is None:
            node = self._get_node()
            if node is not None:
                self._clock_sub = node.create_subscription(
                    rosgraph_msgs.msg.Clock,
                    CLOCK_TOPIC,
                    self.clock_callback,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
                )

    def attach_node(self, node: 'Node') -> None:
        from rclpy.node import Node
        if not isinstance(node, Node):
            raise TypeError('Node must be of type rclpy.node.Node')
        # Remove an existing node.
        if self._node_weak_ref is not None:
            self.detach_node()
        self._node_weak_ref = weakref.ref(node)

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

        node.add_on_set_parameters_callback(self._on_parameter_event)

    def detach_node(self) -> None:
        # Remove the subscription to the clock topic.
        if self._clock_sub is not None:
            node = self._get_node()
            if node is None:
                raise RuntimeError('Unable to destroy previously created clock subscription')
            node.destroy_subscription(self._clock_sub)
        self._clock_sub = None
        self._node_weak_ref = None

    def attach_clock(self, clock: ROSClock) -> None:
        if not isinstance(clock, ROSClock):
            raise ValueError('Only clocks with type ROS_TIME can be attached.')

        clock.set_ros_time_override(self._last_time_set)
        clock._set_ros_time_is_active(self.ros_time_is_active)
        self._associated_clocks.add(clock)

    def clock_callback(self, msg: rosgraph_msgs.msg.Clock) -> None:
        # Cache the last message in case a new clock is attached.
        time_from_msg = Time.from_msg(msg.clock)
        self._last_time_set = time_from_msg
        for clock in self._associated_clocks:
            clock.set_ros_time_override(time_from_msg)

    def _on_parameter_event(self, parameter_list: List[Parameter]) -> SetParametersResult:
        successful = True
        reason = ''

        for parameter in parameter_list:
            if parameter.name == USE_SIM_TIME_NAME:
                if parameter.type_ == Parameter.Type.BOOL:
                    self.ros_time_is_active = parameter.value
                else:
                    successful = False
                    reason = '{} parameter set to something besides a bool'.format(
                        USE_SIM_TIME_NAME)

                    node = self._get_node()
                    if node:
                        node.get_logger().error(reason)
                break

        return SetParametersResult(successful=successful, reason=reason)

    def _get_node(self) -> Optional['Node']:
        if self._node_weak_ref is not None:
            return self._node_weak_ref()
        return None
