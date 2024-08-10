# Copyright 2024 Open Source Robotics Foundation, Inc.
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

from __future__ import annotations

from types import TracebackType
from typing import Any, Generic, Literal, Sequence, TypedDict

from rclpy.clock import JumpHandle
from rclpy.clock_type import ClockType
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy,
                       QoSReliabilityPolicy)
from rclpy.subscription import MessageInfo
from rclpy.type_support import MsgT


def rclpy_remove_ros_args(pycli_args: Sequence[str]) -> list[str]:
    """Remove ROS-specific arguments from argument vector."""


def rclpy_get_rmw_implementation_identifier() -> str:
    """Retrieve the identifier for the active RMW implementation."""


class Client:
    pass


class rcl_time_point_t:
    nanoseconds: int
    clock_type: ClockType


class Destroyable:

    def __enter__(self) -> None: ...

    def __exit__(self, exc_type: type[BaseException] | None,
                 exc_val: BaseException | None, exctb: TracebackType | None) -> None: ...

    def destroy_when_not_in_use(self) -> None:
        """Destroy the rcl object as soon as it's not actively being used."""


class Clock(Destroyable):

    def __init__(self, clock_type: int) -> None: ...

    def get_now(self) -> rcl_time_point_t:
        """Value of the clock."""

    def get_ros_time_override_is_enabled(self) -> bool:
        """Return if a clock using ROS time has the ROS time override enabled."""

    def set_ros_time_override_is_enabled(self, enabled: bool) -> None:
        """Set if a clock using ROS time has the ROS time override enabled."""

    def set_ros_time_override(self, time_point: rcl_time_point_t) -> None:
        """Set the ROS time override for a clock using ROS time."""

    def add_clock_callback(self, pyjump_handle: JumpHandle,
                           on_clock_change: bool, min_forward: int,
                           min_backward: int) -> None:
        """Add a time jump callback to a clock."""

    def remove_clock_callback(self, pyjump_handle: JumpHandle) -> None:
        """Remove a time jump callback from a clock."""


class Context(Destroyable):

    def __init__(self, pyargs: list[str], domain_id: int) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def get_domain_id(self) -> int:
        """Retrieve domain id from init_options of context."""

    def ok(self) -> bool:
        """Status of the the client library."""

    def shutdown(self) -> None:
        """Shutdown context."""


class rcl_duration_t:
    nanoseconds: int


class EventHandle(Destroyable):

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""


class GuardCondition(Destroyable):

    def __init__(self, context: Context) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def trigger_guard_condition(self) -> None:
        """Trigger a general purpose guard condition."""


class Service:
    pass


class Subscription(Destroyable, Generic[MsgT]):

    def __init__(self, node: Node, pymsg_type: type[MsgT], topic: str,
                 pyqos_profile: rmw_qos_profile_t) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def take_message(self, pymsg_type: type[MsgT], raw: bool) -> tuple[MsgT, MessageInfo]:
        """Take a message and its metadata from a subscription."""

    def get_logger_name(self) -> str:
        """Get the name of the logger associated with the node of the subscription."""

    def get_topic_name(self) -> str:
        """Return the resolved topic name of a subscription."""

    def get_publisher_count(self) -> int:
        """Count the publishers from a subscription."""


class Node:
    pass


class Publisher(Destroyable, Generic[MsgT]):

    def __init__(self, arg0: Node, arg1: type[MsgT], arg2: str, arg3: rmw_qos_profile_t) -> None:
        """Create _rclpy.Publisher."""

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def get_logger_name(self) -> str:
        """Get the name of the logger associated with the node of the publisher."""

    def get_subscription_count(self) -> int:
        """Count subscribers from a publisher."""

    def get_topic_name(self) -> str:
        """Retrieve the topic name from a Publisher."""

    def publish(self, arg0: MsgT) -> None:
        """Publish a message."""

    def publish_raw(self, arg0: bytes) -> None:
        """Publish a serialized message."""

    def wait_for_all_acked(self, arg0: rcl_duration_t) -> bool:
        """Wait until all published message data is acknowledged."""


class TimeInfoDict(TypedDict):
    expected_call_time: int
    actual_call_time: int


class Timer(Destroyable):

    def __init__(self, clock: Clock, context: Context, period_nsec: int,
                 autostart: bool) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def reset_timer(self) -> None:
        """Reset a timer."""

    def is_timer_ready(self) -> bool:
        """Check if a timer as reached timeout."""

    def call_timer(self) -> None:
        """Call a timer and starts counting again."""

    def call_timer_with_info(self) -> TimeInfoDict:
        """Call a timer and starts counting again, retrieves actual and expected call time."""

    def change_timer_period(self, period_nsec: int) -> None:
        """Set the period of a timer."""

    def time_until_next_call(self) -> int | None:
        """Get the remaining time before timer is ready."""

    def time_since_last_call(self) -> int:
        """Get the elapsed time since last timer call."""

    def get_timer_period(self) -> int:
        """Get the period of a timer."""

    def cancel_timer(self) -> None:
        """Cancel a timer."""

    def is_timer_canceled(self) -> bool:
        """Check if a timer is canceled."""


PredefinedQosProfileTNames = Literal['qos_profile_sensor_data', 'qos_profile_default',
                                     'qos_profile_system_default', 'qos_profile_services_default',
                                     'qos_profile_unknown', 'qos_profile_parameters',
                                     'qos_profile_parameter_events', 'qos_profile_best_available']


class rmw_qos_profile_dict(TypedDict):
    qos_history: QoSHistoryPolicy | int
    qos_depth: int
    qos_reliability: QoSReliabilityPolicy | int
    qos_durability: QoSDurabilityPolicy | int
    pyqos_lifespan: rcl_duration_t
    pyqos_deadline: rcl_duration_t
    qos_liveliness: QoSLivelinessPolicy | int
    pyqos_liveliness_lease_duration: rcl_duration_t
    avoid_ros_namespace_conventions: bool


class rmw_qos_profile_t:

    def __init__(
        self,
        qos_history: int,
        qos_depth: int,
        qos_reliability: int,
        qos_durability: int,
        pyqos_lifespan: rcl_duration_t,
        pyqos_deadline: rcl_duration_t,
        qos_liveliness: int,
        pyqos_liveliness_lease_duration: rcl_duration_t,
        avoid_ros_namespace_conventions: bool
    ) -> None: ...

    def to_dict(self) -> rmw_qos_profile_dict: ...

    @staticmethod
    def predefined(qos_profile_name: PredefinedQosProfileTNames) -> rmw_qos_profile_t: ...


IsReadyValues = Literal['subscription', 'client', 'service', 'timer', 'guard_condition', 'event']
GetReadyEntityValues = Literal['subscription', 'client', 'service', 'timer', 'guard_condition']


class WaitSet(Destroyable):

    def __init__(self, number_of_subscriptions: int, number_of_guard_conditions: int,
                 number_of_timers: int, number_of_clients: int, number_of_services: int,
                 number_of_events: int, context: Context) -> None:
        """Construct a WaitSet."""

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def clear_entities(self) -> None:
        """Clear all the pointers in the wait set."""

    def add_service(self, service: Service) -> int:
        """Add a service to the wait set structure."""

    def add_subscription(self, subscription: Subscription[Any]) -> int:
        """Add a subcription to the wait set structure."""

    def add_client(self, client: Client) -> int:
        """Add a client to the wait set structure."""

    def add_guard_condition(self, guard_condition: GuardCondition) -> int:
        """Add a guard condition to the wait set structure."""

    def add_timer(self, timer: Timer) -> int:
        """Add a timer to the wait set structure."""

    def add_event(self, event: EventHandle) -> int:
        """Add an event to the wait set structure."""

    def is_ready(self, entity_type: IsReadyValues, index: int) -> bool:
        """Check if an entity in the wait set is ready by its index."""

    def get_ready_entities(self, entity_type: GetReadyEntityValues) -> list[int]:
        """Get list of entities ready by entity type."""

    def wait(self, timeout: int) -> None:
        """Wait until timeout is reached or event happened."""
