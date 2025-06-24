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

import builtins
from enum import IntEnum
from types import TracebackType
from typing import (Any, Callable, Coroutine, Final, Generic, Literal, Optional, overload,
                    Sequence, TypeAlias, TypedDict, TypeVar)


from action_msgs.msg import GoalInfo
from action_msgs.msg._goal_status_array import GoalStatusArray
from action_msgs.srv._cancel_goal import CancelGoal
from rclpy.clock import JumpHandle
from rclpy.context import Context as RCLPyContext
from rclpy.duration import Duration
from rclpy.impl import service_introspection as service_introspection
from rclpy.node import Node as RCLPyNode
from rclpy.parameter import Parameter
from rclpy.subscription import MessageInfo
from rclpy.task import Future
from rclpy.task import Task
from rclpy.type_support import (Action, FeedbackMessage, FeedbackT, GetResultServiceRequest,
                                GetResultServiceResponse, GoalT, Msg, MsgT, ResultT,
                                SendGoalServiceRequest, SendGoalServiceResponse, Srv, SrvRequestT,
                                SrvResponseT)
from type_description_interfaces.srv import GetTypeDescription

T = TypeVar('T')

# All things are defined in same order as defined in _rclpy_pybind11.cpp


class Destroyable:

    def __enter__(self) -> None: ...

    def __exit__(self, exc_type: type[BaseException] | None,
                 exc_val: BaseException | None, exctb: TracebackType | None) -> None: ...

    def destroy_when_not_in_use(self) -> None:
        """Destroy the rcl object as soon as it's not actively being used."""


class ClockType(IntEnum):
    UNINITIALIZED = ...
    ROS_TIME = ...
    SYSTEM_TIME = ...
    STEADY_TIME = ...


class GoalEvent(IntEnum):
    EXECUTE = ...
    CANCEL_GOAL = ...
    SUCCEED = ...
    ABORT = ...
    CANCELED = ...


RCL_DEFAULT_DOMAIN_ID: Final[int] = ...
RMW_DURATION_INFINITE: Final[int] = ...
RMW_QOS_DEADLINE_BEST_AVAILABLE: Final[int] = ...
RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE: Final[int] = ...


class ClockChange(IntEnum):
    ROS_TIME_NO_CHANGE = ...
    """ROS time is active and will continue to be active"."""
    ROS_TIME_ACTIVATED = ...
    """ROS time is being activated."""
    ROS_TIME_DEACTIVATED = ...
    """ROS TIME is being deactivated, the clock will report system time after the jump."""
    SYSTEM_TIME_NO_CHANGE = ...
    """ROS time is inactive and the clock will keep reporting system time."""


class QoSCompatibility(IntEnum):
    OK = ...
    WARNING = ...
    ERROR = ...


class _rmw_qos_compatibility_type_e(IntEnum):
    RMW_QOS_COMPATIBILITY_OK = ...
    RMW_QOS_COMPATIBILITY_WARNING = ...
    RMW_QOS_COMPATIBILITY_ERROR = ...


_rmw_qos_compatibility_type_t: TypeAlias = _rmw_qos_compatibility_type_e


class QoSCheckCompatibleResult:
    """Result type for checking QoS compatibility with result."""

    def __init__(self) -> None: ...

    @property
    def compatibility(self) -> _rmw_qos_compatibility_type_t: ...

    @property
    def reason(self) -> str: ...


class RCUtilsError(RuntimeError):

    def __init__(self, error_text: str): ...


class RMWError(RuntimeError):

    def __init__(self, error_text: str): ...


class RCLError(RuntimeError):

    def __init__(self, error_text: str): ...


class RCLInvalidROSArgsError(RCLError):
    pass


class UnknownROSArgsError(RuntimeError):
    pass


class NodeNameNonExistentError(RCLError):
    pass


class UnsupportedEventTypeError(RCLError):
    pass


class NotImplementedError(builtins.NotImplementedError):  # noqa: A001
    pass


class InvalidHandle(RuntimeError):
    pass


# Service Introspection imported above


class Client(Destroyable, Generic[SrvRequestT, SrvResponseT]):

    def __init__(self, node: Node, srv_type: type[Srv],
                 srv_name: str, pyqos_profile: rmw_qos_profile_t) -> None: ...

    @property
    def service_name(self) -> str:
        """Get the name of the service."""

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def send_request(self, pyrequest: SrvRequestT) -> int:
        """Send a request."""

    def service_server_is_available(self) -> bool:
        """Return true if the service server is available."""

    def take_response(
        self, pyresponse_type: type[SrvResponseT]
    ) -> tuple[rmw_service_info_t, SrvResponseT] | tuple[None, None]:
        """Take a received response from an earlier request."""

    def configure_introspection(
        self,
        clock: Clock,
        pyqos_service_event_pub: rmw_qos_profile_t,
        introspection_state: service_introspection.ServiceIntrospectionState
    ) -> None:
        """Configure whether introspection is enabled."""

    def get_logger_name(self) -> str:
        """Get the name of the logger associated with the node of the client."""


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

    def __init__(self, nanoseconds: int) -> None: ...

    @property
    def nanoseconds(self) -> int: ...


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


class Service(Destroyable, Generic[SrvRequestT, SrvResponseT]):

    def __init__(self, node: Node, pysrv_type: type[Srv],
                 name: str, pyqos_profile: rmw_qos_profile_t) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    @property
    def name(self) -> str:
        """Get the name of the service."""

    @property
    def qos(self) -> _rmw_qos_profile_dict:
        """Get the qos profile of the service."""

    def service_send_response(self, pyresponse: SrvResponseT, header: rmw_request_id_t) -> None:
        """Send a response."""

    def service_take_request(
        self,
        pyrequest_type: type[SrvRequestT]
    ) -> tuple[rmw_service_info_t, SrvRequestT] | tuple[None, None]:
        """Take a request from a given service."""

    def configure_introspection(
        self, clock: Clock,
        pyqos_service_event_pub: rmw_qos_profile_t,
        introspection_state: service_introspection.ServiceIntrospectionState
    ) -> None:
        """Configure whether introspection is enabled."""

    def get_logger_name(self) -> str:
        """Get the name of the logger associated with the node of the service."""


class TypeDescriptionService(Destroyable):

    def __init__(self, handle: Node) -> None: ...

    @property
    def impl(self) -> Service[GetTypeDescription.Request, GetTypeDescription.Response]:
        """Get the rcl service wrapper capsule."""

    def handle_request(
        self, pyrequest: GetTypeDescription.Request,
        pyresponse_type: type[GetTypeDescription.Response],
        node: Node
    ) -> GetTypeDescription.Response:
        """Handle an incoming request by calling RCL implementation."""


class rmw_service_info_t:

    @property
    def source_timestamp(self) -> int: ...

    @property
    def received_timestamp(self) -> int: ...

    @property
    def request_id(self) -> rmw_request_id_t: ...


class rmw_request_id_t:

    @property
    def sequence_number(self) -> int: ...


def rclpy_qos_check_compatible(publisher_qos_profile: rmw_qos_profile_t,
                               subscription_qos_profile: rmw_qos_profile_t
                               ) -> QoSCheckCompatibleResult:
    """Check if two QoS profiles are compatible."""


class ActionClient(Generic[GoalT, ResultT, FeedbackT], Destroyable):

    def __init__(
            self,
            node: Node,
            pyaction_type: type[Action],
            action_name: str,
            goal_service_qos: rmw_qos_profile_t,
            result_service_qos: rmw_qos_profile_t,
            cancel_service_qos: rmw_qos_profile_t,
            feedback_service_qos: rmw_qos_profile_t,
            status_topci_qos: rmw_qos_profile_t
        ) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def take_goal_response(self, pymsg_type: type[SendGoalServiceResponse]
                           ) -> tuple[int, SendGoalServiceResponse] | tuple[None, None]:
        """Take an action goal response."""

    def send_result_request(self, pyrequest: GetResultServiceRequest) -> int:
        """Send an action result requst."""

    def take_cancel_response(self, pymsg_type: type[CancelGoal.Response]
                             ) -> tuple[int, CancelGoal.Response] | tuple[None, None]:
        """Take an action cancel response."""

    def take_feedback(self, pymsg_type: type[FeedbackMessage[FeedbackT]]
                      ) -> FeedbackMessage[FeedbackT] | None:
        """Take a feedback message from a given action client."""

    def send_cancel_request(self, pyrequest: CancelGoal.Request) -> int:
        """Send an action cancel request."""

    def send_goal_request(self, pyrequest: SendGoalServiceRequest[GoalT]) -> int:
        """Send an action goal request."""

    def take_result_response(
        self,
        pymsg_type: type[GetResultServiceResponse[ResultT]]
    ) -> tuple[int, GetResultServiceResponse[ResultT]] | tuple[None, None]:
        """Take an action result response."""

    def get_num_entities(self) -> tuple[int, int, int, int, int]:
        """Get the number of wait set entities that make up an action entity."""

    def is_action_server_available(self) -> bool:
        """Check if an action server is available for the given action client."""

    def add_to_waitset(self, wait_set: WaitSet) -> None:
        """Add an action entity to a wait set."""

    def is_ready(self, wait_set: WaitSet) -> tuple[bool, bool, bool, bool, bool]:
        """Check if an action entity has any ready wait set entities."""

    def take_status(self, pymsg_type: type[GoalStatusArray]) -> GoalStatusArray | None:
        """Take an action status response."""

    def configure_introspection(
        self,
        clock: Clock,
        pyqos_service_event_pub: Optional[rmw_qos_profile_t],
        introspection_state: service_introspection.ServiceIntrospectionState
    ) -> None:
        """Configure whether internal client introspection is enabled."""


class ActionGoalHandle(Destroyable):

    def __init__(self, action_server: ActionServer[Any, Any, Any],
                 pygoal_info_msg: GoalInfo) -> None:
        ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def get_status(self) -> GoalEvent:
        """Get the status of a goal."""

    def update_goal_state(self, event: GoalEvent) -> None:
        """Update a goal state."""

    def is_active(self) -> bool:
        """Check if a goal is active."""


class ActionServer(Generic[GoalT, ResultT, FeedbackT], Destroyable):

    def __init__(
        self,
        node: Node,
        rclpy_clock: Clock,
        pyaction_type: type[Action],
        action_name: str,
        goal_service_qos: rmw_qos_profile_t,
        result_service_qos: rmw_qos_profile_t,
        cancel_service_qos: rmw_qos_profile_t,
        feedback_topic_qos: rmw_qos_profile_t,
        status_topic_qos: rmw_qos_profile_t,
        result_timeout: float
    ) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def take_goal_request(
        self,
        pymsg_type: type[SendGoalServiceRequest[GoalT]]
    ) -> tuple[rmw_request_id_t, SendGoalServiceRequest[GoalT]] | tuple[None, None]:
        """Take an action goal request."""

    def send_goal_response(
        self,
        header: rmw_request_id_t,
        pyresponse: SendGoalServiceResponse
    ) -> None:
        """Send an action goal response."""

    def send_result_response(
        self,
        header: rmw_request_id_t,
        pyresponse: GetResultServiceResponse[ResultT]
    ) -> None:
        """Send an action result response."""

    def take_cancel_request(
        self,
        pymsg_type: type[CancelGoal.Request]
    ) -> tuple[rmw_request_id_t, CancelGoal.Request] | tuple[None, None]:
        """Take an action cancel request."""

    def take_result_request(
        self,
        pymsg_type: type[GetResultServiceRequest]
    ) -> tuple[rmw_request_id_t, GetResultServiceRequest] | tuple[None, None]:
        """Take an action result request."""

    def send_cancel_response(
        self,
        header: rmw_request_id_t,
        pyresponse: int
    ) -> None:
        """Send an action cancel response."""

    def publish_feedback(
        self,
        pymsg: FeedbackMessage[FeedbackT]
    ) -> None:
        """Publish a feedback message from a given action server."""

    def publish_status(self) -> None:
        """Publish a status message from a given action server."""

    def notify_goal_done(self) -> None:
        """Notify goal is done."""

    def goal_exists(self, pygoal_info: GoalInfo) -> bool:
        """Check is a goal exists in the server."""

    def process_cancel_request(
        self,
        pycancel_request: CancelGoal.Request,
        pycancel_response_tpye: type[CancelGoal.Response]
    ) -> CancelGoal.Response:
        """Process a cancel request."""

    def expire_goals(self, max_num_goals: int) -> tuple[GoalInfo, ...]:
        """Expired goals."""

    def get_num_entities(self) -> tuple[int, int, int, int, int]:
        """Get the number of wait set entities that make up an action entity."""

    def is_ready(self, wait_set: WaitSet) -> tuple[bool, bool, bool, bool]:
        """Check if an action entity has any ready wait set entities."""

    def add_to_waitset(self, wait_set: WaitSet) -> None:
        """Add an action entity to a wait set."""

    def configure_introspection(
        self,
        clock: Clock,
        pyqos_service_pub: Optional[rmw_qos_profile_t],
        introspection_state: service_introspection.ServiceIntrospectionState
    ) -> None:
        """Configure whether internal service introspection is enabled."""


def rclpy_action_get_rmw_qos_profile(rmw_profile: str) -> _rmw_qos_profile_dict:
    """Get an action RMW QoS profile."""


class GuardCondition(Destroyable):

    def __init__(self, context: Context) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def trigger_guard_condition(self) -> None:
        """Trigger a general purpose guard condition."""


class _TimeInfoDict(TypedDict):
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

    def call_timer_with_info(self) -> _TimeInfoDict:
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


class rcl_time_point_t:

    def __init__(self, nanoseconds: int, clock_type: int) -> None: ...

    @property
    def nanoseconds(self) -> int: ...

    @property
    def clock_type(self) -> ClockType: ...


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


_IsReadyValues = Literal['subscription', 'client', 'service', 'timer', 'guard_condition', 'event']
_GetReadyEntityValues = Literal['subscription', 'client', 'service', 'timer', 'guard_condition']


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

    def add_service(self, service: Service[Any, Any]) -> int:
        """Add a service to the wait set structure."""

    def add_subscription(self, subscription: Subscription[Any]) -> int:
        """Add a subcription to the wait set structure."""

    def add_client(self, client: Client[Any, Any]) -> int:
        """Add a client to the wait set structure."""

    def add_guard_condition(self, guard_condition: GuardCondition) -> int:
        """Add a guard condition to the wait set structure."""

    def add_timer(self, timer: Timer) -> int:
        """Add a timer to the wait set structure."""

    def add_event(self, event: EventHandle[Any]) -> int:
        """Add an event to the wait set structure."""

    def is_ready(self, entity_type: _IsReadyValues, index: int) -> bool:
        """Check if an entity in the wait set is ready by its index."""

    def get_ready_entities(self, entity_type: _GetReadyEntityValues) -> list[int]:
        """Get list of entities ready by entity type."""

    def wait(self, timeout: int) -> None:
        """Wait until timeout is reached or event happened."""


def rclpy_expand_topic_name(topic: str, node_name: str, node_namespace: str) -> str:
    """Expand a topic name."""


def rclpy_remap_topic_name(node: Node, topic_name: str) -> str:
    """Remap a topic name."""


def rclpy_get_validation_error_for_topic_name(topic_name: str) -> tuple[str, int] | None:
    """Get the error message and invalid index of a topic name or None if valid."""


def rclpy_get_validation_error_for_full_topic_name(topic_name: str) -> tuple[str, int] | None:
    """Get the error message and invalid index of a full topic name or None if valid."""


def rclpy_get_validation_error_for_namespace(namespace_: str) -> tuple[str, int] | None:
    """Get the error message and invalid index of a namespace or None if valid."""


def rclpy_get_validation_error_for_node_name(namespace_: str) -> tuple[str, int] | None:
    """Get the error message and invalid index of a node name or None if valid."""


def rclpy_resolve_name(node: Node, topic_name: str, only_expand: bool, is_service: bool) -> str:
    """Expand and remap a topic or service name."""


def rclpy_get_topic_names_and_types(node: Node, no_demangle: bool) -> list[tuple[str, list[str]]]:
    """Get all topic names and types in the ROS graph."""


def rclpy_get_publisher_names_and_types_by_node(node: Node, no_demangle: bool, node_name: str,
                                                node_namespace: str
                                                ) -> list[tuple[str, list[str]]]:
    """Get topic names and types for which a remote node has publishers."""


def rclpy_get_subscriber_names_and_types_by_node(node: Node, no_demangle: bool, node_name: str,
                                                 node_namespace: str
                                                 ) -> list[tuple[str, list[str]]]:
    """Get topic names and types for which a remote node has subscribers."""


class _TypeHashDict(TypedDict):
    version: int
    value: bytes


class _TopicEndpointInfoDict(TypedDict):
    node_name: str
    node_namespace: str
    topic_type: str
    topic_type_hash: _TypeHashDict
    endpoint_type: int
    endpoint_gid: list[int]
    qos_profile: _rmw_qos_profile_dict


def rclpy_get_publishers_info_by_topic(node: Node, topic_name: str, no_mangle: bool
                                       ) -> list[_TopicEndpointInfoDict]:
    """Get publishers info for a topic."""


def rclpy_get_subscriptions_info_by_topic(node: Node, topic_name: str, no_mangle: bool
                                          ) -> list[_TopicEndpointInfoDict]:
    """Get subscriptions info for a topic."""


def rclpy_get_service_names_and_types(node: Node) -> list[tuple[str, list[str]]]:
    """Get all service names and types in the ROS graph."""


def rclpy_get_service_names_and_types_by_node(node: Node, node_name: str, node_namespace: str
                                              ) -> list[tuple[str, list[str]]]:
    """Get all service names and types in the ROS graph."""


def rclpy_get_client_names_and_types_by_node(node: Node, node_name: str, node_namespace: str
                                             ) -> list[tuple[str, list[str]]]:
    """Get service names and types for which a remote node has servers."""


def rclpy_serialize(pymsg: Msg, py_msg_type: type[Msg]) -> bytes:
    """Serialize a ROS message."""


def rclpy_deserialize(pybuffer: bytes, pymsg_type: type[MsgT]) -> MsgT:
    """Deserialize a ROS message."""


class Node(Destroyable):

    def __init__(self, node_name: str, namespace_: str, context: Context,
                 pycli_args: list[str] | None, use_global_arguments: bool,
                 enable: bool, rosout_qos_profile: rmw_qos_profile_t) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def get_fully_qualified_name(self) -> str:
        """Get the fully qualified name of the node."""

    def logger_name(self) -> str:
        """Get the name of the logger associated with a node."""

    def get_node_name(self) -> str:
        """Get the name of a node."""

    def get_namespace(self) -> str:
        """Get the namespace of a node."""

    def get_count_publishers(self, topic_name: str) -> int:
        """Return the count of all the publishers known for that topic in the entire ROS graph."""

    def get_count_subscribers(self, topic_name: str) -> int:
        """Return the count of all the subscribers known for that topic in the entire ROS graph."""

    def get_count_clients(self, service_name: str) -> int:
        """Return the count of all the clients known for that service in the entire ROS graph."""

    def get_count_services(self, service_name: str) -> int:
        """Return the count of all the servers known for that service in the entire ROS graph."""

    def get_node_names_and_namespaces(self) -> list[tuple[str, str]]:
        """Get the list of nodes discovered by the provided node."""

    def get_node_names_and_namespaces_with_enclaves(self) -> list[tuple[str, str, str]]:
        """Get the list of nodes discovered by the provided node, with their enclaves."""

    def get_action_client_names_and_types_by_node(self, remote_node_name: str,
                                                  remote_node_namespace: str) -> list[tuple[str,
                                                                                      list[str]]]:
        """Get action client names and types by node."""

    def get_action_server_names_and_types_by_node(self, remote_node_name: str,
                                                  remote_node_namespace: str) -> list[tuple[str,
                                                                                      list[str]]]:
        """Get action server names and types by node."""

    def get_action_names_and_types(self) -> list[tuple[str, list[str]]]:
        """Get action names and types."""

    def get_parameters(self, pyparamter_cls: type[Parameter[Any]]) -> dict[str, Parameter[Any]]:
        """Get a list of parameters for the current node."""


class _rmw_qos_incompatible_event_status_s:
    total_count: int
    total_count_change: int
    last_policy_kind: rmw_qos_policy_kind_t


_rmw_qos_incompatible_event_status_t: TypeAlias = _rmw_qos_incompatible_event_status_s
_rmw_offered_qos_incompatible_event_status_t: TypeAlias = _rmw_qos_incompatible_event_status_t


class EventHandle(Destroyable, Generic[T]):

    @overload
    def __init__(
        self,
        subcription: Subscription[Any],
        event_type: rcl_subscription_event_type_t
    ) -> None: ...

    @overload
    def __init__(
        self,
        subcription: Publisher[Any],
        event_type: rcl_publisher_event_type_t
    ) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def take_event(self) -> T | None:
        """Get pending data from a ready event."""


class rcl_subscription_event_type_t(IntEnum):
    RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED = ...
    RCL_SUBSCRIPTION_LIVELINESS_CHANGED = ...
    RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS = ...
    RCL_SUBSCRIPTION_MESSAGE_LOST = ...
    RCL_SUBSCRIPTION_INCOMPATIBLE_TYPE = ...
    RCL_SUBSCRIPTION_MATCHED = ...


class rcl_publisher_event_type_t(IntEnum):
    RCL_PUBLISHER_OFFERED_DEADLINE_MISSED = ...
    RCL_PUBLISHER_LIVELINESS_LOST = ...
    RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS = ...
    RCL_PUBLISHER_INCOMPATIBLE_TYPE = ...
    RCL_PUBLISHER_MATCHED = ...


class rmw_requested_deadline_missed_status_t:

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...


class rmw_liveliness_changed_status_t:

    @property
    def alive_count(self) -> int: ...

    @property
    def not_alive_count(self) -> int: ...

    @property
    def alive_count_change(self) -> int: ...

    @property
    def not_alive_count_change(self) -> int: ...


class rmw_message_lost_status_t:

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...


class rmw_requested_qos_incompatible_event_status_t:

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...

    @property
    def last_policy_kind(self) -> rmw_qos_policy_kind_t: ...


class rmw_offered_deadline_missed_status_t:

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...


class rmw_liveliness_lost_status_t:

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...


class rmw_matched_status_t:

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...

    @property
    def current_count(self) -> int: ...

    @property
    def current_count_change(self) -> int: ...


class rmw_qos_policy_kind_t(IntEnum):
    RMW_QOS_POLICY_INVALID = ...
    RMW_QOS_POLICY_DURABILITY = ...
    RMW_QOS_POLICY_DEADLINE = ...
    RMW_QOS_POLICY_LIVELINESS = ...
    RMW_QOS_POLICY_RELIABILITY = ...
    RMW_QOS_POLICY_HISTORY = ...
    RMW_QOS_POLICY_LIFESPAN = ...
    RMW_QOS_POLICY_DEPTH = ...
    RMW_QOS_POLICY_LIVELINESS_LEASE_DURATION = ...
    RMW_QOS_POLICY_AVOID_ROS_NAMESPACE_CONVENTIONS = ...


class rmw_incompatible_type_status_t:

    @property
    def total_count_change(self) -> int: ...


def rclpy_get_rmw_implementation_identifier() -> str:
    """Retrieve the identifier for the active RMW implementation."""


def rclpy_assert_liveliness(publisher: Publisher[Any]) -> None:
    """Assert the liveliness of an entity."""


def rclpy_remove_ros_args(pycli_args: Sequence[str]) -> list[str]:
    """Remove ROS-specific arguments from argument vector."""


_PredefinedQosProfileTNames = Literal['qos_profile_sensor_data', 'qos_profile_default',
                                      'qos_profile_system_default', 'qos_profile_services_default',
                                      'qos_profile_unknown', 'qos_profile_parameters',
                                      'qos_profile_parameter_events', 'qos_profile_best_available',
                                      'qos_profile_rosout_default']


class _rmw_qos_profile_dict(TypedDict):
    depth: int
    history: int
    reliability: int
    durability: int
    lifespan: Duration
    deadline: Duration
    liveliness: int
    liveliness_lease_duration: Duration
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

    def to_dict(self) -> _rmw_qos_profile_dict: ...

    @staticmethod
    def predefined(qos_profile_name: _PredefinedQosProfileTNames) -> rmw_qos_profile_t: ...


def rclpy_logging_fini() -> None:
    """Finalize RCL logging."""


def rclpy_logging_configure(context: Context) -> None:
    """Initialize RCL logging."""


class RCUTILS_LOG_SEVERITY(IntEnum):
    RCUTILS_LOG_SEVERITY_UNSET = ...
    RCUTILS_LOG_SEVERITY_DEBUG = ...
    RCUTILS_LOG_SEVERITY_INFO = ...
    RCUTILS_LOG_SEVERITY_WARN = ...
    RCUTILS_LOG_SEVERITY_ERROR = ...
    RCUTILS_LOG_SEVERITY_FATAL = ...


def rclpy_logging_get_separator_string() -> str: ...


def rclpy_logging_initialize() -> None: ...


def rclpy_logging_shutdown() -> None: ...


def rclpy_logging_set_logger_level(name: str, level: int,
                                   detailed_error: bool = False) -> None: ...


def rclpy_logging_get_logger_effective_level(name: str) -> int: ...


def rclpy_logging_logger_is_enabled_for(name: str, severity: int) -> bool: ...


def rclpy_logging_rcutils_log(severity: int, name: str, message: str, function_name: str,
                              file_name: str, line_number: int) -> None: ...


def rclpy_logging_severity_level_from_string(log_level: str) -> int: ...


def rclpy_logging_get_logging_directory() -> str: ...


def rclpy_logging_rosout_add_sublogger(logger_name: str, sublogger_name: str) -> bool: ...


def rclpy_logging_rosout_remove_sublogger(logger_name: str, sublogger_name: str) -> None: ...


def rclpy_logging_get_logger_level(name: str) -> int: ...


def register_sigint_guard_condition(guard_condition: GuardCondition) -> None:
    """Register a guard condition to be called on SIGINT."""


def unregister_sigint_guard_condition(guard_condition: GuardCondition) -> None:
    """Stop triggering a guard condition when SIGINT occurs."""


def install_signal_handlers(options: SignalHandlerOptions) -> None:
    """Install rclpy signal handlers."""


def get_current_signal_handlers_options() -> SignalHandlerOptions:
    """Get currently installed signal handler options."""


def uninstall_signal_handlers() -> None:
    """Uninstall rclpy signal handlers."""


class SignalHandlerOptions(IntEnum):
    """Enum with values: `ALL`, `SIGINT`, `SIGTERM`, `NO`."""

    NO = ...
    SigInt = ...
    SigTerm = ...
    ALL = ...


class ClockEvent:

    def __init__(self) -> None: ...

    def wait_until_steady(self, clock: Clock, until: rcl_time_point_t) -> None:
        """Wait for the event to be set (monotonic wait)."""

    def wait_until_system(self, clock: Clock, until: rcl_time_point_t) -> None:
        """Wait for the event to be set (system timed wait)."""

    def wait_until_ros(self, clock: Clock, until: rcl_time_point_t) -> None:
        """Wait for the event to be set (ROS timed wait)."""

    def is_set(self) -> bool:
        """Return True if the event is set, False otherwise."""

    def set(self) -> None:   # noqa: A003
        """Set the event, waking all those who wait on it."""

    def clear(self) -> None:
        """Unset the event."""


_LifecycleStateMachineState: TypeAlias = tuple[int, str]


class LifecycleStateMachine(Destroyable):

    def __init__(self, node: Node, enable_com_interface: bool) -> None: ...

    @property
    def initialized(self) -> bool:
        """Check if state machine is initialized."""

    @property
    def current_state(self) -> _LifecycleStateMachineState:
        """Get the current state machine state."""

    @property
    def available_states(self) -> list[_LifecycleStateMachineState]:
        """Get the available states."""

    @property
    def available_transitions(self) -> list[tuple[int, str, int, str, int, str]]:
        """Get the available transitions."""

    @property
    def transition_graph(self) -> list[tuple[int, str, int, str, int, str]]:
        """Get the transition graph."""

    def get_transition_by_label(self, label: str) -> int:
        """Get the transition id from a transition label."""

    def trigger_transition_by_id(self, transition_id: int, publish_update: bool) -> None:
        """Trigger a transition by transition id."""

    def trigger_transition_by_label(self, label: str, publish_update: bool) -> None:
        """Trigger a transition by label."""

    @property
    def service_change_state(self) -> Service[Any, Any]:
        """Get the change state service."""

    @property
    def service_get_state(self) -> Service[Any, Any]:
        """Get the get state service."""

    @property
    def service_get_available_states(self) -> Service[Any, Any]:
        """Get the get available states service."""

    @property
    def service_get_available_transitions(self) -> Service[Any, Any]:
        """Get the get available transitions service."""

    @property
    def service_get_transition_graph(self) -> Service[Any, Any]:
        """Get the get transition graph service."""


class TransitionCallbackReturnType(IntEnum):
    SUCCESS = ...
    FAILURE = ...
    ERROR = ...

    def to_label(self) -> str:
        """Convert the transition callback return code to a transition label."""


class EventsExecutor:

    def __init__(self, context: RCLPyContext): ...

    @property
    def context(self) -> RCLPyContext: ...

    @overload
    def create_task(self, callback: Callable[..., Coroutine[Any, Any, T]],
                    *args: Any, **kwargs: Any
                    ) -> Task[T]: ...

    @overload
    def create_task(self, callback: Callable[..., T], *args: Any, **kwargs: Any
                    ) -> Task[T]: ...

    def shutdown(self, timeout_sec: Optional[float] = None) -> bool: ...

    def add_node(self, node: RCLPyNode) -> bool: ...

    def remove_node(self, node: RCLPyNode) -> None: ...

    def wake(self) -> None: ...

    def get_nodes(self) -> list[RCLPyNode]: ...

    def spin(self) -> None: ...

    def spin_once(self, timeout_sec: Optional[float] = None) -> None: ...

    def spin_until_future_complete(self, future: Future[Any],
                                   timeout_sec: Optional[float] = None) -> None: ...

    def spin_once_until_future_complete(self, future: Future[Any],
                                        timeout_sec: Optional[float] = None) -> None: ...

    def __enter__(self) -> EventsExecutor: ...

    def __exit__(self, exc_type: type[BaseException] | None,
                 exc_val: BaseException | None, exctb: TracebackType | None) -> None: ...
