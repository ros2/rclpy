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

from enum import Enum, IntEnum
from types import TracebackType
from typing import Any, Generic, Literal, overload, Sequence, TypeAlias, TypedDict


from action_msgs.msg import GoalInfo
from action_msgs.msg._goal_status_array import GoalStatusArray
from action_msgs.srv._cancel_goal import CancelGoal
from rclpy.clock import JumpHandle
from rclpy.clock_type import ClockType
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.subscription import MessageInfo
from type_support import (MsgT, Action, GoalT, ResultT, FeedbackT, SendGoalServiceResponse,
                          GetResultServiceResponse, FeedbackMessage, SendGoalServiceRequest, GetResultServiceRequest)


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


class rcl_subscription_event_type_t(Enum):
    _value_: int
    RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED = ...
    RCL_SUBSCRIPTION_LIVELINESS_CHANGED = ...
    RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS = ...
    RCL_SUBSCRIPTION_MESSAGE_LOST = ...
    RCL_SUBSCRIPTION_INCOMPATIBLE_TYPE = ...
    RCL_SUBSCRIPTION_MATCHED = ...


class rcl_publisher_event_type_t(Enum):
    _value_: int
    RCL_PUBLISHER_OFFERED_DEADLINE_MISSED = ...
    RCL_PUBLISHER_LIVELINESS_LOST = ...
    RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS = ...
    RCL_PUBLISHER_INCOMPATIBLE_TYPE = ...
    RCL_PUBLISHER_MATCHED = ...


class EventHandle(Destroyable):

    @overload
    def __init__(self, subcription: Subscription,
                 event_type: rcl_subscription_event_type_t) -> None: ...

    @overload
    def __init__(self, publisher: Publisher, event_type: rcl_publisher_event_type_t) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer."""

    def take_event(self) -> Any | None:
        """Get pending data from a ready event."""


LifecycleStateMachineState: TypeAlias = tuple[int, str]


class LifecycleStateMachine(Destroyable):

    def __init__(self, node: Node, enable_com_interface: bool) -> None: ...

    @property
    def initialized(self) -> bool:
        """Check if state machine is initialized."""

    @property
    def current_state(self) -> LifecycleStateMachineState:
        """Get the current state machine state."""

    @property
    def available_states(self) -> list[LifecycleStateMachineState]:
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
    def service_change_state(self) -> Service:
        """Get the change state service."""

    @property
    def service_get_state(self) -> Service:
        """Get the get state service."""

    @property
    def service_get_available_states(self) -> Service:
        """Get the get available states service."""

    @property
    def service_get_available_transitions(self) -> Service:
        """Get the get available transitions service."""

    @property
    def service_get_transition_graph(self) -> Service:
        """Get the get transition graph service."""


class TransitionCallbackReturnType(Enum):
    _value_: int
    SUCCESS = ...
    FAILURE = ...
    ERROR = ...

    def to_label(self) -> str:
        """Convert the transition callback return code to a transition label."""


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


class Node(Destroyable):

    def __init__(self, node_name: str, namespace_: str, context: Context,
                 pycli_args: list[str] | None, use_global_arguments: bool,
                 enable_rosout: bool) -> None: ...

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

    def get_node_names_and_namespaces(self) -> list[tuple[str, str, str] | tuple[str, str]]:
        """Get the list of nodes discovered by the provided node."""

    def get_node_names_and_namespaces_with_enclaves(self) -> list[tuple[str, str, str] |
                                                                  tuple[str, str]]:
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

    def get_parameters(self, pyparamter_cls: type[Parameter]) -> dict[str, Parameter]:
        """Get a list of parameters for the current node."""


def rclpy_resolve_name(node: Node, topic_name: str, only_expand: bool, is_service: bool) -> str:
    """Expand and remap a topic or service name."""


def rclpy_get_publisher_names_and_types_by_node(node: Node, no_demangle: bool, node_name: str,
                                                node_namespace: str
                                                ) -> list[tuple[str, list[str]]]:
    """Get topic names and types for which a remote node has publishers."""


def rclpy_get_subscriber_names_and_types_by_node(node: Node, no_demangle: bool, node_name: str,
                                                 node_namespace: str
                                                 ) -> list[tuple[str, list[str]]]:
    """Get topic names and types for which a remote node has subscribers."""


def rclpy_get_service_names_and_types_by_node(node: Node, node_name: str, node_namespace: str
                                              ) -> list[tuple[str, list[str]]]:
    """Get all service names and types in the ROS graph."""


def rclpy_get_client_names_and_types_by_node(node: Node, node_name: str, node_namespace: str
                                             ) -> list[tuple[str, list[str]]]:
    """Get service names and types for which a remote node has servers."""


def rclpy_get_service_names_and_types(node: Node) -> list[tuple[str, list[str]]]:
    """Get all service names and types in the ROS graph."""


class TypeHashDict(TypedDict):
    version: int
    value: bytes


class QoSDict(TypedDict):
    pass


class TopicEndpointInfoDict(TypedDict):
    node_name: str
    node_namespace: str
    topic_type: str
    topic_type_hash: TypeHashDict
    endpoint_type: int
    endpoint_gid: list[int]
    qos_profile: rmw_qos_profile_dict


def rclpy_get_publishers_info_by_topic(node: Node, topic_name: str, no_mangle: bool
                                       ) -> list[TopicEndpointInfoDict]:
    """Get publishers info for a topic."""


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


class ActionClient(Generic[GoalT, ResultT, FeedbackT], Destroyable):

    def __init__(
            self,
            node: Node,
            pyaction_type: type[Action[GoalT, ResultT, FeedbackT]],
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

    def send_cancel_request(self: CancelGoal.Request) -> int:
        """Send an action cancel request."""

    def send_goal_request(self: SendGoalServiceRequest[GoalT]) -> int:
        """Send an action goal request."""
    
    def take_result_response(self, pymsg_type: type[GetResultServiceResponse[ResultT]]
                             ) -> tuple[int, GetResultServiceResponse[ResultT]] | tuple[None, None]:
        """Take an action result response."""

    def get_num_entities(self) -> tuple[int, int, int, int, int]:
        """Get the number of wait set entities that make up an action entity."""

    def is_action_server_available(self) -> bool:
        """Check if an action server is available for the given action client."""   

    def add_to_waitset(self, waitset: WaitSet) -> None:
        """Add an action entity to a wait set."""

    def is_ready(self) -> bool:
        """Check if an action entity has any ready wait set entities."""

    def take_status(self, pymsg_type: type[GoalStatusArray]) -> GoalStatusArray | None:
        """Take an action status response."""


class GoalEvent(Enum):
    _value_: int
    EXECUTE = ...
    CANCEL_GOAL = ...
    SUCCEED = ...
    ABORT = ...
    CANCELED = ...


class rmw_request_id_t:
    writer_guid: list[int]
    sequence_number: int


class ActionServer(Generic[GoalT, ResultT, FeedbackT], Destroyable):

    def __init__(
        self,
        node: Node,
        rclpy_clock: Clock,
        pyaction_type: type[Action[GoalT, ResultT, FeedbackT]],
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
        """Process a cancel request"""

    def expire_goals(self, max_num_goals: int) -> tuple[GoalInfo, ...]:
        """Expired goals."""

    def get_num_entities(self) -> tuple[int, int, int, int, int]:
        """Get the number of wait set entities that make up an action entity."""

    def is_ready(self, wait_set: WaitSet) -> tuple[bool, bool, bool, bool]:
        """Check if an action entity has any ready wait set entities."""

    def add_to_waitset(self, wait_set: WaitSet) -> None:
        """Add an action entity to a wait set."""


class ActionGoalHandle:

    def __init__(self, action_server: ActionServer, pygoal_info_msg: GoalInfo) -> None:
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


class RCLError(RuntimeError):
    pass


class NodeNameNonExistentError(RCLError):
    pass


class InvalidHandle(RuntimeError):
    pass


class SignalHandlerOptions(Enum):
    _value_: int
    NO = ...
    SigInt = ...
    SigTerm = ...
    All = ...


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
