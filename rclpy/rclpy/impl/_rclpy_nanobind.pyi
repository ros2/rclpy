"""ROS 2 Python client library."""

from collections.abc import Callable, Sequence
import enum
from typing import Any, Generic, TypeVar, overload

from action_msgs.msg import GoalInfo, GoalStatusArray
from action_msgs.srv._cancel_goal import (
    CancelGoal_Request,
    CancelGoal_Response
)
from rclpy.subscription import MessageInfo
from rclpy.subscription_content_filter_options import (
    ContentFilterOptions
)
from rclpy.type_support import (
    Action,
    FeedbackMessage,
    FeedbackT,
    GetResultServiceRequest,
    GetResultServiceResponse,
    GoalT,
    ImplT,
    MsgT,
    ResultT,
    SendGoalServiceRequest,
    SendGoalServiceResponse,
    Srv,
    SrvRequestT,
    SrvResponseT
)

from . import service_introspection as service_introspection


T = TypeVar("T")

class Destroyable:
    def __enter__(self) -> None: ...

    def __exit__(self, arg0: object | None, arg1: object | None, arg2: object | None) -> None: ...

    def destroy_when_not_in_use(self) -> None:
        """
        Forcefully destroy the rcl object as soon as it's not actively being used
        """

class ClockType(enum.IntEnum):
    UNINITIALIZED = 0

    ROS_TIME = 1

    SYSTEM_TIME = 2

    STEADY_TIME = 3

class GoalEvent(enum.IntEnum):
    EXECUTE = 0

    CANCEL_GOAL = 1

    SUCCEED = 2

    ABORT = 3

    CANCELED = 4

RCL_DEFAULT_DOMAIN_ID: int = 18446744073709551615

RMW_DURATION_INFINITE: int = 9223372036854775807

RMW_QOS_DEADLINE_BEST_AVAILABLE: int = 9223372036854775806

RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE: int = 9223372036854775806

class ClockChange(enum.IntEnum):
    ROS_TIME_NO_CHANGE = 1
    """ROS time is active and will continue to be active"""

    ROS_TIME_ACTIVATED = 2
    """ROS time is being activated"""

    ROS_TIME_DEACTIVATED = 3
    """
    ROS TIME is being deactivated, the clock will report system time after the jump
    """

    SYSTEM_TIME_NO_CHANGE = 4
    """ROS time is inactive and the clock will keep reporting system time"""

class QoSCompatibility(enum.IntEnum):
    OK = 0

    WARNING = 1

    ERROR = 2

class QoSCheckCompatibleResult:
    """Result type for checking QoS compatibility with result"""

    def __init__(self) -> None: ...

    @property
    def compatibility(self) -> QoSCompatibility: ...

    @property
    def reason(self) -> str: ...

class RCUtilsError(RuntimeError):
    pass

class RMWError(RuntimeError):
    pass

class RCLError(RuntimeError):
    pass

class RCLInvalidROSArgsError(RCLError):
    pass

class UnknownROSArgsError(RuntimeError):
    pass

class NodeNameNonExistentError(RCLError):
    pass

class UnsupportedEventTypeError(RCLError):
    pass

class TimerCancelledError(RCLError):
    pass

class NotImplementedError(NotImplementedError):
    pass

class InvalidHandle(RuntimeError):
    pass

class Client(Destroyable, Generic[SrvRequestT, SrvResponseT]):
    def __init__(self, node: Node, srv_type: type[Srv[SrvRequestT, SrvResponseT]], srv_name: str, pyqos_profile: rmw_qos_profile_t | None, /) -> None: ...

    @property
    def service_name(self) -> str:
        """Get the name of the service"""

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def send_request(self, pyrequest: SrvRequestT, /) -> int:
        """Send a request"""

    def service_server_is_available(self) -> bool:
        """Return true if the service server is available"""

    def take_response(self, pyresponse_type: type[SrvResponseT], /) -> tuple[rmw_service_info_t, SrvResponseT] | tuple[None, None]:
        """Take a received response from an earlier request"""

    def configure_introspection(self, arg0: Clock, arg1: rmw_qos_profile_t | None, arg2: service_introspection.ServiceIntrospectionState) -> None:
        """Configure whether introspection is enabled"""

    def get_logger_name(self) -> str:
        """Get the name of the logger associated with the node of the client."""

    def set_on_new_response_callback(self, callback: Callable[[int], None]) -> None: ...

    def clear_on_new_response_callback(self) -> None: ...

class Context(Destroyable):
    def __init__(self, arg0: list, arg1: int, /) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def get_domain_id(self) -> int:
        """Retrieves domain id from init_options of context."""

    def ok(self) -> bool:
        """Status of the the client library"""

    def shutdown(self) -> None:
        """Shutdown context"""

class rcl_duration_t:
    def __init__(self, arg: int, /) -> None: ...

    @property
    def nanoseconds(self) -> int: ...

class Publisher(Destroyable, Generic[MsgT]):
    def __init__(self, node: Node, msg_type: type[MsgT], topic: str, pyqos_profile: rmw_qos_profile_t | None, /) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def get_logger_name(self) -> str:
        """Get the name of the logger associated with the node of the publisher"""

    def get_subscription_count(self) -> int:
        """Count subscribers from a publisher."""

    def get_topic_name(self) -> str:
        """Retrieve the topic name from a Publisher."""

    def publish(self, msg: MsgT, /) -> None:
        """Publish a message"""

    def publish_raw(self, arg: bytes, /) -> None:
        """Publish a serialized message."""

    def wait_for_all_acked(self, arg: rcl_duration_t, /) -> bool:
        """Wait until all published message data is acknowledged"""

class Service(Destroyable, Generic[SrvRequestT, SrvResponseT]):
    def __init__(self, node: Node, pysrv_type: type[Srv[SrvRequestT, SrvResponseT]], name: str, pyqos_profile: rmw_qos_profile_t | None, /) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    @property
    def name(self) -> str:
        """Get the name of the service"""

    @property
    def qos(self) -> dict:
        """Get the qos profile of the service"""

    def service_send_response(self, pyresponse: SrvResponseT, header: rmw_request_id_t, /) -> None:
        """Send a response"""

    def service_take_request(self, pyrequest_type: type[SrvRequestT], /) -> tuple[SrvRequestT, rmw_service_info_t] | tuple[None, None]:
        """Take a request from a given service"""

    def configure_introspection(self, arg0: Clock, arg1: rmw_qos_profile_t | None, arg2: service_introspection.ServiceIntrospectionState) -> None:
        """Configure whether introspection is enabled"""

    def get_logger_name(self) -> str:
        """Get the name of the logger associated with the node of the service."""

    def set_on_new_request_callback(self, callback: Callable[[int], None]) -> None: ...

    def clear_on_new_request_callback(self) -> None: ...

class TypeDescriptionService(Destroyable):
    def __init__(self, arg: Node, /) -> None: ...

    @property
    def impl(self) -> Service:
        """Get the rcl service wrapper capsule."""

    def handle_request(self, arg0: object, arg1: object, arg2: Node, /) -> object:
        """Handle an incoming request by calling RCL implementation"""

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

def rclpy_qos_check_compatible(arg0: rmw_qos_profile_t, arg1: rmw_qos_profile_t, /) -> QoSCheckCompatibleResult:
    """Check if two QoS profiles are compatible."""

class ActionClient(Destroyable, Generic[GoalT, ResultT, FeedbackT, ImplT]):
    def __init__(self, node: Node, action_type: type[Action[GoalT, ResultT, FeedbackT, ImplT]], action_name: str, goal_service_qos_profile: rmw_qos_profile_t, result_service_qos_profile: rmw_qos_profile_t, cancel_service_qos_profile: rmw_qos_profile_t, feedback_sub_qos_profile: rmw_qos_profile_t, status_sub_qos_profile: rmw_qos_profile_t, enable_feedback_msg_optimization: bool = False) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def take_goal_response(self, pymsg_type: type[SendGoalServiceResponse], /) -> tuple[int, SendGoalServiceResponse] | tuple[None, None]:
        """Take an action goal response."""

    def send_result_request(self, pyrequest: GetResultServiceRequest, /) -> int:
        """Send an action result request."""

    def take_cancel_response(self, pymsg_type: type[CancelGoal_Response], /) -> tuple[int, CancelGoal_Response] | tuple[None, None]:
        """Take an action cancel response."""

    def take_feedback(self, pymsg_type: type[FeedbackMessage[FeedbackT]], /) -> FeedbackMessage[FeedbackT] | None:
        """Take a feedback message from a given action client."""

    def send_cancel_request(self, pyrequest: CancelGoal_Request, /) -> int:
        """Send an action cancel request."""

    def send_goal_request(self, pyrequest: SendGoalServiceRequest[GoalT], /) -> int:
        """Send an action goal request."""

    def take_result_response(self, pymsg_type: type[GetResultServiceResponse[ResultT]], /) -> tuple[int, GetResultServiceResponse[ResultT]] | tuple[None, None]:
        """Take an action result response."""

    def get_num_entities(self) -> tuple:
        """Get the number of wait set entities that make up an action entity."""

    def is_action_server_available(self) -> bool:
        """Check if an action server is available for the given action client."""

    def add_to_waitset(self, arg: WaitSet, /) -> None:
        """Add an action entity to a wait set."""

    def is_ready(self, arg: WaitSet, /) -> tuple:
        """Check if an action entity has any ready wait set entities."""

    def take_status(self, pymsg_type: type[GoalStatusArray], /) -> GoalStatusArray | None:
        """Take an action status response."""

    def configure_introspection(self, arg0: Clock, arg1: rmw_qos_profile_t | None, arg2: service_introspection.ServiceIntrospectionState) -> None:
        """Configure whether internal client introspection is enabled"""

    def configure_feedback_subscription_filter_add_goal_id(self, arg: bytes, /) -> bool:
        """Configure feedback subscription content filter to add a goal ID."""

    def configure_feedback_subscription_filter_remove_goal_id(self, arg: bytes, /) -> bool:
        """Configure feedback subscription content filter to remove a goal ID."""

class ActionGoalHandle(Destroyable):
    def __init__(self, arg0: ActionServer, arg1: object, /) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def get_status(self) -> int:
        """Get the status of a goal."""

    def update_goal_state(self, arg: GoalEvent, /) -> None:
        """Update a goal state."""

    def is_active(self) -> bool:
        """Check if a goal is active."""

class ActionServer(Destroyable, Generic[GoalT, ResultT, FeedbackT, ImplT]):
    def __init__(self, node: Node, rclpy_clock: Clock, pyaction_type: type[Action[GoalT, ResultT, FeedbackT, ImplT]], action_name: str, goal_service_qos: rmw_qos_profile_t, result_service_qos: rmw_qos_profile_t, cancel_service_qos: rmw_qos_profile_t, feedback_topic_qos: rmw_qos_profile_t, status_topic_qos: rmw_qos_profile_t, result_timeout: float, /) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def take_goal_request(self, pymsg_type: type[SendGoalServiceRequest[GoalT]], /) -> tuple[rmw_request_id_t, SendGoalServiceRequest[GoalT]] | tuple[None, None]:
        """Take an action goal request."""

    def send_goal_response(self, header: rmw_request_id_t, pyresponse: SendGoalServiceResponse, /) -> None:
        """Send an action goal response."""

    def send_result_response(self, header: rmw_request_id_t, pyresponse: GetResultServiceResponse[ResultT], /) -> None:
        """Send an action result response."""

    def take_cancel_request(self, pymsg_type: type[CancelGoal_Request], /) -> tuple[rmw_request_id_t, CancelGoal_Request] | tuple[None, None]:
        """Take an action cancel request."""

    def take_result_request(self, pymsg_type: type[GetResultServiceRequest], /) -> tuple[rmw_request_id_t, GetResultServiceRequest] | tuple[None, None]:
        """Take an action result request."""

    def send_cancel_response(self, header: rmw_request_id_t, pyresponse: CancelGoal_Response, /) -> None:
        """Send an action cancel response."""

    def publish_feedback(self, pymsg: FeedbackT, /) -> None:
        """Publish a feedback message from a given action server."""

    def publish_status(self) -> None:
        """Publish a status message from a given action server."""

    def notify_goal_done(self) -> None:
        """Notify goal is done."""

    def goal_exists(self, pygoal_info: GoalInfo, /) -> bool:
        """Check is a goal exists in the server."""

    def process_cancel_request(self, pycancel_request: CancelGoal_Request, pycancel_response_type: type[CancelGoal_Response], /) -> CancelGoal_Response:
        """Process a cancel request"""

    def expire_goals(self, max_num_goals: int, /) -> tuple[GoalInfo, ...]:
        """Expired goals."""

    def get_num_entities(self) -> tuple:
        """Get the number of wait set entities that make up an action entity."""

    def is_ready(self, arg: WaitSet, /) -> tuple:
        """Check if an action entity has any ready wait set entities."""

    def add_to_waitset(self, arg: WaitSet, /) -> None:
        """Add an action entity to a wait set."""

    def configure_introspection(self, arg0: Clock, arg1: rmw_qos_profile_t | None, arg2: service_introspection.ServiceIntrospectionState) -> None:
        """Configure whether internal service introspection is enabled"""

def rclpy_action_get_rmw_qos_profile(arg: str, /) -> dict:
    """Get an action RMW QoS profile."""

class GuardCondition(Destroyable):
    def __init__(self, arg: Context, /) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def trigger_guard_condition(self) -> None:
        """Trigger a general purpose guard condition"""

class Timer(Destroyable):
    def __init__(self, arg0: Clock, arg1: Context, arg2: int, arg3: bool, /) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def reset_timer(self) -> None:
        """Reset a timer."""

    def is_timer_ready(self) -> bool:
        """Check if a timer as reached timeout."""

    def call_timer(self) -> None:
        """Call a timer and starts counting again."""

    def call_timer_with_info(self) -> object:
        """
        Call a timer and starts counting again, retrieves actual and expected call time.
        """

    def change_timer_period(self, arg: int, /) -> None:
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

    def set_on_reset_callback(self, callback: Callable[[int], None]) -> None: ...

    def clear_on_reset_callback(self) -> None: ...

class Subscription(Destroyable, Generic[MsgT]):
    def __init__(self, node: Node, msg_type: type[MsgT], topic: str, qos_profile: rmw_qos_profile_t | None, content_filter_options: ContentFilterOptions | None = None, acceptable_buffer_backends: str | None = None) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def take_message(self, pymsg_type: type[MsgT], raw: bool, /) -> tuple[MsgT | bytes, MessageInfo] | None:
        """Take a message and its metadata from a subscription"""

    def get_logger_name(self) -> str:
        """
        Get the name of the logger associated with the node of the subscription.
        """

    def get_topic_name(self) -> str:
        """Return the resolved topic name of a subscription."""

    def get_publisher_count(self) -> int:
        """Count the publishers from a subscription."""

    def set_on_new_message_callback(self, callback: Callable[[int], None]) -> None: ...

    def clear_on_new_message_callback(self) -> None: ...

    def is_cft_supported(self) -> bool:
        """Check if subscription instance supports content filtering."""

    def is_cft_enabled(self) -> bool:
        """Check if content filtering is enabled for this subscription."""

    def set_content_filter(self, arg0: str, arg1: Sequence[str], /) -> None:
        """
        Set the filter expression and expression parameters for the subscription.
        """

    def get_content_filter(self) -> ContentFilterOptions:
        """
        Get the filter expression and expression parameters for the subscription.
        """

class rcl_time_point_t:
    def __init__(self, arg0: int, arg1: int, /) -> None: ...

    @property
    def nanoseconds(self) -> int: ...

    @property
    def clock_type(self) -> ClockType: ...

class Clock(Destroyable):
    def __init__(self, arg: int, /) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def get_now(self) -> rcl_time_point_t:
        """Current value of the clock"""

    def get_ros_time_override_is_enabled(self) -> bool:
        """Returns if a clock using ROS time has the ROS time override enabled."""

    def set_ros_time_override_is_enabled(self, arg: bool, /) -> None:
        """Set if a clock using ROS time has the ROS time override enabled."""

    def set_ros_time_override(self, arg: rcl_time_point_t, /) -> None:
        """Set the ROS time override for a clock using ROS time."""

    def add_clock_callback(self, arg0: object, arg1: bool, arg2: int, arg3: int, /) -> None:
        """Add a time jump callback to a clock."""

    def remove_clock_callback(self, arg: object, /) -> None:
        """Remove a time jump callback from a clock."""

class WaitSet(Destroyable):
    def __init__(self, arg0: int, arg1: int, arg2: int, arg3: int, arg4: int, arg5: int, arg6: Context, /) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def clear_entities(self) -> None:
        """Clear all the pointers in the wait set"""

    def add_service(self, arg: Service, /) -> int:
        """Add a service to the wait set structure"""

    def add_subscription(self, arg: Subscription, /) -> int:
        """Add a subscription to the wait set structure"""

    def add_client(self, arg: Client, /) -> int:
        """Add a client to the wait set structure"""

    def add_guard_condition(self, arg: GuardCondition, /) -> int:
        """Add a guard condition to the wait set structure"""

    def add_timer(self, arg: Timer, /) -> int:
        """Add a timer to the wait set structure"""

    def add_event(self, arg: EventHandle, /) -> int:
        """Add an event to the wait set structure"""

    def is_ready(self, arg0: str, arg1: int, /) -> bool:
        """Check if an entity in the wait set is ready by its index"""

    def get_ready_entities(self, arg: str, /) -> list:
        """Get list of entities ready by entity type"""

    def wait(self, arg: int, /) -> None:
        """Wait until timeout is reached or event happened"""

def rclpy_expand_topic_name(arg0: str, arg1: str, arg2: str, /) -> str:
    """Expand a topic name."""

def rclpy_remap_topic_name(arg0: Node, arg1: str, /) -> str:
    """Remap a topic name."""

def rclpy_get_validation_error_for_topic_name(arg: str, /) -> object:
    """
    Get the error message and invalid index of a topic name or None if valid.
    """

def rclpy_get_validation_error_for_full_topic_name(arg: str, /) -> object:
    """
    Get the error message and invalid index of a full topic name or None if valid.
    """

def rclpy_get_validation_error_for_namespace(arg: str, /) -> object:
    """
    Get the error message and invalid index of a namespace or None if valid.
    """

def rclpy_get_validation_error_for_node_name(arg: str, /) -> object:
    """
    Get the error message and invalid index of a node name or None if valid.
    """

def rclpy_resolve_name(arg0: Node, arg1: str, arg2: bool, arg3: bool, /) -> str:
    """Expand and remap a topic or service name."""

def rclpy_get_topic_names_and_types(arg0: Node, arg1: bool, /) -> list:
    """Get all topic names and types in the ROS graph."""

def rclpy_get_publisher_names_and_types_by_node(arg0: Node, arg1: bool, arg2: str, arg3: str, /) -> list:
    """Get topic names and types for which a remote node has publishers."""

def rclpy_get_subscriber_names_and_types_by_node(arg0: Node, arg1: bool, arg2: str, arg3: str, /) -> list:
    """Get topic names and types for which a remote node has subscribers."""

def rclpy_get_publishers_info_by_topic(arg0: Node, arg1: str, arg2: bool, /) -> list:
    """Get publishers info for a topic."""

def rclpy_get_subscriptions_info_by_topic(arg0: Node, arg1: str, arg2: bool, /) -> list:
    """Get subscriptions info for a topic."""

def rclpy_get_clients_info_by_service(arg0: Node, arg1: str, arg2: bool, /) -> list:
    """Get clients info for a service."""

def rclpy_get_servers_info_by_service(arg0: Node, arg1: str, arg2: bool, /) -> list:
    """Get servers info for a service."""

def rclpy_get_service_names_and_types(arg: Node, /) -> list:
    """Get all service names and types in the ROS graph."""

def rclpy_get_service_names_and_types_by_node(arg0: Node, arg1: str, arg2: str, /) -> list:
    """Get service names and types for which a remote node has servers."""

def rclpy_get_client_names_and_types_by_node(arg0: Node, arg1: str, arg2: str, /) -> list:
    """Get service names and types for which a remote node has clients."""

def rclpy_get_action_client_names_and_types_by_node(arg0: Node, arg1: str, arg2: str, /) -> list:
    """Get action client names and types by node."""

def rclpy_get_action_server_names_and_types_by_node(arg0: Node, arg1: str, arg2: str, /) -> list:
    """Get action server names and types by node."""

def rclpy_get_action_names_and_types(arg: Node, /) -> list:
    """Get all action names and types in the ROS graph."""

def rclpy_get_action_clients_info_by_action(arg0: Node, arg1: str, /) -> list:
    """Get action clients info for an action."""

def rclpy_get_action_servers_info_by_action(arg0: Node, arg1: str, /) -> list:
    """Get action servers info for an action."""

def rclpy_serialize(arg0: object, arg1: object, /) -> bytes:
    """Serialize a ROS message."""

def rclpy_deserialize(arg0: bytes, arg1: object, /) -> object:
    """Deserialize a ROS message."""

class Node(Destroyable):
    def __init__(self, arg0: str, arg1: str, arg2: Context, arg3: list | None, arg4: bool, arg5: bool, arg6: rmw_qos_profile_t | None) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def get_fully_qualified_name(self) -> str:
        """Get the fully qualified name of the node."""

    def logger_name(self) -> str:
        """Get the name of the logger associated with a node."""

    def get_node_name(self) -> str:
        """Get the name of a node."""

    def get_namespace(self) -> str:
        """Get the namespace of a node."""

    def get_count_publishers(self, arg: str, /) -> int:
        """
        Returns the count of all the publishers known for that topic in the entire ROS graph.
        """

    def get_count_subscribers(self, arg: str, /) -> int:
        """
        Returns the count of all the subscribers known for that topic in the entire ROS graph.
        """

    def get_count_clients(self, arg: str, /) -> int:
        """
        Returns the count of all the clients known for that service in the entire ROS graph.
        """

    def get_count_services(self, arg: str, /) -> int:
        """
        Returns the count of all the servers known for that service in the entire ROS graph.
        """

    def get_count_action_clients(self, arg: str, /) -> int:
        """
        Returns the count of all the action clients known for that action in the entire ROS graph.
        """

    def get_count_action_servers(self, arg: str, /) -> int:
        """
        Returns the count of all the action servers known for that action in the entire ROS graph.
        """

    def get_node_names_and_namespaces(self) -> list:
        """Get the list of nodes discovered by the provided node"""

    def get_node_names_and_namespaces_with_enclaves(self) -> list:
        """
        Get the list of nodes discovered by the provided node, with their respective enclaves.
        """

    def get_action_client_names_and_types_by_node(self, arg0: str, arg1: str, /) -> list:
        """Get action client names and types by node."""

    def get_action_server_names_and_types_by_node(self, arg0: str, arg1: str, /) -> list:
        """Get action server names and types by node."""

    def get_action_names_and_types(self) -> list:
        """Get action names and types."""

    def get_parameters(self, arg: object, /) -> dict:
        """Get a list of parameters for the current node"""

class EventHandle(Destroyable, Generic[T]):
    @overload
    def __init__(self, subscription: Subscription[Any], event_type: rcl_subscription_event_type_t, /) -> None: ...

    @overload
    def __init__(self, publisher: Publisher[Any], event_type: rcl_publisher_event_type_t, /) -> None: ...

    @property
    def pointer(self) -> int:
        """Get the address of the entity as an integer"""

    def take_event(self) -> T | None:
        """Get pending data from a ready event"""

class rcl_subscription_event_type_t(enum.IntEnum):
    RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED = 0

    RCL_SUBSCRIPTION_LIVELINESS_CHANGED = 1

    RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS = 2

    RCL_SUBSCRIPTION_MESSAGE_LOST = 3

    RCL_SUBSCRIPTION_INCOMPATIBLE_TYPE = 4

    RCL_SUBSCRIPTION_MATCHED = 5

class rcl_publisher_event_type_t(enum.IntEnum):
    RCL_PUBLISHER_OFFERED_DEADLINE_MISSED = 0

    RCL_PUBLISHER_LIVELINESS_LOST = 1

    RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS = 2

    RCL_PUBLISHER_INCOMPATIBLE_TYPE = 3

    RCL_PUBLISHER_MATCHED = 4

class rmw_requested_deadline_missed_status_t:
    def __init__(self) -> None: ...

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...

class rmw_liveliness_changed_status_t:
    def __init__(self) -> None: ...

    @property
    def alive_count(self) -> int: ...

    @property
    def not_alive_count(self) -> int: ...

    @property
    def alive_count_change(self) -> int: ...

    @property
    def not_alive_count_change(self) -> int: ...

class rmw_message_lost_status_t:
    def __init__(self) -> None: ...

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...

class rmw_requested_qos_incompatible_event_status_t:
    def __init__(self) -> None: ...

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...

    @property
    def last_policy_kind(self) -> rmw_qos_policy_kind_t: ...

class rmw_offered_deadline_missed_status_t:
    def __init__(self) -> None: ...

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...

class rmw_liveliness_lost_status_t:
    def __init__(self) -> None: ...

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...

class rmw_matched_status_t:
    def __init__(self) -> None: ...

    @property
    def total_count(self) -> int: ...

    @property
    def total_count_change(self) -> int: ...

    @property
    def current_count(self) -> int: ...

    @property
    def current_count_change(self) -> int: ...

class rmw_qos_policy_kind_t(enum.IntEnum):
    RMW_QOS_POLICY_INVALID = 1

    RMW_QOS_POLICY_DURABILITY = 2

    RMW_QOS_POLICY_DEADLINE = 4

    RMW_QOS_POLICY_LIVELINESS = 8

    RMW_QOS_POLICY_RELIABILITY = 16

    RMW_QOS_POLICY_HISTORY = 32

    RMW_QOS_POLICY_LIFESPAN = 64

    RMW_QOS_POLICY_DEPTH = 128

    RMW_QOS_POLICY_LIVELINESS_LEASE_DURATION = 256

    RMW_QOS_POLICY_AVOID_ROS_NAMESPACE_CONVENTIONS = 512

class rmw_incompatible_type_status_t:
    def __init__(self) -> None: ...

    @property
    def total_count_change(self) -> int: ...

def publisher_event_type_is_supported(arg: rcl_publisher_event_type_t, /) -> bool:
    """
    Check if a publisher event type is supported by the active RMW implementation.
    """

def subscription_event_type_is_supported(arg: rcl_subscription_event_type_t, /) -> bool:
    """
    Check if a subscription event type is supported by the active RMW implementation.
    """

def rclpy_get_rmw_implementation_identifier() -> str:
    """Retrieve the identifier for the active RMW implementation."""

def rclpy_assert_liveliness(arg: Publisher, /) -> None:
    """Assert the liveliness of an entity."""

def rclpy_remove_ros_args(arg: list | None) -> list:
    """Remove ROS-specific arguments from argument vector."""

class rmw_qos_profile_t:
    def __init__(self, arg0: int, arg1: int, arg2: int, arg3: int, arg4: rcl_duration_t, arg5: rcl_duration_t, arg6: int, arg7: rcl_duration_t, arg8: bool, /) -> None: ...

    def to_dict(self) -> dict: ...

    @staticmethod
    def predefined(arg: str, /) -> rmw_qos_profile_t: ...

def rclpy_logging_fini() -> None:
    """Finalize RCL logging."""

def rclpy_logging_configure(arg: Context, /) -> None:
    """Initialize RCL logging."""

class RCUTILS_LOG_SEVERITY(enum.IntEnum):
    RCUTILS_LOG_SEVERITY_UNSET = 0

    RCUTILS_LOG_SEVERITY_DEBUG = 10

    RCUTILS_LOG_SEVERITY_INFO = 20

    RCUTILS_LOG_SEVERITY_WARN = 30

    RCUTILS_LOG_SEVERITY_ERROR = 40

    RCUTILS_LOG_SEVERITY_FATAL = 50

def rclpy_logging_get_separator_string() -> str: ...

def rclpy_logging_initialize() -> None: ...

def rclpy_logging_shutdown() -> None: ...

def rclpy_logging_set_logger_level(name: str, level: int, detailed_error: bool = False) -> None: ...

def rclpy_logging_get_logger_effective_level(arg: str, /) -> int: ...

def rclpy_logging_logger_is_enabled_for(arg0: str, arg1: int, /) -> bool: ...

def rclpy_logging_rcutils_log(arg0: int, arg1: str, arg2: str, arg3: str, arg4: str, arg5: int, /) -> None: ...

def rclpy_logging_severity_level_from_string(arg: str, /) -> int: ...

def rclpy_logging_get_logging_directory() -> str: ...

def rclpy_logging_rosout_add_sublogger(arg0: str, arg1: str, /) -> bool: ...

def rclpy_logging_rosout_remove_sublogger(arg0: str, arg1: str, /) -> None: ...

def rclpy_logging_get_logger_level(arg: str, /) -> int: ...

def register_sigint_guard_condition(arg: GuardCondition, /) -> None:
    """Register a guard condition to be called on SIGINT."""

def unregister_sigint_guard_condition(arg: GuardCondition, /) -> None:
    """Stop triggering a guard condition when SIGINT occurs."""

def install_signal_handlers(arg: SignalHandlerOptions, /) -> None:
    """Install rclpy signal handlers."""

def get_current_signal_handlers_options() -> SignalHandlerOptions:
    """Get currently installed signal handler options."""

def uninstall_signal_handlers() -> None:
    """Uninstall rclpy signal handlers."""

class SignalHandlerOptions(enum.IntEnum):
    """Enum with values: `ALL`, `SIGINT`, `SIGTERM`, `NO`."""

    ALL = 3

    NO = 0

    SIGINT = 1

    SIGTERM = 2

class ClockEvent:
    def __init__(self) -> None: ...

    def wait_until_steady(self, arg0: Clock, arg1: rcl_time_point_t, /) -> None:
        """Wait for the event to be set (monotonic wait)"""

    def wait_until_system(self, arg0: Clock, arg1: rcl_time_point_t, /) -> None:
        """Wait for the event to be set (system timed wait)"""

    def wait_until_ros(self, arg0: Clock, arg1: rcl_time_point_t, /) -> None:
        """Wait for the event to be set (ROS timed wait)"""

    def is_set(self) -> bool:
        """Return True if the event is set, False otherwise."""

    def set(self) -> None:
        """Set the event, waking all those who wait on it."""

    def clear(self) -> None:
        """Unset the event."""

class LifecycleStateMachine(Destroyable):
    def __init__(self, arg0: Node, arg1: Clock, arg2: bool, /) -> None: ...

    @property
    def initialized(self) -> bool:
        """Check if state machine is initialized."""

    @property
    def current_state(self) -> tuple:
        """Get the current state machine state."""

    @property
    def available_states(self) -> list[tuple[int, str]]:
        """Get the available states."""

    @property
    def available_transitions(self) -> list[tuple[int, str, int, str, int, str]]:
        """Get the available transitions."""

    @property
    def transition_graph(self) -> list[tuple[int, str, int, str, int, str]]:
        """Get the transition graph."""

    def get_transition_by_label(self, arg: str, /) -> int:
        """Get the transition id from a transition label."""

    def trigger_transition_by_id(self, arg0: int, arg1: bool, /) -> None:
        """Trigger a transition by transition id."""

    def trigger_transition_by_label(self, arg0: str, arg1: bool, /) -> None:
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

class TransitionCallbackReturnType(enum.IntEnum):
    SUCCESS = 97
    """Callback succeeded."""

    FAILURE = 98
    """Callback failed."""

    ERROR = 99
    """Callback had an error."""

    def to_label(self) -> str:
        """Convert the transition callback return code to a transition label"""

class EventsExecutor:
    def __init__(self, context: object) -> None: ...

    @property
    def context(self) -> object: ...

    def create_task(self, callback: object, *args, **kwargs) -> object: ...

    def create_future(self) -> object: ...

    def shutdown(self, timeout_sec: float | None = None) -> bool: ...

    def add_node(self, node: object) -> bool: ...

    def remove_node(self, node: object) -> None: ...

    def wake(self) -> None: ...

    def get_nodes(self) -> list: ...

    def spin(self) -> None: ...

    def spin_once(self, timeout_sec: float | None = None) -> None: ...

    def spin_until_future_complete(self, future: object, timeout_sec: float | None = None) -> None: ...

    def spin_once_until_future_complete(self, future: object, timeout_sec: float | None = None) -> None: ...

    def __enter__(self) -> EventsExecutor: ...

    def __exit__(self, arg0: object | None, arg1: object | None, arg2: object | None) -> None: ...
