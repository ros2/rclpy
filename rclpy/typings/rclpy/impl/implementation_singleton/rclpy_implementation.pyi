from enum import Enum, IntEnum, auto
from typing import Any, Dict, List, Literal, NewType, Optional, Sequence, Tuple, Type, TypeVar
from rclpy.clock import JumpHandle

from rclpy.qos import QoSPolicyKind

WaitSetIndex = NewType('WaitSetIndex', int)
EntityType = Literal['subscription', 'service', 'timer', 'client', 'event', 'guard_condition']


class rcl_publisher_event_type_t(Enum):
    RCL_PUBLISHER_OFFERED_DEADLINE_MISSED = auto()
    RCL_PUBLISHER_LIVELINESS_LOST = auto()
    RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS = auto()


class rcl_subscription_event_type_t(IntEnum):
    RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED = auto()
    RCL_SUBSCRIPTION_LIVELINESS_CHANGED = auto()
    RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS = auto()
    RCL_SUBSCRIPTION_MESSAGE_LOST = auto()


class rmw_requested_deadline_missed_status_t:
    """
    Lifetime cumulative number of missed deadlines detected for any instance read by the
    subscription.
    Missed deadlines accumulate; that is, each deadline period the total_count will be incremented
    by one for each instance for which data was not received.
    """
    total_count: int
    total_count_change: int

# QoS Liveliness Changed information provided by a subscription.


class rmw_liveliness_changed_status_t:
    """
    The total number of currently active Publishers which publish to the topic associated with
    the Subscription.
    This count increases when a newly matched Publisher asserts its liveliness for the first time
    or when a Publisher previously considered to be not alive reasserts its liveliness.
    The count decreases when a Publisher considered alive fails to assert its liveliness and
    becomes not alive, whether because it was deleted normally or for some other reason.
    """

    alive_count: int

    # The total count of current Publishers which publish to the topic associated with the
    # Subscription that are no longer asserting their liveliness.
    # This count increases when a Publisher considered alive fails to assert its liveliness and
    # becomes not alive for some reason other than the normal deletion of that Publisher.
    # It decreases when a previously not alive Publisher either reasserts its liveliness or is
    # deleted normally.
    not_alive_count: int

    # The change in the alive_count since the status was last read.
    alive_count_change: int

    # The change in the not_alive_count since the status was last read.
    not_alive_count_change: int


class NodeNameNonExistentError(Exception):
    pass


class Context:
    def __init__(self, args: List[str], domain_id: int) -> None: ...

    def destroy_when_not_in_use(self) -> None: ...

    def __enter__(self) -> None:
        pass

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        pass

    def ok(self) -> bool: ...

    def shutdown(self) -> None: ...

    def get_domain_id(self) -> int: ...


T = TypeVar('T')


class Node:
    def __init__(
        self,
        name: str,
        namespace: str,
        context: Context,
        cli_args: Optional[List[str]],
        use_global_arguments: bool,
        enable_rosout: bool,
        /
    ) -> None: ...

    def logger_name(self) -> str: ...

    def get_parameters(self, cls: Type[T]) -> Dict[str, T]: ...

    def __enter__(self) -> None: ...

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        pass

    def get_node_name(self) -> str: ...

    def get_namespace(self) -> str: ...

    def get_count_subscribers(self, topic: str) -> int: ...

    def get_count_publishers(self, topic: str) -> int: ...

    def get_fully_qualified_name(self) -> str: ...

    def get_node_names_and_namespaces_with_enclaves(self) -> List[Tuple[str, str, str]]: ...

    def get_node_names_and_namespaces(self) -> List[Tuple[str, str]]: ...

    def destroy_when_not_in_use(self) -> None: ...


RCL_DEFAULT_DOMAIN_ID: int = ...


def rclpy_logging_configure(context: Context) -> None: ...


def rclpy_logging_fini() -> None: ...


def rclpy_expand_topic_name(topic_name: str, node_name: str, node_namespace: str) -> str: ...


def rclpy_resolve_name(node: Node, service: str, only_expand: bool, is_service: bool) -> str: ...


class Publisher:

    def __init__(
        self,
        node: Node,
        msg_type: Any,
        topic: str,
        qos_profile: Any,
        qos_overriding_options: Any = ...,
        /
    ) -> None: ...

    def __enter__(self) -> None: ...

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        pass

    def publish(self, msg: Any) -> None: ...

    def publish_raw(self, msg: bytes) -> None: ...

    def get_subscription_count(self) -> int: ...

    def get_topic_name(self) -> str: ...

    def destroy_when_not_in_use(self) -> None: ...

    def get_logger_name(self) -> str: ...


def rclpy_assert_liveliness(publisher: Publisher) -> None: ...


class Subscription:

    def __init__(
        self,
        node: Node,
        msg_type: Any,
        topic: str,
        qos_profile: Any,
        qos_overriding_options: Any = ...,
        /
    ) -> None: ...

    def destroy_when_not_in_use(self) -> None: ...

    def get_logger_name(self) -> str: ...

    def __enter__(self) -> None: ...

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        pass

    def take_message(self, msg_type: Any, raw: bool) -> Optional[Sequence[Any]]: ...

    def get_topic_name(self) -> str: ...

    pointer: Any


SequenceNumber = NewType('SequenceNumber', int)


class Service:
    def __init__(
        self,
        node: Node,
        service_type: Any,
        service_name: str,
        qos_profile: Any,
        qos_overriding_options: Any = ...,
        /
    ) -> None: ...

    def service_send_response(self, response: Any, snd_arg: Any, /) -> None: ...

    def __enter__(self) -> None: ...

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        pass

    def destroy_when_not_in_use(self) -> None: ...

    def service_take_request(self, request: Any) -> Any: ...

    pointer: Any



class Client:
    def __init__(
        self,
        node: Node,
        service_type: Any,
        service_name: str,
        qos_profile: Any,
        qos_overriding_options: Any = ...,
        /
    ) -> None: ...

    def __enter__(self) -> None: ...

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        pass

    def send_request(self, request: Any) -> SequenceNumber: ...

    def service_server_is_available(self) -> bool: ...

    def destroy_when_not_in_use(self) -> None: ...

    def take_response(self, response: Any) -> None: ...

    pointer: Any



class QoSEvent:
    def __init__(self, parent_impl: Any, event_type: Any, /) -> None: ...

    def __enter__(self) -> None: ...

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        pass

    def destroy_when_not_in_use(self) -> None: ...

    def take_event(self) -> Any: ...

    last_policy_kind: QoSPolicyKind

    

class Time:
    nanoseconds: int

class ClockEvent:
    
    def set(self) -> None: ...

    def wait_until_system(self, clock: Clock, time_handle: rcl_time_point_t) -> None: ...
    
    def wait_until_ros(self, clock: Clock, time_handle: rcl_time_point_t) -> None: ...

class Clock:

    def __init__(self, type: ClockType, /) -> None: ...

    def __enter__(self) -> None: ...

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        pass

    def destroy_when_not_in_use(self) -> None: ...

    def remove_clock_callback(self, jh: JumpHandle, /) -> None: ...

    def add_clock_callback(self, jh: JumpHandle, on_clock_change: bool, min_forward: int, min_backward: int, /) -> None: ...

    def get_now(self) -> Time: ...

    def get_ros_time_override_is_enabled(self) -> bool: ...

    def set_ros_time_override_is_enabled(self, enabled: bool, /) -> None: ...

    def set_ros_time_override(self, time_handle: rcl_time_point_t, /) -> Time: ...

    pointer: Any


class Timer:

    def __init__(
        self,
        clock: Clock,
        context: Context,
        period_ns: int,
        /
    ) -> None: ...

    def __enter__(self) -> None: ...

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        pass

    def destroy_when_not_in_use(self) -> None: ...

    def call_timer(self) -> None: ...

    def get_timer_period(self) -> int: ...

    def change_timer_period(self, period_ns: int) -> None: ...

    def is_timer_ready(self) -> bool: ...

    def is_timer_canceled(self) -> bool: ...

    def cancel_timer(self) -> None: ...

    def reset_timer(self) -> None: ...

    def time_since_last_call(self) -> int: ...

    def time_until_next_call(self) -> int: ...

    pointer: Any


class GuardCondition:

    def __init__(self, context: Context, /) -> None: ...

    def __enter__(self) -> None: ...

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        pass

    def destroy_when_not_in_use(self) -> None: ...

    def trigger_guard_condition(self) -> None: ...

    pointer: Any

class WaitSet:
    def __init__(
        self,
        num_subscriptions: int,
        num_guard_conditions: int,
        num_timers: int,
        num_clients: int,
        num_services: int,
        num_events: int,
        context: Optional[Context],
        /
    ) -> None: ...

    def clear_entities(self) -> None: ...

    def add_service(self, service: Service) -> WaitSetIndex: ...

    def add_subscription(self, subscription: Subscription) -> WaitSetIndex: ...

    def add_client(self, client: Client) -> WaitSetIndex: ...

    def add_guard_condition(self, guard_condition: GuardCondition) -> WaitSetIndex: ...

    def add_timer(self, timer: Timer) -> WaitSetIndex: ...

    def add_event(self, qos_event: QoSEvent) -> WaitSetIndex: ...

    def is_ready(self, entity_type: EntityType, index: WaitSetIndex) -> bool: ...

    def get_ready_entities(self, entity_type: EntityType) -> List[Any]: ...

    def wait(self, timeout: int) -> None: ...


def rclpy_get_subscriptions_info_by_topic(node: Node, topic: str, no_mangle: bool) -> Any: ...


def rclpy_remap_topic_name(node: Node, fq_topic_name: str) -> str: ...


def rclpy_get_topic_names_and_types(node: Node, no_demangle: bool) -> List[Tuple[str, List[str]]]: ...


def rclpy_get_service_names_and_types(node: Node) -> List[Tuple[str, List[str]]]: ...


def rclpy_get_client_names_and_types_by_node(
    node: Node, node_name: str, node_namespace: str) -> List[Tuple[str, List[str]]]: ...


def rclpy_get_service_names_and_types_by_node(
    node: Node, node_name: str, node_namespace: str) -> List[Tuple[str, List[str]]]: ...


def rclpy_get_subscriber_names_and_types_by_node(
    node: Node, no_demangle: bool, node_name: str, node_namespace: str) -> List[Tuple[str, List[str]]]: ...


def rclpy_get_publisher_names_and_types_by_node(
    node: Node, no_demangle: bool, node_name: str, node_namespace: str) -> List[Tuple[str, List[str]]]: ...


def rclpy_get_publishers_info_by_topic(node: Node, topic: str, no_mangle: bool) -> List[Dict[Any, Any]]: ...


class rmw_service_info_t:
    request_id: int


class rmw_request_id_t:
    pass


class InvalidHandle(Exception):
    pass


class rcl_duration_t:
    def __init__(self, nanoseconds: int, /) -> None: ...

    nanoseconds: int


RMW_DURATION_INFINITE: int = ...


def rclpy_logging_initialize() -> None: ...


def rclpy_logging_shutdown() -> None: ...


def rclpy_logging_set_logger_level(name: str, level: Any, /) -> None: ...


def rclpy_logging_get_logger_effective_level(name: str, /) -> Any: ...


def rclpy_logging_severity_level_from_string(log_severity: Any, /) -> None: ...


def rclpy_logging_get_logging_directory() -> str: ...


class rmw_message_lost_status_t:
    pass


class rmw_requested_qos_incompatible_event_status_t:
    pass


class rmw_offered_deadline_missed_status_t:
    pass


class rmw_liveliness_lost_status_t:
    pass


class UnsupportedEventTypeError(Exception):
    pass

def rclpy_get_validation_error_for_topic_name(name: str, /) -> Optional[Tuple[str, int]]: ...


class rmw_qos_profile_t:

    def to_dict(self) -> Dict[str, Any]: ...

    @staticmethod
    def predefined(qos_profile: str, /) -> rmw_qos_profile_t: ...

RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE: int = ...

RMW_QOS_DEADLINE_BEST_AVAILABLE: int = ...

def rclpy_action_get_rmw_qos_profile(qos_profile: str, /) -> Dict[str, Any]: ...

class QoSCompatibility:
    def __init__(self, compatibility: Any, /) -> None: ...

class QoSCheckCompatibleResult:
    compatibility: Any
    reason: str

def rclpy_qos_check_compatible(a: Any, b: Any, /) -> QoSCheckCompatibleResult: ...

class ClockType(Enum):
    SYSTEM_TIME = ...
    ROS_TIME = ...
    STEADY_TIME = ...

class ClockChange(Enum):
    ROS_TIME_ACTIVATED = ...
    ROS_TIME_DEACTIVATED = ...

class rcl_time_point_t:
    def __init__(self, nanoseconds: int, clock_type: ClockType) -> None: ...

    nanoseconds: int
    clock_type: ClockType
