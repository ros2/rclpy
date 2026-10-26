# Injects the imports needed by the custom nb::sig() signatures in the C++ bindings,
# which reference rclpy and action_msgs types by their short names.
# The entry below matches the module-level TypeVar ``T`` and re-declares it verbatim.
# The \from escapes add the imports to the top of the generated stub.
# The TypedDict / TypeAlias declarations that follow describe plain Python
# containers returned by the bindings; they are not nanobind types, so they
# cannot be generated from the extension module and are declared here instead.
_rclpy_nanobind\.T$:
    \from typing import Any
    \from typing import Literal
    \from typing import TypeAlias
    \from typing import TypedDict
    \from typing import TypeVar
    \from typing import overload
    \from collections.abc import Callable
    \from collections.abc import Sequence
    \from action_msgs.msg import GoalInfo
    \from action_msgs.msg import GoalStatusArray
    \from action_msgs.srv._cancel_goal import CancelGoal_Request
    \from action_msgs.srv._cancel_goal import CancelGoal_Response
    \from rclpy.duration import Duration
    \from rclpy.node import Node as RCLPyNode
    \from rclpy.parameter import Parameter
    \from rclpy.subscription import MessageInfo
    \from rclpy.subscription_content_filter_options import ContentFilterOptions
    \from rclpy.task import Task
    \from rclpy.type_support import Action
    \from rclpy.type_support import FeedbackMessage
    \from rclpy.type_support import FeedbackT
    \from rclpy.type_support import GetResultServiceRequest
    \from rclpy.type_support import GetResultServiceResponse
    \from rclpy.type_support import GoalT
    \from rclpy.type_support import ImplT
    \from rclpy.type_support import MsgT
    \from rclpy.type_support import ResultT
    \from rclpy.type_support import SendGoalServiceRequest
    \from rclpy.type_support import SendGoalServiceResponse
    \from rclpy.type_support import Srv
    \from rclpy.type_support import SrvRequestT
    \from rclpy.type_support import SrvResponseT
    \from type_description_interfaces.srv._get_type_description import GetTypeDescription_Request
    \from type_description_interfaces.srv._get_type_description import GetTypeDescription_Response
    T = TypeVar("T")

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

    class _ServiceEndpointInfoDict(TypedDict):
        node_name: str
        node_namespace: str
        service_type: str
        service_type_hash: _TypeHashDict
        qos_profiles: list[_rmw_qos_profile_dict]
        endpoint_gids: list[list[int]]
        endpoint_type: int
        endpoint_count: int

    class _ActionEndpointInfoDict(TypedDict):
        goal_service_info: _ServiceEndpointInfoDict | None
        cancel_service_info: _ServiceEndpointInfoDict | None
        result_service_info: _ServiceEndpointInfoDict | None
        feedback_topic_info: _TopicEndpointInfoDict | None
        status_topic_info: _TopicEndpointInfoDict | None

    class _TimeInfoDict(TypedDict):
        expected_call_time: int
        actual_call_time: int

    # rmw's offered/requested incompatible QoS status structs are the same C type.
    _rmw_offered_qos_incompatible_event_status_t: TypeAlias = rmw_requested_qos_incompatible_event_status_t

# nanobind registers this exception with the same name as the builtin it derives
# from, which stubgen would otherwise render as a self-referential class.
_rclpy_nanobind\.NotImplementedError$:
    \from builtins import NotImplementedError as _BuiltinNotImplementedError
    class NotImplementedError(_BuiltinNotImplementedError):
        pass

# The raw flag selects between a deserialized message and its serialized bytes;
# a single nb::sig() cannot express that, so spell out the overloads here.
_rclpy_nanobind\.Subscription\.take_message$:
    @overload
    def take_message(self, pymsg_type: type[MsgT], raw: Literal[True], /) -> tuple[bytes, MessageInfo] | None: ...

    @overload
    def take_message(self, pymsg_type: type[MsgT], raw: Literal[False], /) -> tuple[MsgT, MessageInfo] | None: ...

    @overload
    def take_message(self, pymsg_type: type[MsgT], raw: bool, /) -> tuple[MsgT | bytes, MessageInfo] | None:
        \doc
