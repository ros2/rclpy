# Copyright 2016-2021 Open Source Robotics Foundation, Inc.
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

from typing import Any, ClassVar, Iterable, Optional, Protocol, Type, TYPE_CHECKING, TypeVar, Union


from action_msgs.msg._goal_status_array import GoalStatusArray
from action_msgs.srv._cancel_goal import CancelGoal
from builtin_interfaces.msg import Time
from rclpy.exceptions import NoTypeSupportImportedException
from service_msgs.msg._service_event_info import ServiceEventInfo
from unique_identifier_msgs.msg import UUID


if TYPE_CHECKING:
    from typing_extensions import TypeAlias


class PyCapsule(Protocol):
    """Alias for PyCapsule Pybind object."""

    pass


# Done because metaclasses need to inherit from type
ProtocolType: Type = type(Protocol)


class CommonMsgSrvMetaClass(ProtocolType):
    """Shared attributes between messages and services."""

    _TYPE_SUPPORT: Optional[PyCapsule]

    @classmethod
    def __import_type_support__(cls) -> None:
        ...


class MsgMetaClass(CommonMsgSrvMetaClass):
    """Generic Message Metaclass Alias."""

    _CREATE_ROS_MESSAGE:  Optional[PyCapsule]
    _CONVERT_FROM_PY:  Optional[PyCapsule]
    _CONVERT_TO_PY:  Optional[PyCapsule]
    _DESTROY_ROS_MESSAGE:  Optional[PyCapsule]


class Msg(Protocol, metaclass=MsgMetaClass):
    """Generic Message Alias."""

    pass


MsgT = TypeVar('MsgT', bound=Msg, contravariant=True)

SrvRequestT = TypeVar('SrvRequestT', bound=Msg)
SrvResponseT = TypeVar('SrvResponseT', bound=Msg)


class EventMessage(Msg, Protocol[SrvRequestT, SrvResponseT]):
    info: ServiceEventInfo
    request: Iterable[SrvRequestT]
    response: Iterable[SrvResponseT]


class Srv(Protocol[SrvRequestT, SrvResponseT], metaclass=CommonMsgSrvMetaClass):
    """Generic Service Type Alias."""

    Request: Type[SrvRequestT]
    Response: Type[SrvResponseT]
    Event: Type[EventMessage[SrvRequestT, SrvResponseT]]


GoalT = TypeVar('GoalT', bound=Msg)
ResultT = TypeVar('ResultT', bound=Msg)
FeedbackT = TypeVar('FeedbackT', bound=Msg)


class SendGoalServiceRequest(Msg, Protocol[GoalT]):
    goal_id: UUID
    goal: GoalT


class SendGoalServiceResponse(Msg, Protocol):
    accepted: bool
    stamp: Time


SendGoalService: 'TypeAlias' = Srv[SendGoalServiceRequest[GoalT], SendGoalServiceResponse]


class GetResultServiceRequest(Msg, Protocol):
    goal_id: UUID


class GetResultServiceResponse(Msg, Protocol[ResultT]):
    status: int
    result: ResultT


GetResultService: 'TypeAlias' = Srv[GetResultServiceRequest, GetResultServiceResponse[ResultT]]


class FeedbackMessage(Msg, Protocol[FeedbackT]):
    goal_id: UUID
    feedback: FeedbackT


class Impl(Protocol[GoalT, ResultT, FeedbackT]):

    SendGoalService: Type[SendGoalService[GoalT]]
    GetResultService: Type[GetResultService[ResultT]]
    FeedbackMessage: Type[FeedbackMessage[FeedbackT]]
    CancelGoalService: ClassVar[Type[CancelGoal]]
    GoalStatusMessage: ClassVar[Type[GoalStatusArray]]


class Action(Protocol[GoalT,
                      ResultT,
                      FeedbackT],
             metaclass=CommonMsgSrvMetaClass):
    Goal: Type[GoalT]
    Result: Type[ResultT]
    Feedback: Type[FeedbackT]

    Impl: Type[Impl[GoalT, ResultT, FeedbackT]]


# Can be used if https://github.com/python/typing/issues/548 ever gets approved.
SrvT = TypeVar('SrvT', bound=Srv)
ActionT = TypeVar('ActionT', bound=Action)


def check_for_type_support(msg_or_srv_type: Type[Union[Msg, Srv[Any, Any],
                                                       Action[Any, Any, Any]]]) -> None:
    try:
        ts = msg_or_srv_type._TYPE_SUPPORT
    except AttributeError as e:
        e.args = (
            e.args[0] +
            ' This might be a ROS 1 message type but it should be a ROS 2 message type.'
            ' Make sure to source your ROS 2 workspace after your ROS 1 workspace.',
            *e.args[1:])
        raise
    if ts is None:
        msg_or_srv_type.__import_type_support__()
    if msg_or_srv_type._TYPE_SUPPORT is None:
        raise NoTypeSupportImportedException()


def check_is_valid_msg_type(msg_type: Type[Msg]) -> None:
    check_for_type_support(msg_type)
    try:
        assert None not in (
            msg_type._CREATE_ROS_MESSAGE,
            msg_type._CONVERT_FROM_PY,
            msg_type._CONVERT_TO_PY,
            msg_type._DESTROY_ROS_MESSAGE,
        )
    except (AssertionError, AttributeError):
        raise RuntimeError(
            f'The message type provided is not valid ({msg_type}),'
            ' this might be a service or action'
        ) from None


def check_is_valid_srv_type(srv_type: Type[Srv[Any, Any]]) -> None:
    check_for_type_support(srv_type)
    try:
        assert None not in (
            srv_type.Response,
            srv_type.Request,
        )
    except (AssertionError, AttributeError):
        raise RuntimeError(
            f'The service type provided is not valid ({srv_type}),'
            ' this might be a message or action'
        ) from None
