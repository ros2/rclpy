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

from typing import Any, ClassVar, Protocol, Type, TypeVar, Union

from rclpy.exceptions import NoTypeSupportImportedException


class CommonMsgSrv(Protocol):
    """Shared attributes between messages and services."""

    _TYPE_SUPPORT: Any

    @classmethod
    def __import_type_support__(cls) -> None:
        ...


class Msg(CommonMsgSrv):
    """Generic Message Type Alias."""

    _CREATE_ROS_MESSAGE: Any
    _CONVERT_FROM_PY: Any
    _CONVERT_TO_PY: Any
    _DESTROY_ROS_MESSAGE: Any


class Srv(CommonMsgSrv):
    """Generic Service Type Alias."""

    Request: ClassVar = Msg
    Response: ClassVar = Msg
    Event: ClassVar = Msg


MsgType = TypeVar('MsgType', bound=Msg)
SrvType = TypeVar('SrvType', bound=Srv)


def check_for_type_support(msg_or_srv_type: Type[Union[Msg, Srv]]) -> None:
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


def check_is_valid_srv_type(srv_type: Type[Srv]) -> None:
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
