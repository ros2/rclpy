# Copyright 2020 Amazon.com, Inc. or its affiliates.
# Copyright 2025 Minju Lee (이민주).
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

from enum import IntEnum
from typing import Any, List, Union

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSHistoryPolicy, QoSPresetProfiles, QoSProfile
from rclpy.type_hash import TypeHash, TypeHashDictionary


class EndpointTypeEnum(IntEnum):
    """
    Enum for possible types of topic endpoints.

    This enum matches the one defined in rmw/types.h
    """

    INVALID = 0
    PUBLISHER = 1
    SUBSCRIPTION = 2
    CLIENT = 3
    SERVER = 4


class TopicEndpointInfo:
    """Information on a topic endpoint."""

    __slots__ = [
        '_node_name',
        '_node_namespace',
        '_topic_type',
        '_topic_type_hash',
        '_endpoint_type',
        '_endpoint_gid',
        '_qos_profile'
    ]

    def __init__(
        self,
        node_name: str = '',
        node_namespace: str = '',
        topic_type: str = '',
        topic_type_hash: Union[TypeHash, TypeHashDictionary] = TypeHash(),
        endpoint_type: Union[EndpointTypeEnum, int] = EndpointTypeEnum.INVALID,
        endpoint_gid: List[int] = [],
        qos_profile: Union[QoSProfile, '_rclpy._rmw_qos_profile_dict'] =
            QoSPresetProfiles.UNKNOWN.value
    ):
        self.node_name = node_name
        self.node_namespace = node_namespace
        self.topic_type = topic_type
        self.topic_type_hash = topic_type_hash
        self.endpoint_type = endpoint_type
        self.endpoint_gid = endpoint_gid
        self.qos_profile = qos_profile

    @property
    def node_name(self) -> str:
        """
        Get field 'node_name'.

        :returns: node_name attribute
        """
        return self._node_name

    @node_name.setter
    def node_name(self, value: str) -> None:
        assert isinstance(value, str)
        self._node_name = value

    @property
    def node_namespace(self) -> str:
        """
        Get field 'node_namespace'.

        :returns: node_namespace attribute
        """
        return self._node_namespace

    @node_namespace.setter
    def node_namespace(self, value: str) -> None:
        assert isinstance(value, str)
        self._node_namespace = value

    @property
    def topic_type(self) -> str:
        """
        Get field 'topic_type'.

        :returns: topic_type attribute
        """
        return self._topic_type

    @topic_type.setter
    def topic_type(self, value: str) -> None:
        assert isinstance(value, str)
        self._topic_type = value

    # Has to be marked Any due to mypy#3004. Return type is actually TypeHash
    @property
    def topic_type_hash(self) -> Any:
        """
        Get field 'topic_type_hash'.

        :returns: topic_type_hash attribute
        """
        return self._topic_type_hash

    @topic_type_hash.setter
    def topic_type_hash(self, value: Union[TypeHash, TypeHashDictionary]) -> None:
        if isinstance(value, TypeHash):
            self._topic_type_hash = value
        elif isinstance(value, dict):
            self._topic_type_hash = TypeHash(**value)
        else:
            assert False

    # Has to be marked Any due to mypy#3004. Return type is actually EndpointTypeEnum
    @property
    def endpoint_type(self) -> Any:
        """
        Get field 'endpoint_type'.

        :returns: endpoint_type attribute
        """
        return self._endpoint_type

    @endpoint_type.setter
    def endpoint_type(self, value: Union[EndpointTypeEnum, int]) -> None:
        if isinstance(value, EndpointTypeEnum):
            self._endpoint_type = value
        elif isinstance(value, int):
            self._endpoint_type = EndpointTypeEnum(value)
        else:
            assert False

    @property
    def endpoint_gid(self) -> List[int]:
        """
        Get field 'endpoint_gid'.

        :returns: endpoint_gid attribute
        """
        return self._endpoint_gid

    @endpoint_gid.setter
    def endpoint_gid(self, value: List[int]) -> None:
        assert all(isinstance(x, int) for x in value)
        self._endpoint_gid = value

    # Has to be marked Any due to mypy#3004. Return type is actually QoSProfile
    @property
    def qos_profile(self) -> Any:
        """
        Get field 'qos_profile'.

        :returns: qos_profile attribute
        """
        return self._qos_profile

    @qos_profile.setter
    def qos_profile(self, value: Union[QoSProfile, '_rclpy._rmw_qos_profile_dict']) -> None:
        if isinstance(value, QoSProfile):
            self._qos_profile = value
        elif isinstance(value, dict):
            self._qos_profile = QoSProfile(**value)
        else:
            assert False

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, TopicEndpointInfo):
            return False
        return all(
            self.__getattribute__(slot) == other.__getattribute__(slot)
            for slot in self.__slots__)

    def __str__(self) -> str:
        gid = '.'.join(format(x, '02x') for x in self.endpoint_gid)
        if self.qos_profile.history.value != QoSHistoryPolicy.KEEP_LAST:
            history_depth_str = self.qos_profile.history.name
        else:
            history_depth_str = f'{self.qos_profile.history.name} ({self.qos_profile.depth})'
        return '\n'.join([
            f'Node name: {self.node_name}',
            f'Node namespace: {self.node_namespace}',
            f'Topic type: {self.topic_type}',
            f'Topic type hash: {self.topic_type_hash}',
            f'Endpoint type: {self.endpoint_type.name}',
            f'GID: {gid}',
            'QoS profile:',
            f'  Reliability: {self.qos_profile.reliability.name}',
            f'  History (Depth): {history_depth_str}',
            f'  Durability: {self.qos_profile.durability.name}',
            f'  Lifespan: {self.qos_profile.lifespan}',
            f'  Deadline: {self.qos_profile.deadline}',
            f'  Liveliness: {self.qos_profile.liveliness.name}',
            f'  Liveliness lease duration: {self.qos_profile.liveliness_lease_duration}',
        ])


class ServiceEndpointInfo:
    """Information on a service endpoint."""

    __slots__ = [
        '_node_name',
        '_node_namespace',
        '_service_type',
        '_service_type_hash',
        '_qos_profiles',
        '_endpoint_gids',
        '_endpoint_type',
        '_endpoint_count'
    ]

    def __init__(
        self,
        node_name: str = '',
        node_namespace: str = '',
        service_type: str = '',
        service_type_hash: Union[TypeHash, TypeHashDictionary] = TypeHash(),
        qos_profiles: List[Union[QoSProfile, '_rclpy._rmw_qos_profile_dict']] = [],
        endpoint_gids: List[List[int]] = [],
        endpoint_type: EndpointTypeEnum = EndpointTypeEnum.INVALID,
        endpoint_count: int = 0
    ):
        self.node_name = node_name
        self.node_namespace = node_namespace
        self.service_type = service_type
        self.service_type_hash = service_type_hash
        self.endpoint_type = endpoint_type
        self.endpoint_gids = endpoint_gids
        self.qos_profiles = qos_profiles
        self.endpoint_count = endpoint_count

    @property
    def node_name(self) -> str:
        """
        Get field 'node_name'.

        :returns: node_name attribute
        """
        return self._node_name

    @node_name.setter
    def node_name(self, value: str) -> None:
        assert isinstance(value, str)
        self._node_name = value

    @property
    def node_namespace(self) -> str:
        """
        Get field 'node_namespace'.

        :returns: node_namespace attribute
        """
        return self._node_namespace

    @node_namespace.setter
    def node_namespace(self, value: str) -> None:
        assert isinstance(value, str)
        self._node_namespace = value

    @property
    def service_type(self) -> str:
        """
        Get field 'topic_type'.

        :returns: topic_type attribute
        """
        return self._service_type

    @service_type.setter
    def service_type(self, value: str) -> None:
        assert isinstance(value, str)
        self._service_type = value

    @property
    def service_type_hash(self) -> Any:
        """
        Get field 'service_type_hash'.

        :returns: service_type_hash attribute
        """
        return self._service_type_hash

    @service_type_hash.setter
    def service_type_hash(self, value: Union[TypeHash, TypeHashDictionary]) -> None:
        if isinstance(value, TypeHash):
            self._service_type_hash = value
        elif isinstance(value, dict):
            self._service_type_hash = TypeHash(**value)
        else:
            assert False

    @property
    def endpoint_type(self) -> EndpointTypeEnum:
        """
        Get field 'endpoint_type'.

        :returns: endpoint_type attribute
        """
        return self._endpoint_type

    @endpoint_type.setter
    def endpoint_type(self, value: Union[EndpointTypeEnum, int]) -> None:
        if isinstance(value, EndpointTypeEnum):
            self._endpoint_type = value
        elif isinstance(value, int):
            self._endpoint_type = EndpointTypeEnum(value)
        else:
            assert False

    @property
    def endpoint_count(self) -> int:
        """
        Get field 'node_name'.

        :returns: node_name attribute
        """
        return self._endpoint_count

    @endpoint_count.setter
    def endpoint_count(self, value: int) -> None:
        assert isinstance(value, int)
        self._endpoint_count = value

    @property
    def endpoint_gids(self) -> List[List[int]]:
        """
        Get field 'endpoint_gids'.

        :returns: endpoint_gids attribute
        """
        return self._endpoint_gids

    @endpoint_gids.setter
    def endpoint_gids(self, value: List[List[int]]) -> None:
        assert isinstance(value, list)
        assert all(isinstance(x, list) and all(isinstance(i, int) for i in x) for x in value)
        self._endpoint_gids = value

    @property
    def qos_profiles(self) -> List[QoSProfile]:
        """
        Get field 'qos_profile'.

        :returns: qos_profile attribute
        """
        return self._qos_profiles

    @qos_profiles.setter
    def qos_profiles(self, value: List[Union[QoSProfile, '_rclpy.rmw_qos_profile_dict']]) -> None:
        assert isinstance(value, list)
        self._qos_profiles = [v if isinstance(v, QoSProfile) else QoSProfile(**v) for v in value]

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, ServiceEndpointInfo):
            return False
        return all(
            self.__getattribute__(slot) == other.__getattribute__(slot)
            for slot in self.__slots__)

    def __str__(self) -> str:
        gids = []
        for endpoint_gid in self.endpoint_gids:
            gid = '.'.join(format(x, '02x') for x in endpoint_gid)
            gids.append(gid)

        def format_qos(qos, indent='  ') -> str:
            return '\n'.join([
                f'{indent}Reliability: {qos.reliability.name}',
                f'{indent}History (Depth): {qos.history.name} ({qos.depth})',
                f'{indent}Durability: {qos.durability.name}',
                f'{indent}Lifespan: {qos.lifespan}',
                f'{indent}Deadline: {qos.deadline}',
                f'{indent}Liveliness: {qos.liveliness.name}',
                f'{indent}Liveliness lease duration: {qos.liveliness_lease_duration}'
            ])
        info_lines = [
            f'Node name: {self.node_name}',
            f'Node namespace: {self.node_namespace}',
            f'Service type: {self.service_type}',
            f'Service type hash: {self.service_type_hash}',
            f'Endpoint type: {self.endpoint_type.name}',
            f'Endpoint count: {self.endpoint_count}',
        ]

        if 1 == self.endpoint_count:
            info_lines.append(f'GID: {gids[0]}')
            info_lines.append('QoS profile:')
            info_lines.append(format_qos(self.qos_profiles[0]))
        elif self.endpoint_type == EndpointTypeEnum.CLIENT:
            info_lines += [
                'GIDs:',
                f' - Request Writer : {gids[1]}',
                f' - Response Reader : {gids[0]}',
                'QoS profiles:',
                ' - Request Writer :',
                format_qos(self.qos_profiles[1], indent='      '),
                ' - Response Reader :',
                format_qos(self.qos_profiles[0], indent='      '),
            ]
        else:
            info_lines += [
                'GIDs:',
                f' - Request Reader : {gids[0]}',
                f' - Response Writer : {gids[1]}',
                'QoS profiles:',
                ' - Request Reader :',
                format_qos(self.qos_profiles[0], indent='      '),
                ' - Response Writer :',
                format_qos(self.qos_profiles[1], indent='      '),
            ]

        return '\n'.join(info_lines)
