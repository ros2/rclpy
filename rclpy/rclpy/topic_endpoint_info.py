# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

from rclpy.qos import QoSPresetProfiles, QoSProfile


class TopicEndpointTypeEnum(IntEnum):
    """
    Enum for possible types of topic endpoints.

    This enum matches the one defined in rmw/types.h
    """

    INVALID = 0
    PUBLISHER = 1
    SUBSCRIPTION = 2


class TopicEndpointInfo:
    """Information on a topic endpoint."""

    __slots__ = [
        '_node_name',
        '_node_namespace',
        '_topic_type',
        '_endpoint_type',
        '_endpoint_gid',
        '_qos_profile'
    ]

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %r' % kwargs.keys()

        self.node_name = kwargs.get('node_name', '')
        self.node_namespace = kwargs.get('node_namespace', '')
        self.topic_type = kwargs.get('topic_type', '')
        self.endpoint_type = kwargs.get('endpoint_type', TopicEndpointTypeEnum.INVALID)
        self.endpoint_gid = kwargs.get('endpoint_gid', [])
        self.qos_profile = kwargs.get('qos_profile', QoSPresetProfiles.UNKNOWN.value)

    @property
    def node_name(self):
        """
        Get field 'node_name'.

        :returns: node_name attribute
        :rtype: str
        """
        return self._node_name

    @node_name.setter
    def node_name(self, value):
        assert isinstance(value, str)
        self._node_name = value

    @property
    def node_namespace(self):
        """
        Get field 'node_namespace'.

        :returns: node_namespace attribute
        :rtype: str
        """
        return self._node_namespace

    @node_namespace.setter
    def node_namespace(self, value):
        assert isinstance(value, str)
        self._node_namespace = value

    @property
    def topic_type(self):
        """
        Get field 'topic_type'.

        :returns: topic_type attribute
        :rtype: str
        """
        return self._topic_type

    @topic_type.setter
    def topic_type(self, value):
        assert isinstance(value, str)
        self._topic_type = value

    @property
    def endpoint_type(self):
        """
        Get field 'endpoint_type'.

        :returns: endpoint_type attribute
        :rtype: TopicEndpointTypeEnum
        """
        return self._endpoint_type

    @endpoint_type.setter
    def endpoint_type(self, value):
        if isinstance(value, TopicEndpointTypeEnum):
            self._endpoint_type = value
        elif isinstance(value, int):
            self._endpoint_type = TopicEndpointTypeEnum(value)
        else:
            assert False

    @property
    def endpoint_gid(self):
        """
        Get field 'endpoint_gid'.

        :returns: endpoint_gid attribute
        :rtype: list
        """
        return self._endpoint_gid

    @endpoint_gid.setter
    def endpoint_gid(self, value):
        assert all(isinstance(x, int) for x in value)
        self._endpoint_gid = value

    @property
    def qos_profile(self):
        """
        Get field 'qos_profile'.

        :returns: qos_profile attribute
        :rtype: QoSProfile
        """
        return self._qos_profile

    @qos_profile.setter
    def qos_profile(self, value):
        if isinstance(value, QoSProfile):
            self._qos_profile = value
        elif isinstance(value, dict):
            self._qos_profile = QoSProfile(**value)
        else:
            assert False

    def __eq__(self, other):
        if not isinstance(other, TopicEndpointInfo):
            return False
        return all(
            self.__getattribute__(slot) == other.__getattribute__(slot)
            for slot in self.__slots__)

    def __str__(self):
        result = 'Node name: %s\n' % self.node_name
        result += 'Node namespace: %s\n' % self.node_namespace
        result += 'Topic type: %s\n' % self.topic_type
        result += 'Endpoint type: %s\n' % self.endpoint_type.name
        result += 'GID: %s\n' % '.'.join(format(x, '02x') for x in self.endpoint_gid)
        result += 'QoS profile:\n'
        result += '  Reliability: %s\n' % self.qos_profile.reliability.name
        result += '  Durability: %s\n' % self.qos_profile.durability.name
        result += '  Lifespan: %d nanoseconds\n' % self.qos_profile.lifespan.nanoseconds
        result += '  Deadline: %d nanoseconds\n' % self.qos_profile.deadline.nanoseconds
        result += '  Liveliness: %s\n' % self.qos_profile.liveliness.name
        result += '  Liveliness lease duration: %d nanoseconds' % \
            self.qos_profile.liveliness_lease_duration.nanoseconds
        return result
