# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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
import warnings

from rclpy.duration import Duration
from rclpy.impl.implementation_singleton import rclpy_action_implementation as _rclpy_action
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class QoSProfile:
    """Define Quality of Service policies."""

    __slots__ = [
        '_history',
        '_depth',
        '_reliability',
        '_durability',
        '_lifespan',
        '_deadline',
        '_liveliness',
        '_liveliness_lease_duration',
        '_avoid_ros_namespace_conventions',
    ]

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %r' % kwargs.keys()
        if 'history' not in kwargs:
            warnings.warn(
                "QoSProfile needs a 'history' setting when constructed", DeprecationWarning)
        self.history = kwargs.get(
            'history',
            QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT)
        if (
            QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST == self.history and
            'depth' not in kwargs
        ):
            warnings.warn(
                'A QoSProfile with history set to KEEP_LAST needs a depth specified',
                DeprecationWarning)
        self.depth = kwargs.get('depth', int())
        self.reliability = kwargs.get(
            'reliability',
            QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT)
        self.durability = kwargs.get(
            'durability',
            QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT)
        self.lifespan = kwargs.get('lifespan', Duration())
        self.deadline = kwargs.get('deadline', Duration())
        self.liveliness = kwargs.get(
            'liveliness',
            QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT)
        self.liveliness_lease_duration = kwargs.get('liveliness_lease_duration', Duration())
        self.avoid_ros_namespace_conventions = kwargs.get(
            'avoid_ros_namespace_conventions',
            False)

    @property
    def history(self):
        """
        Get field 'history'.

        :returns: history attribute
        :rtype: QoSHistoryPolicy
        """
        return self._history

    @history.setter
    def history(self, value):
        assert isinstance(value, QoSHistoryPolicy) or isinstance(value, int)
        self._history = QoSHistoryPolicy(value)

    @property
    def reliability(self):
        """
        Get field 'reliability'.

        :returns: reliability attribute
        :rtype: QoSReliabilityPolicy
        """
        return self._reliability

    @reliability.setter
    def reliability(self, value):
        assert isinstance(value, QoSReliabilityPolicy) or isinstance(value, int)
        self._reliability = QoSReliabilityPolicy(value)

    @property
    def durability(self):
        """
        Get field 'durability'.

        :returns: durability attribute
        :rtype: QoSDurabilityPolicy
        """
        return self._durability

    @durability.setter
    def durability(self, value):
        assert isinstance(value, QoSDurabilityPolicy) or isinstance(value, int)
        self._durability = QoSDurabilityPolicy(value)

    @property
    def depth(self):
        """
        Get field 'depth'.

        :returns: depth attribute
        :rtype: int
        """
        return self._depth

    @depth.setter
    def depth(self, value):
        assert isinstance(value, int)
        self._depth = value

    @property
    def lifespan(self):
        """
        Get field 'lifespan'.

        :returns: lifespan attribute
        :rtype: Duration
        """
        return self._lifespan

    @lifespan.setter
    def lifespan(self, value):
        assert isinstance(value, Duration)
        self._lifespan = value

    @property
    def deadline(self):
        """
        Get field 'deadline'.

        :returns: deadline attribute.
        :rtype: Duration
        """
        return self._deadline

    @deadline.setter
    def deadline(self, value):
        assert isinstance(value, Duration)
        self._deadline = value

    @property
    def liveliness(self):
        """
        Get field 'liveliness'.

        :returns: liveliness attribute
        :rtype: QoSLivelinessPolicy
        """
        return self._liveliness

    @liveliness.setter
    def liveliness(self, value):
        assert isinstance(value, (QoSLivelinessPolicy, int))
        self._liveliness = QoSLivelinessPolicy(value)

    @property
    def liveliness_lease_duration(self):
        """
        Get field 'liveliness_lease_duration'.

        :returns: liveliness_lease_duration attribute.
        :rtype: Duration
        """
        return self._liveliness_lease_duration

    @liveliness_lease_duration.setter
    def liveliness_lease_duration(self, value):
        assert isinstance(value, Duration)
        self._liveliness_lease_duration = value

    @property
    def avoid_ros_namespace_conventions(self):
        """
        Get field 'avoid_ros_namespace_conventions'.

        :returns: avoid_ros_namespace_conventions attribute
        :rtype: bool
        """
        return self._avoid_ros_namespace_conventions

    @avoid_ros_namespace_conventions.setter
    def avoid_ros_namespace_conventions(self, value):
        assert isinstance(value, bool)
        self._avoid_ros_namespace_conventions = value

    def get_c_qos_profile(self):
        return _rclpy.rclpy_convert_from_py_qos_policy(
            self.history,
            self.depth,
            self.reliability,
            self.durability,
            self.lifespan.get_c_duration(),
            self.deadline.get_c_duration(),
            self.liveliness,
            self.liveliness_lease_duration.get_c_duration(),
            self.avoid_ros_namespace_conventions,
        )

    def __eq__(self, other):
        if not isinstance(other, QoSProfile):
            return False
        return all(
            self.__getattribute__(slot) == other.__getattribute__(slot)
            for slot in self.__slots__)


class QoSHistoryPolicy(IntEnum):
    """
    Enum for QoS History settings.

    This enum matches the one defined in rmw/types.h
    """

    RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_HISTORY_KEEP_LAST = 1
    RMW_QOS_POLICY_HISTORY_KEEP_ALL = 2


class QoSReliabilityPolicy(IntEnum):
    """
    Enum for QoS Reliability settings.

    This enum matches the one defined in rmw/types.h
    """

    RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_RELIABILITY_RELIABLE = 1
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT = 2


class QoSDurabilityPolicy(IntEnum):
    """
    Enum for QoS Durability settings.

    This enum matches the one defined in rmw/types.h
    """

    RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL = 1
    RMW_QOS_POLICY_DURABILITY_VOLATILE = 2


class QoSLivelinessPolicy(IntEnum):
    """
    Enum for QoS Liveliness settings.

    This enum matches the one defined in rmw/types.h
    """

    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC = 1
    RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE = 2
    RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC = 3


qos_profile_default = _rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_default')
qos_profile_system_default = _rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_system_default')
qos_profile_sensor_data = _rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_sensor_data')
qos_profile_services_default = _rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_services_default')
qos_profile_parameters = _rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_parameters')
qos_profile_parameter_events = _rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_parameter_events')
qos_profile_action_status_default = _rclpy_action.rclpy_action_get_rmw_qos_profile(
    'rcl_action_qos_profile_status_default')
