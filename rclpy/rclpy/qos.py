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

from enum import Enum
from enum import IntEnum

from rclpy.duration import Duration
from rclpy.impl.implementation_singleton import rclpy_action_implementation as _rclpy_action
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class QoSPolicyKind(IntEnum):
    """
    Enum for types of QoS policies that a Publisher or Subscription can set.

    This enum matches the one defined in rmw/incompatible_qos_events_statuses.h
    """

    # TODO(mm3188): obtain these enum values from the rmw layer, instead of hardcoding
    INVALID = 1 << 0
    DURABILITY = 1 << 1
    DEADLINE = 1 << 2
    LIVELINESS = 1 << 3
    RELIABILITY = 1 << 4
    HISTORY = 1 << 5
    LIFESPAN = 1 << 6


def qos_policy_name_from_kind(policy_kind: QoSPolicyKind):
    """Get QoS policy name from QoSPolicyKind enum."""
    if policy_kind == QoSPolicyKind.DURABILITY:
        return 'DURABILITY_QOS_POLICY'
    elif policy_kind == QoSPolicyKind.DEADLINE:
        return 'DEADLINE_QOS_POLICY'
    elif policy_kind == QoSPolicyKind.LIVELINESS:
        return 'LIVELINESS_QOS_POLICY'
    elif policy_kind == QoSPolicyKind.RELIABILITY:
        return 'RELIABILITY_QOS_POLICY'
    elif policy_kind == QoSPolicyKind.HISTORY:
        return 'HISTORY_QOS_POLICY'
    elif policy_kind == QoSPolicyKind.LIFESPAN:
        return 'LIFESPAN_QOS_POLICY'
    else:
        return 'INVALID_QOS_POLICY'


class InvalidQoSProfileException(Exception):
    """Raised when constructing a QoSProfile with invalid arguments."""

    def __init__(self, *args):
        Exception.__init__(self, 'Invalid QoSProfile', *args)


class QoSProfile:
    """Define Quality of Service policies."""

    # default QoS profile not exposed to the user to encourage them to think about QoS settings
    __qos_profile_default_dict = _rclpy.rclpy_get_rmw_qos_profile('qos_profile_default')

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
            if 'depth' not in kwargs:
                raise InvalidQoSProfileException('History and/or depth settings are required.')
            kwargs['history'] = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST

        self.history = kwargs.get('history')

        if (
            QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST == self.history and
            'depth' not in kwargs
        ):
            raise InvalidQoSProfileException('History set to KEEP_LAST without a depth setting.')

        self.depth = kwargs.get('depth', QoSProfile.__qos_profile_default_dict['depth'])
        self.reliability = kwargs.get(
            'reliability', QoSProfile.__qos_profile_default_dict['reliability'])
        self.durability = kwargs.get(
            'durability', QoSProfile.__qos_profile_default_dict['durability'])
        self.lifespan = kwargs.get('lifespan', QoSProfile.__qos_profile_default_dict['lifespan'])
        self.deadline = kwargs.get('deadline', QoSProfile.__qos_profile_default_dict['deadline'])
        self.liveliness = kwargs.get(
            'liveliness', QoSProfile.__qos_profile_default_dict['liveliness'])
        self.liveliness_lease_duration = kwargs.get(
            'liveliness_lease_duration',
            QoSProfile.__qos_profile_default_dict['liveliness_lease_duration'])
        self.avoid_ros_namespace_conventions = kwargs.get(
            'avoid_ros_namespace_conventions',
            QoSProfile.__qos_profile_default_dict['avoid_ros_namespace_conventions'])

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


class QoSPolicyEnum(IntEnum):
    """
    Base for QoS Policy enumerations.

    Provides helper function to filter keys for utilities.
    """

    @classmethod
    def short_keys(cls):
        """Return a list of shortened typing-friendly enum values."""
        return [k.lower() for k in cls.__members__.keys() if not k.startswith('RMW')]

    @classmethod
    def get_from_short_key(cls, name):
        """Retrieve a policy type from a short name, case-insensitive."""
        return cls[name.upper()].value

    @property
    def short_key(self):
        for k, v in self.__class__.__members__.items():
            if k.startswith('RMW'):
                continue
            if self.value == v:
                return k.lower()
        raise AttributeError(
            'failed to find value %s in %s' %
            (self.value, self.__class__.__name__))


class HistoryPolicy(QoSPolicyEnum):
    """
    Enum for QoS History settings.

    This enum matches the one defined in rmw/types.h
    """

    RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT = 0
    SYSTEM_DEFAULT = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT
    RMW_QOS_POLICY_HISTORY_KEEP_LAST = 1
    KEEP_LAST = RMW_QOS_POLICY_HISTORY_KEEP_LAST
    RMW_QOS_POLICY_HISTORY_KEEP_ALL = 2
    KEEP_ALL = RMW_QOS_POLICY_HISTORY_KEEP_ALL
    RMW_QOS_POLICY_HISTORY_UNKNOWN = 3
    UNKNOWN = RMW_QOS_POLICY_HISTORY_UNKNOWN


# Alias with the old name, for retrocompatibility
QoSHistoryPolicy = HistoryPolicy


class ReliabilityPolicy(QoSPolicyEnum):
    """
    Enum for QoS Reliability settings.

    This enum matches the one defined in rmw/types.h
    """

    RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT = 0
    SYSTEM_DEFAULT = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT
    RMW_QOS_POLICY_RELIABILITY_RELIABLE = 1
    RELIABLE = RMW_QOS_POLICY_RELIABILITY_RELIABLE
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT = 2
    BEST_EFFORT = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
    RMW_QOS_POLICY_RELIABILITY_UNKNOWN = 3
    UNKNOWN = RMW_QOS_POLICY_RELIABILITY_UNKNOWN


# Alias with the old name, for retrocompatibility
QoSReliabilityPolicy = ReliabilityPolicy


class DurabilityPolicy(QoSPolicyEnum):
    """
    Enum for QoS Durability settings.

    This enum matches the one defined in rmw/types.h
    """

    RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT = 0
    SYSTEM_DEFAULT = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL = 1
    TRANSIENT_LOCAL = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
    RMW_QOS_POLICY_DURABILITY_VOLATILE = 2
    VOLATILE = RMW_QOS_POLICY_DURABILITY_VOLATILE
    RMW_QOS_POLICY_DURABILITY_UNKNOWN = 3
    UNKNOWN = RMW_QOS_POLICY_DURABILITY_UNKNOWN


# Alias with the old name, for retrocompatibility
QoSDurabilityPolicy = DurabilityPolicy


class LivelinessPolicy(QoSPolicyEnum):
    """
    Enum for QoS Liveliness settings.

    This enum matches the one defined in rmw/types.h
    """

    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT = 0
    SYSTEM_DEFAULT = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC = 1
    AUTOMATIC = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
    RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC = 3
    MANUAL_BY_TOPIC = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC
    RMW_QOS_POLICY_LIVELINESS_UNKNOWN = 4
    UNKNOWN = RMW_QOS_POLICY_LIVELINESS_UNKNOWN


# Alias with the old name, for retrocompatibility
QoSLivelinessPolicy = LivelinessPolicy

qos_profile_unknown = QoSProfile(**_rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_unknown'))
qos_profile_system_default = QoSProfile(**_rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_system_default'))
qos_profile_sensor_data = QoSProfile(**_rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_sensor_data'))
qos_profile_services_default = QoSProfile(**_rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_services_default'))
qos_profile_parameters = QoSProfile(**_rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_parameters'))
qos_profile_parameter_events = QoSProfile(**_rclpy.rclpy_get_rmw_qos_profile(
    'qos_profile_parameter_events'))
qos_profile_action_status_default = QoSProfile(**_rclpy_action.rclpy_action_get_rmw_qos_profile(
    'rcl_action_qos_profile_status_default'))


class QoSPresetProfiles(Enum):
    UNKNOWN = qos_profile_unknown
    SYSTEM_DEFAULT = qos_profile_system_default
    SENSOR_DATA = qos_profile_sensor_data
    SERVICES_DEFAULT = qos_profile_services_default
    PARAMETERS = qos_profile_parameters
    PARAMETER_EVENTS = qos_profile_parameter_events
    ACTION_STATUS_DEFAULT = qos_profile_action_status_default

    """Noted that the following are duplicated from QoSPolicyEnum.

    Our supported version of Python3 (3.5) doesn't have a fix that allows mixins on Enum.
    """
    @classmethod
    def short_keys(cls):
        """Return a list of shortened typing-friendly enum values."""
        return [k.lower() for k in cls.__members__.keys() if not k.startswith('RMW')]

    @classmethod
    def get_from_short_key(cls, name):
        """Retrieve a policy type from a short name, case-insensitive."""
        return cls[name.upper()].value
