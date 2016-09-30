# Copyright 2016 Open Source Robotics Foundation, Inc.
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
import rclpy


class QoSProfile:
    """Class defining Quality of Service policies."""

    __slots__ = [
        '_history',
        '_depth',
        '_reliability',
        '_durability',
    ]

    def __init__(self, **kwargs):
        assert all(['_' + key in self.__slots__ for key in kwargs.keys()]), \
            "Invalid arguments passed to constructor: %r" % kwargs.keys()
        self.history = kwargs.get(
            'history',
            QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT)
        self.depth = kwargs.get('depth', int())
        self.reliability = kwargs.get(
            'reliability',
            QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT)
        self.durability = kwargs.get(
            'durability',
            QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT)

    @property
    def history(self):
        """Get QoSHistoryPolicy 'history'."""
        return self._history

    @history.setter
    def history(self, value):
        assert isinstance(value, QoSHistoryPolicy)
        self._history = value

    @property
    def reliability(self):
        """Get QoSReliabilityPolicy 'reliability'."""
        return self._reliability

    @reliability.setter
    def reliability(self, value):
        assert isinstance(value, QoSReliabilityPolicy)
        self._reliability = value

    @property
    def durability(self):
        """Get QoSDurabilityPolicy 'durability'."""
        return self._durability

    @durability.setter
    def durability(self, value):
        assert isinstance(value, QoSDurabilityPolicy)
        self._durability = value

    @property
    def depth(self):
        """Get int 'depth'."""
        return self._depth

    @depth.setter
    def depth(self, value):
        assert isinstance(value, int)
        self._depth = value

    def get_c_qos_profile(self):
        return rclpy._rclpy.rclpy_create_qos_policy(
            self.history, self.depth, self.reliability, self.durability)


class QoSHistoryPolicy(IntEnum):
    RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_KEEP_LAST_HISTORY = 1
    RMW_QOS_POLICY_KEEP_ALL_HISTORY = 2


class QoSReliabilityPolicy(IntEnum):
    RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_RELIABLE = 1
    RMW_QOS_POLICY_BEST_EFFORT = 2


class QoSDurabilityPolicy(IntEnum):
    RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_TRANSIENT_LOCAL_DURABILITY = 1
    RMW_QOS_POLICY_VOLATILE_DURABILITY = 2


qos_profile_default = QoSProfile(
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
    depth=10,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT
    # durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT
)

qos_profile_reliable = QoSProfile(
    history=QoSHistoryPolicy.RMW_QOS_POLICY_KEEP_ALL_HISTORY,
    depth=1000,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABLE
    # durability=QoSDurabilityPolicy.RMW_QOS_POLICY_VOLATILE_DURABILITY
)

qos_profile_best_effort = QoSProfile(
    history=QoSHistoryPolicy.RMW_QOS_POLICY_KEEP_LAST_HISTORY,
    depth=5,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_BEST_EFFORT
    # durability=QoSDurabilityPolicy.RMW_QOS_POLICY_TRANSIENT_LOCAL_DURABILITY
)
