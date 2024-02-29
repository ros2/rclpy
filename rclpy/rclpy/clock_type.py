# Copyright 2024 Open Source Robotics Foundation, Inc.
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

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class ClockType(IntEnum):
    """
    Enum for clock type.

    This enum must match the one defined in rclpy/_rclpy_pybind11.cpp.
    """

    UNINITIALIZED = _rclpy.ClockType.UNINITIALIZED
    ROS_TIME = _rclpy.ClockType.ROS_TIME
    SYSTEM_TIME = _rclpy.ClockType.SYSTEM_TIME
    STEADY_TIME = _rclpy.ClockType.STEADY_TIME
