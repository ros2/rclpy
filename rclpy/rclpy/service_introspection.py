# Copyright 2023 Open Source Robotics Foundation, Inc.
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


class ServiceIntrospectionState(IntEnum):
    OFF = _rclpy.service_introspection.ServiceIntrospectionState.OFF
    METADATA = _rclpy.service_introspection.ServiceIntrospectionState.METADATA
    CONTENTS = _rclpy.service_introspection.ServiceIntrospectionState.CONTENTS
