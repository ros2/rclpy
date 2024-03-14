# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


def expand_topic_name(topic_name: str, node_name: str, node_namespace: str) -> str:
    """
    Expand a given topic name using given node name and namespace as well.

    Note that this function can succeed but the expanded topic name may still
    be invalid.
    The :py:func:validate_full_topic_name(): should be used on the expanded
    topic name to ensure it is valid after expansion.

    :param topic_name: topic name to be expanded
    :param node_name: name of the node that this topic is associated with
    :param namespace: namespace that the topic is within
    :returns: expanded topic name which is fully qualified
    :raises: ValueError if the topic name, node name or namespace are invalid
    """
    return _rclpy.rclpy_expand_topic_name(topic_name, node_name, node_namespace)
