# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from typing import List
from typing import Tuple

from rclpy.node import Node


def get_action_client_names_and_types_by_node(
    node: Node,
    remote_node_name: str,
    remote_node_namespace: str
) -> List[Tuple[str, List[str]]]:
    """
    Get a list of action names and types for action clients associated with a node.

    :param node: The node used for discovery.
    :param remote_node_name: The name of a remote node to get action clients for.
    :param node_namespace: Namespace of the remote node.
    :return: List of tuples.
      The first element of each tuple is the action name and the second element is a list of
      action types.
    """
    with node.handle:
        return node.handle.get_action_client_names_and_types_by_node(
            remote_node_name, remote_node_namespace)


def get_action_server_names_and_types_by_node(
    node: Node,
    remote_node_name: str,
    remote_node_namespace: str
) -> List[Tuple[str, List[str]]]:
    """
    Get a list of action names and types for action servers associated with a node.

    :param node: The node used for discovery.
    :param remote_node_name: The name of a remote node to get action servers for.
    :param node_namespace: Namespace of the remote node.
    :return: List of tuples.
      The first element of each tuple is the action name and the second element is a list of
      action types.
    """
    with node.handle:
        return node.handle.get_action_server_names_and_types_by_node(
            remote_node_name, remote_node_namespace)


def get_action_names_and_types(node: Node) -> List[Tuple[str, List[str]]]:
    """
    Get a list of action names and types.

    :param node: The node used for discovery.
    :return: List of action names and types in the ROS graph as tuples.
      The first element of each tuple is the action name and the second element is a list of
      action types.
    """
    with node.handle:
        return node.handle.get_action_names_and_types()
