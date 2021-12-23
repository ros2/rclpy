# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from .managed_entity import ManagedEntity
from .managed_entity import SimpleManagedEntity
from .node import LifecycleNode
from .node import LifecycleNodeMixin
from .node import LifecycleState
from .publisher import LifecyclePublisher

from ..impl.implementation_singleton import rclpy_implementation as _rclpy

# reexport LifecycleNode as Node, so it's possible to write:
# from rclpy.lifecycle import Node
# Do not include that in __all__ to avoid mixing it up with rclpy.node.Node.
Node = LifecycleNode
# same idea here
NodeMixin = LifecycleNodeMixin
State = LifecycleState
Publisher = LifecyclePublisher

# enum defined in pybind11 plugin
TransitionCallbackReturn = _rclpy.TransitionCallbackReturnType


__all__ = [
    'LifecycleNodeMixin',
    'LifecycleNode',
    'LifecycleState',
    'LifecyclePublisher',
    'ManagedEntity',
    'SimpleManagedEntity',
]
