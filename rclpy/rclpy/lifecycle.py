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

from typing import List

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.service import Service


class LifecycleMixin:
    def __init__(self, *, enable_communication_interface: bool = True):
        self.create_state_machine(enable_communication_interface)


    def create_state_machine(self, enable_communication_interface: bool):
        with self.handle:
            # self._state_machine = StateMachine(
            #     _rclpy.LifecycleStateMachine(self.handle, enable_communication_interface))
            self._state_machine = _rclpy.LifecycleStateMachine(self.handle, enable_communication_interface)
        self._Node__services.extend(
            [
                self._state_machine.service_change_state,
                self._state_machine.service_get_state,
                self._state_machine.service_get_available_states,
                self._state_machine.service_get_available_transitions,
                self._state_machine.service_get_transition_graph,
            ]
        )


class LifecycleNode(LifecycleMixin, Node):
    def __init__(self, node_name, *, enable_communication_interface: bool = True, **kwargs):
        Node.__init__(self, node_name, **kwargs)
        LifecycleMixin.__init__(self,
            enable_communication_interface=enable_communication_interface)


class StateMachine:
    """Lifecycle state machine implementation."""

    def __init__(self, state_machine_impl: _rclpy.LifecycleStateMachine):
        self._state_machine = state_machine_impl
    
    # def 
