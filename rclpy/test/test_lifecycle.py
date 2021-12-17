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

import pytest

import rclpy
from rclpy.lifecycle import LifecycleNode


def test_lifecycle_node_init():
    rclpy.init()
    node = LifecycleNode('test_lifecycle_node_init_1')
    assert node
    node.destroy_node()
    node = LifecycleNode('test_lifecycle_node_init_2', enable_communication_interface=False)
    assert node
    # Make sure services were not created
    assert not hasattr(node, '_service_change_state')
    assert not hasattr(node, '_service_get_state')
    assert not hasattr(node, '_service_get_available_states')
    assert not hasattr(node, '_service_get_available_transitions')
    assert not hasattr(node, '_service_get_transition_graph')
    # Make sure also that the services were not created in the pybind11 plugin
    assert not node._state_machine.service_change_state
    assert not node._state_machine.service_get_state
    assert not node._state_machine.service_get_available_states
    assert not node._state_machine.service_get_available_transitions
    assert not node._state_machine.service_get_transition_graph
    node.destroy_node()
    with pytest.raises(TypeError):
        LifecycleNode('test_lifecycle_node_init_3', enable_communication_interface='asd')


if __name__ == '__main__':
    rclpy.init()
    node = LifecycleNode('my_lifecycle_node')
    rclpy.spin(node)
