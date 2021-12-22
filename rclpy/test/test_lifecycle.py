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

from unittest import mock

import pytest

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.publisher import Publisher

from test_msgs.msg import BasicTypes


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


def test_lifecycle_state_transitions():
    node = LifecycleNode(
        'test_lifecycle_state_transitions_1', enable_communication_interface=False)
    # normal transitions
    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS
    # some that are not possible from the current state
    with pytest.raises(_rclpy.RCLError):
        node.trigger_activate()
    with pytest.raises(_rclpy.RCLError):
        node.trigger_deactivate()
    assert node.trigger_shutdown() == TransitionCallbackReturn.SUCCESS
    node.destroy_node()

    class ErrorOnConfigureHandledCorrectlyNode(LifecycleNode):

        def on_configure(self):
            return TransitionCallbackReturn.ERROR

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)

    node = ErrorOnConfigureHandledCorrectlyNode(
        'test_lifecycle_state_transitions_2', enable_communication_interface=False)
    assert node._state_machine.current_state[1] == 'unconfigured'
    assert node.trigger_configure() == TransitionCallbackReturn.ERROR
    assert node._state_machine.current_state[1] == 'unconfigured'

    class ErrorOnConfigureHandledInCorrectlyNode(LifecycleNode):

        def on_configure(self):
            return TransitionCallbackReturn.ERROR

        def on_error(self):
            return TransitionCallbackReturn.ERROR

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)

    node = ErrorOnConfigureHandledInCorrectlyNode(
        'test_lifecycle_state_transitions_3', enable_communication_interface=False)
    assert node.trigger_configure() == TransitionCallbackReturn.ERROR
    assert node._state_machine.current_state[1] == 'finalized'

# TODO(ivanpauno): Add automated tests for lifecycle services!!


def test_lifecycle_publisher():
    node = LifecycleNode('test_lifecycle_publisher', enable_communication_interface=False)
    with mock.patch.object(Publisher, 'publish') as mock_publish:
        pub = node.create_lifecycle_publisher(BasicTypes, 'test_lifecycle_publisher_topic', 10)
        pub.publish(BasicTypes())
        mock_publish.assert_not_called()
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        node.trigger_activate()
        msg = BasicTypes()
        pub.publish(msg)
        mock_publish.assert_called()
        mock_publish.assert_called_with(pub, msg)


# TODO(ivanpauno): DELETE THIS BEFORE MERGING
# CREATE A lifecycle_py demo
if __name__ == '__main__':
    rclpy.init()
    node = LifecycleNode('my_lifecycle_node')
    rclpy.spin(node)
