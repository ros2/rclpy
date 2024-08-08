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

from threading import Thread
from unittest import mock

import lifecycle_msgs.msg
import lifecycle_msgs.srv

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.node import Node
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
    # test many times back to back, to make sure it works robustly
    for _ in range(10):
        assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    for _ in range(10):
        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS
    # some that are not possible from the current state
    with pytest.raises(_rclpy.RCLError):
        node.trigger_activate()
    with pytest.raises(_rclpy.RCLError):
        node.trigger_deactivate()
    assert node.trigger_shutdown() == TransitionCallbackReturn.SUCCESS
    with pytest.raises(_rclpy.RCLError):
        node.trigger_shutdown()
    node.destroy_node()
    # Again but trigger shutdown from 'inactive' instead of 'unconfigured'
    node = LifecycleNode(
        'test_lifecycle_state_transitions_2', enable_communication_interface=False)
    assert node.trigger_shutdown() == TransitionCallbackReturn.SUCCESS
    # Again but trigger shutdown from 'active'
    node = LifecycleNode(
        'test_lifecycle_state_transitions_3', enable_communication_interface=False)
    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_shutdown() == TransitionCallbackReturn.SUCCESS

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


def test_lifecycle_services(request):
    lc_node_name = 'test_lifecycle_services_lifecycle'
    lc_node = LifecycleNode(lc_node_name)
    client_node = Node('test_lifecycle_services_client')
    get_state_cli = client_node.create_client(
        lifecycle_msgs.srv.GetState,
        f'/{lc_node_name}/get_state')
    change_state_cli = client_node.create_client(
        lifecycle_msgs.srv.ChangeState,
        f'/{lc_node_name}/change_state')
    get_available_states_cli = client_node.create_client(
        lifecycle_msgs.srv.GetAvailableStates,
        f'/{lc_node_name}/get_available_states')
    get_available_transitions_cli = client_node.create_client(
        lifecycle_msgs.srv.GetAvailableTransitions,
        f'/{lc_node_name}/get_available_transitions')
    get_transition_graph_cli = client_node.create_client(
        lifecycle_msgs.srv.GetAvailableTransitions,
        f'/{lc_node_name}/get_transition_graph')
    for cli in (
        get_state_cli,
        change_state_cli,
        get_available_states_cli,
        get_available_transitions_cli,
        get_transition_graph_cli,
    ):
        assert cli.wait_for_service(5.)
    # lunch a thread to spin the executor, so we can make sync service calls easily
    executor = SingleThreadedExecutor()
    executor.add_node(client_node)
    executor.add_node(lc_node)
    thread = Thread(target=executor.spin)
    thread.start()

    def cleanup():
        # Stop executor and join thread.
        # This cleanup is run even if an assertion fails.
        executor.shutdown()
        thread.join()
    request.addfinalizer(cleanup)

    # test all services
    req = lifecycle_msgs.srv.GetState.Request()
    resp = get_state_cli.call(req)
    assert resp.current_state.label == 'unconfigured'
    req = lifecycle_msgs.srv.ChangeState.Request()
    req.transition.id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
    resp = change_state_cli.call(req)
    assert resp.success
    req = lifecycle_msgs.srv.GetState.Request()
    resp = get_state_cli.call(req)
    assert resp.current_state.label == 'inactive'
    req = lifecycle_msgs.srv.GetAvailableStates.Request()
    resp = get_available_states_cli.call(req)
    states_labels = {state.label for state in resp.available_states}
    assert states_labels == {
        'unknown', 'unconfigured', 'inactive', 'active', 'finalized', 'configuring', 'cleaningup',
        'shuttingdown', 'activating', 'deactivating', 'errorprocessing'
    }
    req = lifecycle_msgs.srv.GetAvailableTransitions.Request()
    resp = get_available_transitions_cli.call(req)
    transitions_labels = {
        transition_def.transition.label for transition_def in resp.available_transitions}
    assert transitions_labels == {'activate', 'cleanup', 'shutdown'}
    req = lifecycle_msgs.srv.GetAvailableTransitions.Request()
    resp = get_transition_graph_cli.call(req)
    transitions_labels = {
        transition_def.transition.label for transition_def in resp.available_transitions}
    assert transitions_labels == {
        'configure', 'activate', 'cleanup', 'shutdown', 'deactivate', 'transition_error',
        'transition_failure', 'transition_success'
    }


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
