# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import threading
from unittest.mock import MagicMock
from unittest.mock import patch

from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy_components.component_manager import ComponentManager


class FakeEntryPoint:
    name: str
    value: str
    group: str
    mock_class: MagicMock

    def __init__(self, name: str, value: str, group: str = 'rclpy_components'):
        self.name = name
        self.value = value
        self.group = group
        self.mock_class = MagicMock()

    def load(self):
        return self.mock_class


class FakeEntryPointFailsToLoad(FakeEntryPoint):

    def load(self):
        raise ValueError('Oops I failed to load')


EXECUTORS = (SingleThreadedExecutor, MultiThreadedExecutor)


@pytest.fixture(params=EXECUTORS)
def component_manager(request):
    rclpy.init()
    executor = request.param()
    component_manager = ComponentManager(executor, 'ut_container')
    executor.add_node(component_manager)
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    yield component_manager

    executor.remove_node(component_manager)
    component_manager.destroy_node()
    executor.shutdown()
    t.join()
    rclpy.shutdown()


@pytest.fixture()
def node_and_context():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_node', namespace='test_component_manager', context=context)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    yield (node, context)

    node.destroy_node()
    executor.shutdown()
    t.join()
    rclpy.shutdown(context=context)


def test_list_nodes_no_nodes(component_manager):
    res = ListNodes.Response()
    req = ListNodes.Request()
    component_manager.on_list_node(req, res)
    assert len(res.full_node_names) == 0
    assert len(res.unique_ids) == 0


def test_load_node_no_such_node(component_manager):
    res = LoadNode.Response()
    req = LoadNode.Request()
    req.package_name = 'fake_package'
    req.plugin_name = 'FakePluginName'

    component_manager.on_load_node(req, res)

    assert 'not found' in res.error_message
    assert res.success is False


def test_load_then_unload_node(component_manager):
    # Load
    load_res = LoadNode.Response()
    load_req = LoadNode.Request()
    load_req.package_name = 'rclpy_components'
    load_req.plugin_name = 'FooNode'

    component_manager.on_load_node(load_req, load_res)

    assert load_res.success is True

    # List
    list_res = ListNodes.Response()
    list_req = ListNodes.Request()

    component_manager.on_list_node(list_req, list_res)

    assert len(list_res.full_node_names) == 1
    assert len(list_res.unique_ids) == 1
    assert load_res.unique_id in list_res.unique_ids

    # Unload
    unload_res = UnloadNode.Response()
    unload_req = UnloadNode.Request()
    unload_req.unique_id = load_res.unique_id

    component_manager.on_unload_node(unload_req, unload_res)

    assert unload_res.success is True

    # List
    list_res = ListNodes.Response()
    list_req = ListNodes.Request()

    component_manager.on_list_node(list_req, list_res)

    assert len(list_res.full_node_names) == 0
    assert len(list_res.unique_ids) == 0


def test_unload_non_existant_node(component_manager):
    unload_res = UnloadNode.Response()
    unload_req = UnloadNode.Request()
    unload_req.unique_id = 42

    component_manager.on_unload_node(unload_req, unload_res)

    assert unload_res.success is False
    assert '42' in unload_res.error_message


def test_load_fails(component_manager):
    load_res = LoadNode.Response()
    load_req = LoadNode.Request()
    load_req.package_name = 'fake_package'
    load_req.plugin_name = 'FakePlugin'

    with patch('rclpy_components.component_manager.entry_points') as m:
        m.return_value.select.return_value = [
            FakeEntryPointFailsToLoad(
                f'{load_req.package_name}::{load_req.plugin_name}', 'not.used:here'),
        ]
        component_manager.on_load_node(load_req, load_res)

    assert load_res.success is False
    assert 'Oops I failed to load' in load_res.error_message


def test_load_node_wont_init(component_manager):
    load_res = LoadNode.Response()
    load_req = LoadNode.Request()
    load_req.package_name = 'fake_package'
    load_req.plugin_name = 'FakePlugin'

    with patch('rclpy_components.component_manager.entry_points') as m:
        fake_ep = FakeEntryPoint(
            f'{load_req.package_name}::{load_req.plugin_name}', 'not.used:here')
        fake_ep.mock_class.side_effect = ValueError('Oops I failed to construct')
        m.return_value.select.return_value = [fake_ep]
        component_manager.on_load_node(load_req, load_res)

    assert load_res.success is False
    assert 'Oops I failed to construct' in load_res.error_message


def test_load_list_unload_via_services(component_manager, node_and_context):
    node, context = node_and_context
    manager_fqn = component_manager.get_fully_qualified_name()

    load_node_service = f'{manager_fqn}/_container/load_node'
    unload_node_service = f'{manager_fqn}/_container/unload_node'
    list_node_service = f'{manager_fqn}/_container/list_nodes'

    load_cli = node.create_client(LoadNode, load_node_service)
    unload_cli = node.create_client(UnloadNode, unload_node_service)
    list_cli = node.create_client(ListNodes, list_node_service)

    assert load_cli.wait_for_service(timeout_sec=5.0)
    assert unload_cli.wait_for_service(timeout_sec=5.0)
    assert list_cli.wait_for_service(timeout_sec=5.0)

    # Load one node
    load_req = LoadNode.Request()
    load_req.package_name = 'rclpy_components'
    load_req.plugin_name = 'FooNode'

    load_res = load_cli.call(load_req)
    assert load_res.success is True

    # List the node
    list_res = list_cli.call(ListNodes.Request())
    assert len(list_res.full_node_names) == 1
    assert len(list_res.unique_ids) == 1
    assert load_res.unique_id in list_res.unique_ids

    # Unload it

    unload_req = UnloadNode.Request()
    unload_req.unique_id = load_res.unique_id
    unload_res = unload_cli.call(unload_req)
    assert unload_res.success is True
