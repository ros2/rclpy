# Copyright 2023 Sony Group Corporation.
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

from typing import Generator
from typing import List
from typing import Optional
from unittest.mock import Mock

import pytest

import rclpy
from rclpy.node import Node
from rclpy.service import Service

from test_msgs.srv import Empty

from typing_extensions import TypeAlias

EmptyService: TypeAlias = Service[Empty.Request, Empty.Response]


NODE_NAME = 'test_node'


@pytest.fixture(autouse=True)
def default_context() -> Generator[None, None, None]:
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_node():
    node = Node(NODE_NAME)
    yield node
    node.destroy_node()


def test_logger_name_is_equal_to_node_name(test_node):
    srv = test_node.create_service(
        srv_type=Empty,
        srv_name='test_srv',
        callback=lambda _: None
    )

    assert srv.logger_name == NODE_NAME


@pytest.mark.parametrize('service_name, namespace, expected', [
    # No namespaces
    ('service', None, '/service'),
    ('example/service', None, '/example/service'),
    # Using service names with namespaces
    ('service', 'ns', '/ns/service'),
    ('example/service', 'ns', '/ns/example/service'),
    ('example/service', 'my/ns', '/my/ns/example/service'),
    ('example/service', '/my/ns', '/my/ns/example/service'),
    # Global service name
    ('/service', 'ns', '/service'),
    ('/example/service', 'ns', '/example/service'),
])
def test_get_service_name(service_name: str, namespace: Optional[str], expected: str) -> None:
    node = Node('node_name', namespace=namespace, cli_args=None, start_parameter_services=False)
    srv: EmptyService = node.create_service(
        srv_type=Empty,
        srv_name=service_name,
        callback=lambda _, _1: None
    )

    assert srv.service_name == expected

    srv.destroy()
    node.destroy_node()


@pytest.mark.parametrize('service_name, namespace, cli_args, expected', [
    ('service', None, ['--ros-args', '--remap', 'service:=new_service'], '/new_service'),
    ('service', 'ns', ['--ros-args', '--remap', 'service:=new_service'], '/ns/new_service'),
    ('service', 'ns', ['--ros-args', '--remap', 'service:=example/new_service'],
     '/ns/example/new_service'),
    ('example/service', 'ns', ['--ros-args', '--remap', 'example/service:=new_service'],
     '/ns/new_service'),
])
def test_get_service_name_after_remapping(service_name: str, namespace: Optional[str],
                                          cli_args: List[str], expected: str) -> None:
    node = Node(
        'node_name',
        namespace=namespace,
        cli_args=cli_args,
        start_parameter_services=False)
    srv: EmptyService = node.create_service(
        srv_type=Empty,
        srv_name=service_name,
        callback=lambda _, _1: None
    )

    assert srv.service_name == expected

    srv.destroy()
    node.destroy_node()


def test_service_context_manager() -> None:
    with rclpy.create_node('ctx_mgr_test') as node:
        srv: EmptyService
        with node.create_service(
                srv_type=Empty, srv_name='empty_service', callback=lambda _, _1: None) as srv:
            assert srv.service_name == '/empty_service'


def test_set_on_new_request_callback(test_node) -> None:
    cli = test_node.create_client(Empty, '/service')
    srv = test_node.create_service(Empty, '/service', lambda req, res: res)
    cb = Mock()
    srv.handle.set_on_new_request_callback(cb)
    cb.assert_not_called()
    cli.call_async(Empty.Request())
    cb.assert_called_once_with(1)
    srv.handle.clear_on_new_request_callback()
    cli.call_async(Empty.Request())
    cb.assert_called_once()
