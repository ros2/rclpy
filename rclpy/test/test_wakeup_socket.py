# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import contextlib
import socket
import time
from typing import Generator
from typing import Tuple

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from test_msgs.msg import Empty as EmptyMsg
from test_msgs.srv import Empty as EmptySrv


NODE_NAME = 'test_wakeup_socket'
SERVICE_NODE_NAME = 'test_wakeup_socket_service_node'
TOPIC_NAME = '/test_wakeup_socket_topic'
SERVICE_NAME = '/test_wakeup_socket_service'
DISCOVERY_TIMEOUT = 5.0
WAKEUP_TIMEOUT = 5.0
NO_WAKEUP_TIMEOUT = 1.0


@pytest.fixture
def test_node() -> Generator[Node, None, None]:
    with rclpy.init():
        node = Node(NODE_NAME)
        yield node
        node.destroy_node()


@pytest.fixture
def wakeup_socket() -> Generator[Tuple[socket.socket, socket.socket], None, None]:
    read_end, write_end = socket.socketpair()
    write_end.setblocking(False)
    read_end.settimeout(WAKEUP_TIMEOUT)
    yield read_end, write_end
    read_end.close()
    write_end.close()


@pytest.fixture
def _service_node(test_node: Node) -> Generator[Node, None, None]:
    node = Node(SERVICE_NODE_NAME)
    node.create_service(EmptySrv, SERVICE_NAME, lambda req, res: res)
    yield node
    node.destroy_node()


@pytest.fixture
def executor(service_node: Node) -> Generator[SingleThreadedExecutor, None, None]:
    executor = SingleThreadedExecutor()
    executor.add_node(service_node)
    yield executor
    executor.shutdown()


def drain(read_end: socket.socket) -> None:
    read_end.setblocking(False)
    with contextlib.suppress(BlockingIOError):
        while read_end.recv(4096):
            pass


def assert_no_wakeup(read_end: socket.socket) -> None:
    read_end.settimeout(NO_WAKEUP_TIMEOUT)
    with pytest.raises(TimeoutError):
        read_end.recv(1)


def test_on_new_message_wakeup(
    test_node: Node,
    wakeup_socket: Tuple[socket.socket, socket.socket],
) -> None:
    read_end, write_end = wakeup_socket
    sub = test_node.create_subscription(EmptyMsg, TOPIC_NAME, lambda msg: None, 10)
    pub = test_node.create_publisher(EmptyMsg, TOPIC_NAME, 10)

    end_time = time.time() + DISCOVERY_TIMEOUT
    while sub.get_publisher_count() != 1:
        time.sleep(0.1)
        assert time.time() <= end_time, 'Timed out waiting for discovery'

    try:
        sub.handle.set_on_new_message_wakeup(write_end.fileno())
        pub.publish(EmptyMsg())
        assert read_end.recv(1) == b'\x01'
    finally:
        sub.handle.clear_on_new_message_callback()

    drain(read_end)
    pub.publish(EmptyMsg())
    assert_no_wakeup(read_end)


def test_on_new_request_wakeup(
    test_node: Node,
    wakeup_socket: Tuple[socket.socket, socket.socket],
) -> None:
    read_end, write_end = wakeup_socket
    cli = test_node.create_client(EmptySrv, SERVICE_NAME)
    srv = test_node.create_service(EmptySrv, SERVICE_NAME, lambda req, res: res)
    assert cli.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT)
    try:
        srv.handle.set_on_new_request_wakeup(write_end.fileno())
        cli.call_async(EmptySrv.Request())
        assert read_end.recv(1) == b'\x01'
    finally:
        srv.handle.clear_on_new_request_callback()

    drain(read_end)
    cli.call_async(EmptySrv.Request())
    assert_no_wakeup(read_end)


def test_on_new_response_wakeup(
    test_node: Node,
    wakeup_socket: Tuple[socket.socket, socket.socket],
    _service_node: Node,
    executor: SingleThreadedExecutor,
) -> None:
    read_end, write_end = wakeup_socket
    cli = test_node.create_client(EmptySrv, SERVICE_NAME)
    assert cli.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT)
    try:
        cli.handle.set_on_new_response_wakeup(write_end.fileno())
        cli.call_async(EmptySrv.Request())
        executor.spin_once(WAKEUP_TIMEOUT)
        assert read_end.recv(1) == b'\x01'
    finally:
        cli.handle.clear_on_new_response_callback()

    drain(read_end)
    cli.call_async(EmptySrv.Request())
    executor.spin_once(WAKEUP_TIMEOUT)
    assert_no_wakeup(read_end)
