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

import asyncio

import pytest

import rclpy
from rclpy.experimental import AsyncNode

from test_msgs.srv import BasicTypes as BasicTypesSrv


@pytest.fixture(autouse=True)
def rclpy_context():
    """Initialize and shut down rclpy for each test."""
    with rclpy.init():
        yield


@pytest.mark.asyncio
async def test_client_calls_async_service():
    """Client can call a service hosted by another AsyncNode (sync handler)."""
    def handler(request, response):
        response.bool_value = not request.bool_value
        response.string_value = 'inverted'
        return response

    async with (
        AsyncNode('test_full_srv_node') as srv_node,
        AsyncNode('test_full_client_node') as client_node,
    ):
        srv_node.create_service(BasicTypesSrv, '/test_full_service', handler)
        client = client_node.create_client(BasicTypesSrv, '/test_full_service')

        async with asyncio.timeout(5):
            await client.wait_for_service()
            response = await client.call(BasicTypesSrv.Request(bool_value=True))

        assert response.bool_value is False
        assert response.string_value == 'inverted'


@pytest.mark.asyncio
async def test_wait_for_service_timeout():
    """Timeout is raised when no service server exists."""
    async with AsyncNode('test_wfs_timeout_node') as node:
        client = node.create_client(BasicTypesSrv, '/nonexistent_service')
        with pytest.raises(TimeoutError):
            async with asyncio.timeout(0.5):
                await client.wait_for_service()


@pytest.mark.asyncio
async def test_client_call_on_destroyed_client():
    """Calling a destroyed client raises RuntimeError."""
    async with AsyncNode('test_destroyed_call_node') as node:
        client = node.create_client(BasicTypesSrv, '/some_service')
        client.destroy()
        with pytest.raises(RuntimeError):
            await client.call(BasicTypesSrv.Request())


@pytest.mark.asyncio
async def test_client_concurrent_calls():
    """Multiple concurrent calls are correctly demuxed by sequence number."""
    NUM_CALLS = 3
    barrier = asyncio.Barrier(NUM_CALLS)

    async def handler(request, response):
        await barrier.wait()
        return response

    async with (
        AsyncNode('test_conc_call_srv_node') as srv_node,
        AsyncNode('test_conc_call_client_node') as client_node,
    ):
        srv_node.create_service(
            BasicTypesSrv, '/test_conc_call_svc', handler, concurrent=True)
        client = client_node.create_client(
            BasicTypesSrv, '/test_conc_call_svc')

        async with asyncio.timeout(5):
            await client.wait_for_service()
            async with asyncio.TaskGroup() as tg:
                for _ in range(NUM_CALLS):
                    tg.create_task(client.call(BasicTypesSrv.Request()))


@pytest.mark.asyncio
async def test_client_call_timeout_then_next_call_succeeds():
    """A timed-out call does not break subsequent calls on the same client."""
    first_call = True

    async def handler(request, response):
        nonlocal first_call
        if first_call:
            first_call = False
            await asyncio.Event().wait()
        response.bool_value = True
        return response

    async with (
        AsyncNode('test_timeout_srv_node') as srv_node,
        AsyncNode('test_timeout_client_node') as client_node,
    ):
        srv_node.create_service(
            BasicTypesSrv, '/test_timeout_svc', handler, concurrent=True)
        client = client_node.create_client(
            BasicTypesSrv, '/test_timeout_svc')

        async with asyncio.timeout(5):
            await client.wait_for_service()

        with pytest.raises(TimeoutError):
            async with asyncio.timeout(0.2):
                await client.call(BasicTypesSrv.Request())

        async with asyncio.timeout(5):
            response = await client.call(BasicTypesSrv.Request())
        assert response.bool_value is True


@pytest.mark.asyncio
async def test_client_destroy_cancels_in_flight_call():
    """destroy() from the server callback cancels the in-flight call()."""
    async def handler(request, response):
        client.destroy()
        return response

    async with (
        AsyncNode('test_destroy_from_srv_node') as srv_node,
        AsyncNode('test_destroy_from_client_node') as client_node,
    ):
        srv_node.create_service(
            BasicTypesSrv, '/test_destroy_from_svc', handler)
        client = client_node.create_client(
            BasicTypesSrv, '/test_destroy_from_svc')

        async with asyncio.timeout(5):
            await client.wait_for_service()

            with pytest.raises(asyncio.CancelledError):
                await client.call(BasicTypesSrv.Request())
