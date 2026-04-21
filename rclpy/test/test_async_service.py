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
async def test_service_concurrent_dispatch():
    """Concurrent service dispatch handles requests in parallel."""
    NUM_REQUESTS = 3
    barrier = asyncio.Barrier(NUM_REQUESTS)

    async def handler(request, response):
        await barrier.wait()
        response.bool_value = True
        return response

    async with (
        AsyncNode('test_conc_svc_srv_node') as srv_node,
        AsyncNode('test_conc_svc_client_node') as client_node,
    ):
        srv_node.create_service(
            BasicTypesSrv, '/test_conc_svc', handler, concurrent=True)
        client = client_node.create_client(BasicTypesSrv, '/test_conc_svc')

        async with asyncio.timeout(5):
            await client.wait_for_service()
            async with asyncio.TaskGroup() as tg:
                for _ in range(NUM_REQUESTS):
                    tg.create_task(client.call(BasicTypesSrv.Request()))


@pytest.mark.asyncio
@pytest.mark.parametrize('concurrent', [False, True])
async def test_service_callback_exception(concurrent):
    """Exception in service callback propagates as ExceptionGroup."""
    async def bad_handler(request, response):
        raise ValueError('service boom')

    node = AsyncNode('test_svc_exc_node')
    node.create_service(
        BasicTypesSrv, '/test_svc_exc', bad_handler, concurrent=concurrent)
    client_node = AsyncNode('test_svc_exc_client_node')
    client = client_node.create_client(BasicTypesSrv, '/test_svc_exc')

    with pytest.raises(ExceptionGroup) as exc_info:
        async with asyncio.timeout(5):
            async with node, client_node:
                await client.wait_for_service()
                await client.call(BasicTypesSrv.Request())

    assert exc_info.value.subgroup(ValueError)


@pytest.mark.asyncio
@pytest.mark.parametrize('concurrent', [False, True])
async def test_service_destroy_cancels_in_flight_handler(concurrent):
    """destroy_node() during an in-flight handler cancels it without crashing the node."""
    handler_started = asyncio.Event()

    async def slow_handler(request, response):
        handler_started.set()
        await asyncio.Event().wait()
        return response

    async with (
        AsyncNode('test_srv_cancel_srv_node') as srv_node,
        AsyncNode('test_srv_cancel_client_node') as client_node,
    ):
        srv_node.create_service(
            BasicTypesSrv, '/test_srv_cancel_svc',
            slow_handler, concurrent=concurrent)
        client = client_node.create_client(
            BasicTypesSrv, '/test_srv_cancel_svc')

        async with asyncio.timeout(5):
            await client.wait_for_service()
            call_task = asyncio.create_task(
                client.call(BasicTypesSrv.Request()))
            await handler_started.wait()

        srv_node.destroy_node()

        # Service was destroyed mid-handler — no response arrives
        with pytest.raises(TimeoutError):
            async with asyncio.timeout(0.5):
                await call_task
