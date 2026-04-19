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
from rclpy.clock_type import ClockType
from rclpy.exceptions import TimeSourceChangedError
from rclpy.experimental import AsyncNode
from rclpy.parameter import Parameter
from rclpy.time import Time


@pytest.fixture(autouse=True)
def rclpy_context():
    """Initialize and shut down rclpy for each test."""
    with rclpy.init():
        yield


@pytest.mark.asyncio
async def test_sleep_wall_clock():
    """Sleep completes after the requested duration (wall clock)."""
    async with AsyncNode('test_sleep_node') as node:
        async with asyncio.timeout(5):
            await node.get_clock().sleep(0.1)


@pytest.mark.asyncio
async def test_sleep_cancelled_on_close():
    """Pending sleeps are cancelled when node.destroy_node() is called."""
    async with AsyncNode('test_sleep_cancel_node') as node:
        loop = asyncio.get_running_loop()
        loop.call_soon(node.destroy_node)
        with pytest.raises(asyncio.CancelledError):
            async with asyncio.timeout(5):
                await node.get_clock().sleep(999)


@pytest.mark.asyncio
async def test_sleep_raises_on_clock_change():
    """Wall clock sleep raises TimeSourceChangedError when sim time activates."""
    async with AsyncNode('test_sleep_clock_change_node') as node:
        # Activate sim time — triggers ROS_TIME_ACTIVATED jump callback
        loop = asyncio.get_running_loop()
        loop.call_soon(node.set_parameters, [Parameter(
            'use_sim_time', Parameter.Type.BOOL, True)])

        async with asyncio.timeout(5):
            with pytest.raises(TimeSourceChangedError):
                await node.get_clock().sleep(999)


@pytest.mark.asyncio
async def test_clock_sleep_zero_duration():
    """Sleeping for zero duration returns immediately."""
    async with AsyncNode('test_clock_zero_node') as node:
        async with asyncio.timeout(1):
            await node.get_clock().sleep(0)
            await node.get_clock().sleep(-1.0)


@pytest.mark.asyncio
async def test_clock_sleep_on_destroyed_clock():
    """Sleeping on a destroyed clock raises RuntimeError."""
    node = AsyncNode('test_clock_destroyed_node')
    clock = node.get_clock()
    node.destroy_node()
    with pytest.raises(RuntimeError):
        await clock.sleep(1.0)


@pytest.mark.asyncio
async def test_sleep_sim_time_resolves_on_jump():
    """Sim-time sleep resolves when ROS time advances past its target."""
    async with AsyncNode(
        'test_sleep_sim_node',
        parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)],
    ) as node:
        clock = node.get_clock()
        assert clock.ros_time_is_active

        clock.set_ros_time_override(
            Time(seconds=10, clock_type=ClockType.ROS_TIME))
        sleep_task = asyncio.create_task(clock.sleep(1.0))

        with pytest.raises(TimeoutError):
            async with asyncio.timeout(0.05):
                await asyncio.shield(sleep_task)

        clock.set_ros_time_override(
            Time(seconds=11, clock_type=ClockType.ROS_TIME))

        async with asyncio.timeout(0.1):
            await sleep_task
