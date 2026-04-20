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
from rclpy.experimental import AsyncNode
from rclpy.experimental import AsyncTimer
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.timer import TimerInfo


@pytest.fixture(autouse=True)
def rclpy_context():
    """Initialize and shut down rclpy for each test."""
    with rclpy.init():
        yield


@pytest.mark.asyncio
async def test_timer_fires():
    """Timer callback fires at least once within timeout (sync callback)."""
    count = 0
    fired = asyncio.Event()

    def callback():
        nonlocal count
        count += 1
        fired.set()

    async with AsyncNode('test_timer_fires_node') as node:
        node.create_timer(0.05, callback)
        async with asyncio.timeout(5):
            await fired.wait()
        assert count >= 1


@pytest.mark.asyncio
async def test_timer_fires_multiple():
    """Timer fires multiple times over a short period."""
    count = 0
    enough = asyncio.Event()

    async def callback():
        nonlocal count
        count += 1
        if count >= 3:
            enough.set()

    async with AsyncNode('test_timer_multi_node') as node:
        node.create_timer(0.05, callback)
        async with asyncio.timeout(5):
            await enough.wait()
        assert count >= 3


@pytest.mark.asyncio
async def test_timer_cancel_and_reset():
    """Cancelled timer stops firing; reset resumes it."""
    count = 0
    fired = asyncio.Event()

    async def callback():
        nonlocal count
        count += 1
        fired.set()

    async with AsyncNode('test_timer_cancel_reset_node') as node:
        timer = node.create_timer(0.05, callback)

        # Wait for first fire
        async with asyncio.timeout(5):
            await fired.wait()
        assert count >= 1

        # Cancel and verify no more fires
        timer.cancel()
        count_at_cancel = count
        await asyncio.sleep(0.15)
        assert count == count_at_cancel

        # Reset and verify it fires again
        fired.clear()
        timer.reset()
        async with asyncio.timeout(5):
            await fired.wait()
        assert count > count_at_cancel


@pytest.mark.asyncio
async def test_timer_destroy():
    """Destroying a timer stops it."""
    count = 0
    fired = asyncio.Event()

    async def callback():
        nonlocal count
        count += 1
        fired.set()

    async with AsyncNode('test_timer_destroy_node') as node:
        timer = node.create_timer(0.05, callback)
        async with asyncio.timeout(5):
            await fired.wait()

        timer.destroy()
        count_at_destroy = count
        await asyncio.sleep(0.15)
        assert count == count_at_destroy


@pytest.mark.asyncio
async def test_timer_callback_exception():
    """Exception in timer callback propagates as ExceptionGroup."""
    async def bad_callback():
        raise ValueError('timer boom')

    node = AsyncNode('test_timer_exc_node')
    node.create_timer(0.05, bad_callback)

    with pytest.raises(ExceptionGroup) as exc_info:
        async with asyncio.timeout(5):
            await node.run()

    assert exc_info.value.subgroup(ValueError)


@pytest.mark.asyncio
async def test_timer_create_before_aenter():
    """Timer created before entering async context fires after entry."""
    fired = asyncio.Event()

    async def callback():
        fired.set()

    node = AsyncNode('test_timer_pre_aenter_node')
    node.create_timer(0.05, callback)

    async with node:
        async with asyncio.timeout(5):
            await fired.wait()


@pytest.mark.asyncio
async def test_timer_introspection():
    """Timer exposes period and cancel state via BaseTimer."""
    fired = asyncio.Event()

    async def callback():
        fired.set()

    async with AsyncNode('test_timer_introspect_node') as node:
        timer = node.create_timer(0.1, callback)
        assert isinstance(timer, AsyncTimer)
        assert timer.timer_period_ns == 0.1 * 1e9
        assert not timer.is_canceled()

        timer.cancel()
        assert timer.is_canceled()

        timer.reset()
        assert not timer.is_canceled()


@pytest.mark.asyncio
async def test_timer_callback_with_info():
    """Timer callback receives TimerInfo when it accepts a parameter."""
    received_info = []
    fired = asyncio.Event()

    async def callback(info: TimerInfo):
        received_info.append(info)
        fired.set()

    async with AsyncNode('test_timer_info_node') as node:
        node.create_timer(0.05, callback)
        async with asyncio.timeout(5):
            await fired.wait()
        assert len(received_info) >= 1
        assert isinstance(received_info[0], TimerInfo)
        assert received_info[0].expected_call_time is not None
        assert received_info[0].actual_call_time is not None


@pytest.mark.asyncio
async def test_timer_zero_period():
    """Timer with zero period fires immediately."""
    fired = asyncio.Event()

    async def callback():
        fired.set()

    async with AsyncNode('test_timer_zero_node') as node:
        node.create_timer(0.0, callback)
        async with asyncio.timeout(5):
            await fired.wait()


@pytest.mark.asyncio
async def test_timer_callback_signature_rejected():
    """Timer rejects callbacks with invalid signatures (2+ params)."""
    node = AsyncNode('test_timer_bad_sig_node')

    async def bad_callback(a, b):
        pass

    with pytest.raises(RuntimeError):
        node.create_timer(1.0, bad_callback)

    node.destroy_node()


@pytest.mark.asyncio
async def test_timer_fires_under_sim_time():
    """Timer fires when ROS time advances past its period under sim time."""
    count = 0
    fired = asyncio.Event()

    async def callback():
        nonlocal count
        count += 1
        fired.set()

    async with AsyncNode(
        'test_timer_sim_node',
        parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)],
    ) as node:
        clock = node.get_clock()
        assert clock.ros_time_is_active

        clock.set_ros_time_override(
            Time(seconds=0, clock_type=ClockType.ROS_TIME))
        node.create_timer(1.0, callback)

        with pytest.raises(TimeoutError):
            async with asyncio.timeout(0.05):
                await fired.wait()

        clock.set_ros_time_override(
            Time(seconds=1, clock_type=ClockType.ROS_TIME))

        async with asyncio.timeout(1):
            await fired.wait()
        assert count == 1
