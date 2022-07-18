# Copyright 2022 Open Source Robotics Foundation, Inc.
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
import time

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor


MAX_TEST_TIME = 5.0
TIME_FUDGE_FACTOR = 0.2


@pytest.fixture
def node_and_executor():
    rclpy.init()
    node = rclpy.create_node('test_asyncio_interop')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    yield node, executor
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


def test_sleep_in_event_loop(node_and_executor):
    node, executor = node_and_executor

    expected_sleep_time = 0.5
    sleep_time = None

    async def cb():
        nonlocal sleep_time
        start = time.monotonic()
        await asyncio.sleep(expected_sleep_time)
        end = time.monotonic()
        sleep_time = end - start

    guard = node.create_guard_condition(cb)
    guard.trigger()

    async def spin():
        nonlocal sleep_time
        start = time.monotonic()
        while not sleep_time and MAX_TEST_TIME > time.monotonic() - start:
            executor.spin_once(timeout_sec=0)
            # Don't use 100% CPU
            await asyncio.sleep(0.01)

    asyncio.run(spin())
    assert sleep_time >= expected_sleep_time
    assert abs(expected_sleep_time - sleep_time) <= expected_sleep_time * TIME_FUDGE_FACTOR
