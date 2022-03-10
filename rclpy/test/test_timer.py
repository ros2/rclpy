# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import os
import platform
import time

import pytest
import rclpy
from rclpy.constants import S_TO_NS
from rclpy.executors import SingleThreadedExecutor


TEST_PERIODS = (
    0.1,
    pytest.param(
        0.01,
        marks=(
            pytest.mark.skipif(os.name == 'nt', reason='Flaky on windows'),
        )
    ),
    pytest.param(
        0.001,
        marks=(
            pytest.mark.skipif(os.name == 'nt', reason='Flaky on windows'),
            pytest.mark.skipif(platform.machine() == 'aarch64', reason='Flaky on arm')
        )
    )
)


@pytest.mark.parametrize('period', TEST_PERIODS)
def test_zero_callback(period):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('test_timer_no_callback', context=context)
        try:
            executor = SingleThreadedExecutor(context=context)
            try:
                executor.add_node(node)
                # The first spin_once() takes long enough for 1ms timer tests to fail
                executor.spin_once(timeout_sec=0)

                callbacks = []
                timer = node.create_timer(period, lambda: callbacks.append(len(callbacks)))
                try:
                    executor.spin_once(timeout_sec=(period / 2))

                    assert len(callbacks) == 0
                finally:
                    node.destroy_timer(timer)
            finally:
                executor.shutdown()
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


@pytest.mark.parametrize('period', TEST_PERIODS)
def test_number_callbacks(period):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('test_timer_number_callbacks', context=context)
        try:
            executor = SingleThreadedExecutor(context=context)
            try:
                executor.add_node(node)
                # The first spin_once() takes long enough for 1ms timer tests to fail
                executor.spin_once(timeout_sec=0)

                callbacks = []
                timer = node.create_timer(period, lambda: callbacks.append(len(callbacks)))
                try:
                    begin_time = time.time()

                    while rclpy.ok(context=context) and time.time() - begin_time < 4.5 * period:
                        executor.spin_once(timeout_sec=period / 10)

                    assert len(callbacks) == 4
                finally:
                    node.destroy_timer(timer)
            finally:
                executor.shutdown()
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


@pytest.mark.parametrize('period', TEST_PERIODS)
def test_cancel_reset(period):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('test_timer_cancel_reset', context=context)
        try:
            executor = SingleThreadedExecutor(context=context)
            try:
                executor.add_node(node)
                # The first spin_once() takes long enough for 1ms timer tests to fail
                executor.spin_once(timeout_sec=0)

                callbacks = []
                timer = node.create_timer(period, lambda: callbacks.append(len(callbacks)))
                try:
                    # Make sure callbacks can be received
                    assert not timer.is_canceled()
                    begin_time = time.time()
                    while rclpy.ok(context=context) and time.time() - begin_time < 2.5 * period:
                        executor.spin_once(timeout_sec=period / 10)
                    assert len(callbacks) == 2

                    # Cancel timer and make sure no callbacks are received
                    assert not timer.is_canceled()
                    timer.cancel()
                    assert timer.is_canceled()
                    callbacks = []
                    begin_time = time.time()
                    while rclpy.ok(context=context) and time.time() - begin_time < 2.5 * period:
                        executor.spin_once(timeout_sec=period / 10)
                    assert [] == callbacks

                    # Reenable timer and make sure callbacks are received again
                    timer.reset()
                    assert not timer.is_canceled()
                    begin_time = time.time()
                    while rclpy.ok(context=context) and time.time() - begin_time < 2.5 * period:
                        executor.spin_once(timeout_sec=period / 10)
                    assert len(callbacks) == 2
                finally:
                    node.destroy_timer(timer)
            finally:
                executor.shutdown()
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def test_time_until_next_call():
    node = None
    executor = None
    timer = None
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('test_time_until_next_call', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.spin_once(timeout_sec=0)
        timer = node.create_timer(1, lambda: None)
        assert not timer.is_canceled()
        executor.spin_once(0.1)
        assert timer.time_until_next_call() <= (1 * S_TO_NS)
        timer.reset()
        assert not timer.is_canceled()
        assert timer.time_until_next_call() <= (1 * S_TO_NS)
        timer.cancel()
        assert timer.is_canceled()
        assert timer.time_until_next_call() is None
    finally:
        if timer is not None:
            node.destroy_timer(timer)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown(context=context)
