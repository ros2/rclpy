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

import multiprocessing
import os
import platform
import sys
import time
import traceback
from unittest.case import SkipTest

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor


def run_catch_report_raise(func, *args, **kwargs):
    try:
        return func(*args, **kwargs)
    except Exception:
        print('exception in {0}():'.format(func.__name__), file=sys.stderr)
        traceback.print_exc()
        raise


def func_zero_callback(args):
    period = float(args[0])

    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_timer_no_callback', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    # The first spin_once() takes slighly longer, just long enough for 1ms timer tests to fail
    executor.spin_once(timeout_sec=0)

    callbacks = []
    timer = node.create_timer(period, lambda: callbacks.append(len(callbacks)))
    executor.spin_once(timeout_sec=period / 2.)

    node.destroy_timer(timer)
    executor.shutdown()
    rclpy.shutdown(context=context)
    assert len(callbacks) == 0, \
        "shouldn't have received any callback, received %d" % len(callbacks)

    return True


def func_number_callbacks(args):
    period = float(args[0])

    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_timer_no_callback', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    # The first spin_once() takes slighly longer, just long enough for 1ms timer tests to fail
    executor.spin_once(timeout_sec=0)

    callbacks = []
    timer = node.create_timer(period, lambda: callbacks.append(len(callbacks)))
    begin_time = time.time()

    while rclpy.ok(context=context) and time.time() - begin_time < 4.5 * period:
        executor.spin_once(timeout_sec=period / 10)

    node.destroy_timer(timer)
    executor.shutdown()
    rclpy.shutdown(context=context)
    assert len(callbacks) == 4, 'should have received 4 callbacks, received %s' % str(callbacks)

    return True


def func_cancel_reset_timer(args):
    period = float(args[0])

    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_timer_no_callback', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    # The first spin_once() takes slighly longer, just long enough for 1ms timer tests to fail
    executor.spin_once(timeout_sec=0)

    callbacks = []
    timer = node.create_timer(period, lambda: callbacks.append(len(callbacks)))
    begin_time = time.time()

    assert not timer.is_canceled()

    while rclpy.ok(context=context) and time.time() - begin_time < 2.5 * period:
        executor.spin_once(timeout_sec=period / 10)

    assert len(callbacks) == 2, 'should have received 2 callbacks, received %d' % len(callbacks)
    assert not timer.is_canceled()

    timer.cancel()
    assert timer.is_canceled()
    callbacks = []
    begin_time = time.time()
    while rclpy.ok(context=context) and time.time() - begin_time < 2.5 * period:
        executor.spin_once(timeout_sec=period / 10)

    assert timer.is_canceled()
    assert [] == callbacks, \
        "shouldn't have received any callback, received %d" % len(callbacks)

    timer.reset()
    assert not timer.is_canceled()
    begin_time = time.time()
    while rclpy.ok(context=context) and time.time() - begin_time < 2.5 * period:
        executor.spin_once(timeout_sec=period / 10)

    assert not timer.is_canceled()
    assert len(callbacks) == 2, 'should have received 2 callbacks, received %d' % len(callbacks)
    node.destroy_timer(timer)
    executor.shutdown()
    rclpy.shutdown(context=context)

    return True


def func_launch(function, args, message):
    pool = multiprocessing.Pool(1)
    result = pool.apply(
        func=run_catch_report_raise,
        args=(function, args)
    )

    assert result, message
    pool.close()
    pool.join()


def test_timer_zero_callbacks10hertz():
    func_launch(func_zero_callback, ['0.1'], 'received callbacks when not expected')


def test_timer_zero_callbacks100hertz():
    func_launch(func_zero_callback, ['0.01'], 'received callbacks when not expected')


# TODO(mikaelarguedas) reenable these once timer have consistent behaviour
# on every platform at high frequency
# TODO(sloretz) Reenable on arm when executor performance is good enough
@pytest.mark.skip(reason='1kHz tests are too prone to flakiness')
def test_timer_zero_callbacks1000hertz():
    if os.name == 'nt' or platform.machine() == 'aarch64':
        raise SkipTest
    func_launch(func_zero_callback, ['0.001'], 'received callbacks when not expected')


def test_timer_number_callbacks10hertz():
    func_launch(func_number_callbacks, ['0.1'], "didn't receive the expected number of callbacks")


def test_timer_number_callbacks100hertz():
    func_launch(
        func_number_callbacks, ['0.01'], "didn't receive the expected number of callbacks")


@pytest.mark.skip(reason='1kHz tests are too prone to flakiness')
def test_timer_number_callbacks1000hertz():
    if os.name == 'nt' or platform.machine() == 'aarch64':
        raise SkipTest
    func_launch(
        func_number_callbacks, ['0.001'], "didn't receive the expected number of callbacks")


def test_timer_cancel_reset_10hertz():
    func_launch(
        func_cancel_reset_timer, ['0.1'], "didn't receive the expected number of callbacks")


def test_timer_cancel_reset_100hertz():
    func_launch(
        func_cancel_reset_timer, ['0.01'], "didn't receive the expected number of callbacks")


@pytest.mark.skip(reason='1kHz tests are too prone to flakiness')
def test_timer_cancel_reset_1000hertz():
    if os.name == 'nt' or platform.machine() == 'aarch64':
        raise SkipTest
    func_launch(
        func_cancel_reset_timer, ['0.001'], "didn't receive the expected number of callbacks")
