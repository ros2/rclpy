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
import sys
import traceback


def run_catch_report_raise(func, *args, **kwargs):
    try:
        return func(*args, **kwargs)
    except:
        print('exception in {0}():'.format(func.__name__), file=sys.stderr)
        traceback.print_exc()
        raise


def func_zero_callback(args):
    import rclpy

    period = float(args[0])
    print('period: %f' % period)

    rclpy.init()
    node = rclpy.create_node('test_timer_no_callback')

    callbacks = []
    timer = node.create_timer(period, lambda: callbacks.append(len(callbacks)))

    rclpy.spin_once(node, period / 2.)

    node.destroy_timer(timer)
    rclpy.shutdown()
    assert len(callbacks) == 0, 'shouldn\'t have received any callback'

    return True


def func_number_callbacks(args):
    import time

    import rclpy

    period = float(args[0])
    print('period: %f' % period)

    rclpy.init()
    node = rclpy.create_node('test_timer_no_callback')

    callbacks = []
    timer = node.create_timer(period, lambda: callbacks.append(len(callbacks)))
    begin_time = time.time()

    while rclpy.ok() and time.time() - begin_time < 4.5 * period:
        rclpy.spin_once(node, period / 10)

    node.destroy_timer(timer)
    rclpy.shutdown()
    assert len(callbacks) == 4, 'should have received 4 callbacks, received %s' % str(callbacks)

    return True


def func_cancel_reset_timer(args):
    import time

    import rclpy

    period = float(args[0])
    print('period: %f' % period)

    rclpy.init()
    node = rclpy.create_node('test_timer_no_callback')

    callbacks = []
    timer = node.create_timer(period, lambda: callbacks.append(len(callbacks)))
    begin_time = time.time()

    assert not timer.is_canceled

    while rclpy.ok() and time.time() - begin_time < 2.5 * period:
        rclpy.spin_once(node, period / 10)

    assert len(callbacks) == 2, 'should have received 2 callbacks, received %d' % len(callbacks)
    assert not timer.is_canceled

    timer.cancel()
    assert timer.is_canceled
    callbacks = []
    begin_time = time.time()
    while rclpy.ok() and time.time() - begin_time < 2.5 * period:
        rclpy.spin_once(node, period / 10)

    assert timer.is_canceled
    assert [] == callbacks, \
        'shouldn\'t have reveived any callback, received %d' % len(callbacks)

    timer.reset()
    assert not timer.is_canceled
    begin_time = time.time()
    while rclpy.ok() and time.time() - begin_time < 2.5 * period:
        rclpy.spin_once(node, period / 10)

    assert not timer.is_canceled
    assert len(callbacks) == 2, 'should have received 2 callbacks, received %d' % len(callbacks)
    node.destroy_timer(timer)
    rclpy.shutdown()

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


def test_timer_zero_callbacks1000hertz():
    func_launch(func_zero_callback, ['0.001'], 'received callbacks when not expected')


def test_timer_number_callbacks10hertz():
    func_launch(func_number_callbacks, ['0.1'], 'didn\'t receive the expected number of callbacks')


def test_timer_number_callbacks100hertz():
    func_launch(
        func_number_callbacks, ['0.01'], 'didn\'t receive the expected number of callbacks')


def test_timer_number_callbacks1000hertz():
    func_launch(
        func_number_callbacks, ['0.001'], 'didn\'t receive the expected number of callbacks')


def test_timer_cancel_reset_10hertz():
    func_launch(
        func_cancel_reset_timer, ['0.1'], 'didn\'t receive the expected number of callbacks')


def test_timer_cancel_reset_100hertz():
    func_launch(
        func_cancel_reset_timer, ['0.01'], 'didn\'t receive the expected number of callbacks')


def test_timer_cancel_reset_1000hertz():
    func_launch(
        func_cancel_reset_timer, ['0.001'], 'didn\'t receive the expected number of callbacks')
