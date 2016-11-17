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


def func_walltimer(args):
    import time
    import functools
    period = float(args[0])
    max_time = float(args[1])
    print('period: %f' % period)

    def timer_cb(timestamps):
        timestamps.append(time.time())

    import rclpy

    rclpy.init()
    node = rclpy.create_node('talker')
    timestamps = []
    nb_iterations = 0

    test_callback = functools.partial(timer_cb, timestamps=timestamps)

    timer = node.create_timer(period, test_callback)

    begin_time = time.time()

    while rclpy.ok() and time.time() - begin_time < max_time:
        rclpy.spin_once(node)
        nb_iterations += 1

    node.destroy_timer(timer)
    rclpy.shutdown()
    assert len(timestamps) == nb_iterations
    assert nb_iterations * period >= max_time - period, \
        'nb_iterations * period :%f < %f' % (nb_iterations * period, max_time - period)

    assert nb_iterations * period <= max_time + period, \
        'nb_iterations * period :%f > %f' % (nb_iterations * period, max_time + period)
    for i in range(1, nb_iterations):
        assert timestamps[i] - timestamps[i - 1] < period * 1.1

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


def test_timer_1hertz():
    func_launch(func_walltimer, ['1', '5'], 'failed to validate timer')


def test_timer_10hertz():
    func_launch(func_walltimer, ['0.1', '3'], 'failed to validate timer')


def test_timer_100hertz():
    func_launch(func_walltimer, ['0.01', '1'], 'failed to validate timer')
