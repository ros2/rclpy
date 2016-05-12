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

import ctypes
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


def func_init_shutdown():
    import rclpy
    rclpy.init()
    return ctypes.c_bool(rclpy.shutdown() is None)


def func_init_shutdown_sequence():
    import rclpy
    rclpy.init()
    rclpy.shutdown()
    rclpy.init()
    return ctypes.c_bool(rclpy.shutdown() is None)


def func_double_init():
    import rclpy
    rclpy.init()
    try:
        rclpy.init()
    except RuntimeError:
        rclpy.shutdown()
        return ctypes.c_bool(True)

    return ctypes.c_bool(rclpy.shutdown() is None)


def func_double_shutdown():
    import rclpy
    rclpy.init()
    rclpy.shutdown()
    try:
        rclpy.shutdown()
    except RuntimeError:
        return ctypes.c_bool(True)

    return ctypes.c_bool(False)


def test_init_shutdown():
    pool = multiprocessing.Pool(1)
    testlist = [
        [func_init_shutdown, 'failed to shutdown rclpy'],
        [func_init_shutdown_sequence, 'failed to initialize rclpy after shutdown'],
        [func_double_init, 'Expected Runtime error when initializing rclpy twice'],
        [func_double_shutdown, 'Expected Runtime error when shutting down rclpy twice']
    ]

    for fct in testlist:
        result = pool.apply(
            func=run_catch_report_raise,
            args=(fct[0],)
        )

        assert result, fct[1]

    pool.close()
    pool.join()
