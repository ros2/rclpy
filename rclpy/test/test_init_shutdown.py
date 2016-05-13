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


def func_init():
    import rclpy
    try:
        rclpy.init()
    except RuntimeError:
        return False

    rclpy.shutdown()
    return True


def func_init_shutdown():
    import rclpy
    try:
        rclpy.init()
        rclpy.shutdown()
        return True
    except RuntimeError:
        return False


def func_init_shutdown_sequence():
    import rclpy
    rclpy.init()
    rclpy.shutdown()
    try:
        rclpy.init()
        rclpy.shutdown()
        return True
    except RuntimeError:
        return False


def func_double_init():
    import rclpy
    rclpy.init()
    try:
        rclpy.init()
    except RuntimeError:
        rclpy.shutdown()
        return True
    rclpy.shutdown()
    return False


def func_double_shutdown():
    import rclpy
    rclpy.init()
    rclpy.shutdown()
    try:
        rclpy.shutdown()
    except RuntimeError:
        return True

    return False


def test_init():
    pool = multiprocessing.Pool(1)
    result = pool.apply(
        func=run_catch_report_raise,
        args=(func_init,)
    )

    assert result, 'failed to initialize rclpy'
    pool.close()
    pool.join()


def test_init_shutdown():
    pool = multiprocessing.Pool(1)
    result = pool.apply(
        func=run_catch_report_raise,
        args=(func_init_shutdown,)
    )

    assert result, 'failed to shutdown rclpy'
    pool.close()
    pool.join()


def test_init_shutdown_sequence():
    pool = multiprocessing.Pool(1)
    result = pool.apply(
        func=run_catch_report_raise,
        args=(func_init_shutdown_sequence,)
    )

    assert result, 'failed to initialize rclpy after shutdown'
    pool.close()
    pool.join()


def test_double_init():
    pool = multiprocessing.Pool(1)
    result = pool.apply(
        func=run_catch_report_raise,
        args=(func_double_init,)
    )

    assert result, 'Expected Runtime error when initializing rclpy twice'
    pool.close()
    pool.join()


def test_double_shutdown():
    result = pool = multiprocessing.Pool(1)
    pool.apply(
        func=run_catch_report_raise,
        args=(func_double_shutdown,)
    )

    assert result, 'Expected Runtime error when shutting down rclpy twice'
    pool.close()
    pool.join()
