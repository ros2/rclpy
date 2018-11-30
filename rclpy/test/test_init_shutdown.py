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
    except Exception:
        print('exception in {0}():'.format(func.__name__), file=sys.stderr)
        traceback.print_exc()
        raise


def func_init():
    import rclpy
    context = rclpy.context.Context()
    try:
        rclpy.init(context=context)
    except RuntimeError:
        return False

    rclpy.shutdown(context=context)
    return True


def func_init_shutdown():
    import rclpy
    context = rclpy.context.Context()
    try:
        rclpy.init(context=context)
        rclpy.shutdown(context=context)
        return True
    except RuntimeError:
        return False


def func_init_shutdown_sequence():
    import rclpy
    try:
        # local
        context = rclpy.context.Context()
        rclpy.init(context=context)
        rclpy.shutdown(context=context)
        context = rclpy.context.Context()  # context cannot be reused but should not interfere
        rclpy.init(context=context)
        rclpy.shutdown(context=context)

        # global
        rclpy.init()
        rclpy.shutdown()
        rclpy.init()
        rclpy.shutdown()

        return True
    except RuntimeError:
        return False


def func_double_init():
    import rclpy
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        rclpy.init(context=context)
    except RuntimeError:
        return True
    finally:
        rclpy.shutdown(context=context)
    return False


def func_double_shutdown():
    import rclpy
    context = rclpy.context.Context()
    rclpy.init(context=context)
    rclpy.shutdown(context=context)
    try:
        rclpy.shutdown(context=context)
    except RuntimeError:
        return True

    return False


def func_create_node_without_init():
    import rclpy
    from rclpy.exceptions import NotInitializedException
    context = rclpy.context.Context()
    try:
        rclpy.create_node('foo', context=context)
    except NotInitializedException:
        return True
    return False


def func_launch(function, message):
    pool = multiprocessing.Pool(1)
    result = pool.apply(
        func=run_catch_report_raise,
        args=(function,)
    )

    assert result, message
    pool.close()
    pool.join()


def test_init():
    func_launch(func_init, 'failed to initialize rclpy')


def test_init_shutdown():
    func_launch(func_init_shutdown, 'failed to shutdown rclpy')


def test_init_shutdown_sequence():
    func_launch(func_init_shutdown_sequence, 'failed to initialize rclpy after shutdown')


def test_double_init():
    func_launch(func_double_init, 'Expected Runtime error when initializing rclpy twice')


def test_double_shutdown():
    func_launch(func_double_shutdown, 'Expected Runtime error when shutting down rclpy twice')


def test_create_node_without_init():
    func_launch(func_create_node_without_init, 'Expected NotInitializedException')
