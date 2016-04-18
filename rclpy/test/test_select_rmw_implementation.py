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
import os
import sys
import traceback


def run_catch_report_raise(func, *args, **kwargs):
    try:
        return func(*args, **kwargs)
    except:
        print('exception in {0}():'.format(func.__name__), file=sys.stderr)
        traceback.print_exc()
        raise


def func_import_each_available_rmw_implementation(rmw_implementation):
    import rclpy
    from rclpy.impl.rmw_implementation_tools import get_rmw_implementations
    from rclpy.impl.rmw_implementation_tools import select_rmw_implementation

    rmw_implementations = get_rmw_implementations()
    assert(rmw_implementation in rmw_implementations)

    select_rmw_implementation(rmw_implementation)
    rclpy.init([])

    set_rmw = rclpy.get_rmw_implementation_identifier()

    assert set_rmw == rmw_implementation, \
        "expected '{0}' but got '{1}'".format(
            rmw_implementation, set_rmw)
    return ctypes.c_bool(True)


def test_import_each_available_rmw_implementation():
    from rclpy.impl.rmw_implementation_tools import get_rmw_implementations
    for rmw_implementation in get_rmw_implementations():
        pool = multiprocessing.Pool(1)
        result = pool.apply(
            func=run_catch_report_raise,
            args=(func_import_each_available_rmw_implementation, rmw_implementation,)
        )

        assert result
        pool.close()
        pool.join()


def func_select_rmw_implementation_by_environment(rmw_implementation):
    import rclpy
    from rclpy.impl.rmw_implementation_tools import get_rmw_implementations

    os.environ['RCLPY_IMPLEMENTATION'] = rmw_implementation

    assert('RCLPY_IMPLEMENTATION' in os.environ)

    rmw_implementations = get_rmw_implementations()
    assert(os.environ['RCLPY_IMPLEMENTATION'] in rmw_implementations)

    rclpy.init([])

    set_rmw = rclpy.get_rmw_implementation_identifier()

    assert set_rmw == rmw_implementation, \
        "expected '{0}' but got '{1}'".format(
            rmw_implementation, set_rmw)
    return ctypes.c_bool(True)


def test_select_rmw_implementation_by_environment():
    from rclpy.impl.rmw_implementation_tools import get_rmw_implementations
    orig_rclpy_implementation_env = os.environ.get('RCLPY_IMPLEMENTATION', None)
    for rmw_implementation in get_rmw_implementations():
        pool = multiprocessing.Pool(1)
        result = pool.apply(
            func=run_catch_report_raise,
            args=(func_select_rmw_implementation_by_environment, rmw_implementation,)
        )
        if orig_rclpy_implementation_env is None:
            if 'RCLPY_IMPLEMENTATION' in os.environ:
                del os.environ['RCLPY_IMPLEMENTATION']
        else:
            os.environ['RCLPY_IMPLEMENTATION'] = orig_rclpy_implementation_env

        assert result, "type('{0}'): '{1}'".format(type(result), result)
        pool.close()
        pool.join()
