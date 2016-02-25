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

import importlib
from multiprocessing import Array
from multiprocessing import Process
import os


def f_rmw(result):
    import rclpy
    import rclpy.selector
    if len(rclpy.selector.rmw_implementations) < 2:
        # Ignore this test, we need at least two implementations
        print('Need more than one rmw implementation installed')
        return

    for second_rmw in rclpy.selector.rmw_implementations[1:]:
        try:
            rmw_implementation = importlib.import_module(
                '._rclpy__{rmw_implementation}'.format(rmw_implementation=second_rmw),
                package='rclpy').rclpy_get_implementation_identifier()
            break
        except ImportError:
            pass

    orig_rmw = rclpy._rclpy

    rclpy.selector.select(second_rmw)

    rclpy.init([])

    set_rmw = rclpy._rclpy.rclpy_get_implementation_identifier()

    result[0] = (set_rmw == rmw_implementation)
    result[1] = (set_rmw != orig_rmw)


def f_rmw_env(result):
    import rclpy
    import rclpy.selector
    if len(rclpy.selector.rmw_implementations) < 2:
        # Ignore this test, we need at least two implementations
        print('Need more than one rmw implementation installed')
        return

    for second_rmw in rclpy.selector.rmw_implementations[1:]:
        try:
            rmw_implementation = importlib.import_module(
                '._rclpy__{rmw_implementation}'.format(rmw_implementation=second_rmw),
                package='rclpy').rclpy_get_implementation_identifier()
            break
        except ImportError:
            pass

    orig_rmw = rclpy._rclpy

    old_rclpy_rmw_env = os.environ.get('RCLPY_IMPLEMENTATION')
    os.environ['RCLPY_IMPLEMENTATION'] = second_rmw

    rclpy.init([])

    set_rmw = rclpy._rclpy.rclpy_get_implementation_identifier()

    result[0] = (set_rmw == rmw_implementation)
    result[1] = (set_rmw != orig_rmw)
    if old_rclpy_rmw_env is None:
        del os.environ['RCLPY_IMPLEMENTATION']
    else:
        os.environ['RCLPY_IMPLEMENTATION'] = old_rclpy_rmw_env


def test_switch_rmw():
    result = Array('b', [-1 for _ in range(2)])
    test_process = Process(target=f_rmw, args=[result])
    test_process.start()
    test_process.join()

    assert result[0] == 1
    assert result[1] == 1


def test_switch_rmw_env():
    result = Array('b', [-1 for _ in range(2)])
    test_process = Process(target=f_rmw_env, args=[result])
    test_process.start()
    test_process.join()

    assert result[0] == 1
    assert result[1] == 1
