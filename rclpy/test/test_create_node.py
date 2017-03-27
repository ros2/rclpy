# Copyright 2017 Open Source Robotics Foundation, Inc.
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


def func_create_node(args):
    import rclpy

    rclpy.init()

    node_name = 'create_node_test'
    node = rclpy.create_node(node_name)
    assert node.get_name() == node_name

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


def test_create_node():
    func_launch(func_create_node, [], 'create node failed')
