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


def func_destroy_node():
    import rclpy
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_node1', context=context)
    ret = True
    try:
        node.destroy_node()
    except RuntimeError:
        ret = False
    finally:
        rclpy.shutdown(context=context)
    return ret


def func_destroy_node_twice():
    import rclpy
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_node2', context=context)
    assert node.destroy_node() is True
    assert node.destroy_node() is True
    return True


def func_destroy_corrupted_node():
    import rclpy
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_node2', context=context)
    assert node.destroy_node() is True
    org_handle = node._handle
    node._handle = 'garbage'
    ret = False
    try:
        node.destroy_node()
    except ValueError:
        ret = True
        node._handle = org_handle
        node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
    return ret


def func_destroy_timers():
    import rclpy
    context = rclpy.context.Context()
    rclpy.init(context=context)

    node = rclpy.create_node('test_node3', context=context)

    timer1 = node.create_timer(0.1, None)
    timer2 = node.create_timer(1, None)
    timer2  # noqa

    assert 2 == len(node.timers)

    assert node.destroy_timer(timer1) is True

    assert 1 == len(node.timers)
    try:
        assert node.destroy_node() is True
    except SystemError:
        ret = False
    else:
        assert 0 == len(node.timers)
        assert node.handle is None
        ret = True
    finally:
        rclpy.shutdown(context=context)
    return ret


def func_destroy_entities():
    import rclpy
    from test_msgs.msg import Primitives
    context = rclpy.context.Context()
    rclpy.init(context=context)

    node = rclpy.create_node('test_node4', context=context)

    timer = node.create_timer(0.1, None)
    timer  # noqa
    assert 1 == len(node.timers)
    pub1 = node.create_publisher(Primitives, 'pub1_topic')
    assert 2 == len(node.publishers)
    pub2 = node.create_publisher(Primitives, 'pub2_topic')
    pub2  # noqa
    assert 3 == len(node.publishers)
    sub1 = node.create_subscription(
        Primitives, 'sub1_topic', lambda msg: print('Received %r' % msg))
    assert 1 == len(node.subscriptions)
    sub2 = node.create_subscription(
        Primitives, 'sub2_topic', lambda msg: print('Received %r' % msg))
    sub2  # noqa
    assert 2 == len(node.subscriptions)

    assert node.destroy_publisher(pub1) is True
    assert 2 == len(node.publishers)

    assert node.destroy_subscription(sub1) is True
    assert 1 == len(node.subscriptions)

    try:
        assert node.destroy_node() is True
    except SystemError:
        ret = False
    else:
        assert 0 == len(node.timers)
        assert 0 == len(node.publishers)
        assert 0 == len(node.subscriptions)
        assert node.handle is None
        ret = True
    finally:
        rclpy.shutdown(context=context)
    return ret


def func_corrupt_node_handle():
    import rclpy
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_node5', context=context)
    try:
        node.handle = 'garbage'
        ret = False
    except AttributeError:
        node.destroy_node()
        ret = True
    finally:
        rclpy.shutdown(context=context)
    return ret


def func_launch(function, message):
    pool = multiprocessing.Pool(1)
    result = pool.apply(
        func=run_catch_report_raise,
        args=(function,)
    )

    assert result, message
    pool.close()
    pool.join()


def test_destroy_node():
    func_launch(func_destroy_node, 'failed to destroy node')


def test_destroy_node_twice():
    func_launch(func_destroy_node_twice, 'succeeded to destroy same node twice o_O')


def test_destroy_corrupted_node():
    func_launch(func_destroy_corrupted_node, 'destroyed a non existing node o_O')


def test_destroy_timers():
    func_launch(func_destroy_timers, 'failed to destroy node entities')


def test_destroy_entities():
    func_launch(func_destroy_entities, 'failed to destroy node entities')


def test_corrupt_node_handle():
    func_launch(func_corrupt_node_handle, 'successfully modified node handle o_O')
