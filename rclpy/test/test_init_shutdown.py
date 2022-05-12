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

import pytest
import rclpy
from rclpy import signals
from rclpy.exceptions import NotInitializedException
from rclpy.signals import SignalHandlerOptions


def test_init():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    rclpy.shutdown(context=context)


def test_init_with_unknown_ros_args():
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

    context = rclpy.context.Context()
    unknown_ros_args_error_pattern = r'\[\'unknown\'\]'
    with pytest.raises(_rclpy.UnknownROSArgsError, match=unknown_ros_args_error_pattern):
        rclpy.init(context=context, args=['--ros-args', 'unknown'])


def test_init_with_non_utf8_arguments():
    context = rclpy.context.Context()
    # Embed non decodable characters e.g. due to
    # wrong locale settings.
    # See PEP-383 for further reference.
    args = ['my-node.py', 'Ragnar\udcc3\udcb6k']
    with pytest.raises(UnicodeEncodeError):
        rclpy.init(context=context, args=args)


def test_init_shutdown_sequence():
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


def test_double_init():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        with pytest.raises(RuntimeError):
            rclpy.init(context=context)
    finally:
        rclpy.shutdown(context=context)


def test_double_shutdown():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    rclpy.shutdown(context=context)
    with pytest.raises(RuntimeError):
        rclpy.shutdown(context=context)


def test_create_node_without_init():
    context = rclpy.context.Context()
    with pytest.raises(NotInitializedException):
        rclpy.create_node('foo', context=context)


def test_init_with_domain_id():
    rclpy.init(domain_id=123)
    assert rclpy.get_default_context().get_domain_id() == 123
    rclpy.shutdown()
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=123)
    assert context.get_domain_id() == 123
    rclpy.shutdown(context=context)


def test_signal_handlers():
    rclpy.init()
    assert SignalHandlerOptions.ALL == signals.get_current_signal_handlers_options()
    rclpy.shutdown()
    assert SignalHandlerOptions.NO == signals.get_current_signal_handlers_options()


def test_init_with_invalid_domain_id():
    with pytest.raises(RuntimeError):
        rclpy.init(domain_id=-1)
