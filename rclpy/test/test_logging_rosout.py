# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from rcl_interfaces.msg import Log
import rclpy
from rclpy.executors import SingleThreadedExecutor

TEST_PARAMETERS = [
    ('enable_global_rosout_enable_node_rosout', True, True, True),
    ('enable_global_rosout_disable_node_rosout', True, False, False),
    ('disable_global_rosout_enable_node_rosout', False, True, False),
    ('disable_global_rosout_disable_node_rosout', False, False, False),
]

raw_subscription_msg = None  # None=No result yet


def raw_subscription_callback(msg):
    global raw_subscription_msg
    raw_subscription_msg = msg


@pytest.mark.parametrize(
    'name,enable_global_rosout_logs,enable_node_rosout,expected_data',
    TEST_PARAMETERS)
def test_enable_rosout(
    name,
    enable_global_rosout_logs,
    enable_node_rosout,
    expected_data
):
    if enable_global_rosout_logs:
        args = ['--ros-args', '--enable-rosout-logs']
    else:
        args = ['--ros-args', '--disable-rosout-logs']

    context = rclpy.context.Context()
    rclpy.init(context=context, args=args)
    executor = SingleThreadedExecutor(context=context)

    # create node
    node = rclpy.create_node(
        node_name='my_node_'+name,
        namespace='/my_ns',
        enable_rosout=enable_node_rosout,
        context=context
    )
    executor.add_node(node)

    global raw_subscription_msg
    raw_subscription_msg = None
    # create subscriber of 'rosout' topic
    node.create_subscription(
        Log,
        '/rosout',
        raw_subscription_callback,
        1,
        raw=True
    )

    node.get_logger().info('SOMETHING')
    executor.spin_once(timeout_sec=1)

    if expected_data:
        assert (raw_subscription_msg is not None)
        assert (type(raw_subscription_msg) == bytes)
        assert (len(raw_subscription_msg) != 0)
    else:
        assert (raw_subscription_msg is None)

    node.destroy_node()
    rclpy.shutdown(context=context)
