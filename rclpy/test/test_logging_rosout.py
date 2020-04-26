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
from rclpy.logging import LoggingSeverity

TEST_PARAMETERS = [
    # name, enable_global_rosout_logs, enable_node_rosout, expected_data
    ('enable_global_rosout_enable_node_rosout', True, True, True),
    ('enable_global_rosout_disable_node_rosout', True, False, False),
    ('disable_global_rosout_enable_node_rosout', False, True, False),
    ('disable_global_rosout_disable_node_rosout', False, False, False),
]

receive_expected_rosout_msg = False  # True if expected rosout msg is received


def rosout_subscription_callback(msg):
    global receive_expected_rosout_msg
    if (LoggingSeverity(msg.level) == LoggingSeverity.INFO and msg.msg == 'SOMETHING'):
        receive_expected_rosout_msg = True


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

    global receive_expected_rosout_msg
    receive_expected_rosout_msg = False
    # create subscriber of 'rosout' topic
    node.create_subscription(
        Log,
        '/rosout',
        rosout_subscription_callback,
        1
    )

    cycle_count = 0
    while cycle_count < 5 and receive_expected_rosout_msg is False:
        node.get_logger().info('SOMETHING')
        cycle_count += 1
        executor.spin_once(timeout_sec=1)

    if expected_data:
        assert (receive_expected_rosout_msg is True)
    else:
        assert (receive_expected_rosout_msg is False)

    node.destroy_node()
    rclpy.shutdown(context=context)
