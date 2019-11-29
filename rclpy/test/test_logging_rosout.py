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

import unittest

from parameterized import parameterized
from rcl_interfaces.msg import Log
import rclpy
from rclpy.executors import SingleThreadedExecutor


class TestLoggingRosout(unittest.TestCase):

    @parameterized.expand([
        ['enable_global_rosout_enable_node_rosout', True, True, True],
        ['enable_global_rosout_disable_node_rosout', True, False, False],
        ['disable_global_rosout_enable_node_rosout', False, True, False],
        ['disable_global_rosout_disable_node_rosout', False, False, False],
    ])
    def test_enable_rosout(
        self,
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

        # create subscriber of 'rosout' topic
        self.raw_subscription_msg = None  # None=No result yet
        node.create_subscription(
            Log,
            'rosout',
            self.raw_subscription_callback,
            1,
            raw=True
        )

        node.get_logger().info('SOMETHING')
        executor.spin_once(timeout_sec=1)
        if expected_data:
            self.assertIsNotNone(self.raw_subscription_msg, 'raw subscribe timed out')
            self.assertIs(
                type(self.raw_subscription_msg), bytes, 'raw subscribe did not return bytes'
            )
            self.assertNotEqual(len(self.raw_subscription_msg), 0, 'raw subscribe invalid length')
        else:
            self.assertIsNone(self.raw_subscription_msg)
        node.destroy_node()
        rclpy.shutdown(context=context)

    def raw_subscription_callback(self, msg):
        self.raw_subscription_msg = msg


if __name__ == '__main__':
    unittest.main()
