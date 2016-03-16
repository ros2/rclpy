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

import functools
import os
import sys

from launch import LaunchDescriptor
from launch.launcher import AsynchronousLauncher
from launch.launcher import DefaultLauncher

this_dir = os.path.normpath(os.path.dirname(os.path.abspath(__file__)))


def listener_callback(msg, received_messages):
    print('received: ' + msg.data)
    received_messages.append(msg)


def test_talker_listener():
    launch_desc = LaunchDescriptor()
    launch_desc.add_process(
        cmd=[sys.executable, os.path.join(this_dir, 'talker.py')],
        name='test_talker_listener__talker'
    )
    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(launch_desc)
    async_launcher = AsynchronousLauncher(launcher)
    async_launcher.start()

    import rclpy

    rclpy.init([])

    from rclpy.qos import qos_profile_default

    from std_msgs.msg import String
    assert String.__class__._TYPE_SUPPORT is not None

    node = rclpy.create_node('listener')

    received_messages = []

    chatter_callback = functools.partial(
        listener_callback, received_messages=received_messages)
    # TODO(wjwwood): should the subscription object hang around?
    node.create_subscription(String, 'chatter', chatter_callback, qos_profile_default)

    spin_count = 0
    while len(received_messages) == 0 and spin_count < 10 and launcher.is_launch_running():
        rclpy.spin_once(node)  # This will wait 1 second or until something has been done.
        spin_count += 1

    async_launcher.terminate()
    async_launcher.join()

    assert len(received_messages) > 0, "Should have received a message from talker"
