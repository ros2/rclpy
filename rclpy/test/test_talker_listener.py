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


def listener_nested_callback(msg, received_messages):
    print('received: frame_id={}, child_frame={}, translationx={}, rotationx{}'
          .format(msg.header.frame_id, msg.child_frame_id, msg.transform.translation.x,
                  msg.transform.rotation.x))
    received_messages.append(msg)


def listener_fixed_array_callback(msg, received_messages):
    print('received: ({})'.format(msg.angular_velocity_covariance))
    received_messages.append(msg)


def listener_unbounded_array_callback(msg, received_messages):
    print('received: ({})'.format(msg.intensities))
    received_messages.append(msg)


def listener_nested_array_callback(msg, received_messages):
    print('received: ({})'.format([f.name for f in msg.fields]))
    received_messages.append(msg)


def launch_talker_listener(test_file, message_pkg, message_name, callback):
    import rclpy
    from rclpy.qos import qos_profile_default
    import importlib

    launch_desc = LaunchDescriptor()
    launch_desc.add_process(
        cmd=[sys.executable, os.path.join(this_dir, test_file)],
        name='test_talker_listener__{}'.format(test_file)
    )
    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(launch_desc)
    async_launcher = AsynchronousLauncher(launcher)
    async_launcher.start()

    rclpy.init([])

    module = importlib.import_module(message_pkg + '.msg')
    msg_mod = getattr(module, message_name)
    assert msg_mod.__class__._TYPE_SUPPORT is not None

    node = rclpy.create_node(test_file[:test_file.rfind('.')])

    received_messages = []

    chatter_callback = functools.partial(
        callback, received_messages=received_messages)

    # TODO(wjwwood): should the subscription object hang around?
    node.create_subscription(
        msg_mod,
        test_file[:test_file.rfind('.')].replace('talker', 'chatter'),
        chatter_callback,
        qos_profile_default)

    spin_count = 0
    while len(received_messages) == 0 and spin_count < 10 and launcher.is_launch_running():
        rclpy.spin_once(node)  # This will wait 1 second or until something has been done.
        spin_count += 1

    async_launcher.terminate()
    async_launcher.join()
    rclpy.shutdown()
    assert len(received_messages) > 0,\
        'Should have received a {} message from talker'.format(message_name)


def test_talker_listener():
    launch_talker_listener(
        test_file='talker.py',
        message_pkg='std_msgs',
        message_name='String',
        callback=listener_callback)


def test_talker_listener_nested():
    launch_talker_listener(
        test_file='talker_nested.py',
        message_pkg='geometry_msgs',
        message_name='TransformStamped',
        callback=listener_nested_callback)


def test_talker_listener_fixed_array():
    launch_talker_listener(
        test_file='talker_fixed_array.py',
        message_pkg='sensor_msgs',
        message_name='Imu',
        callback=listener_fixed_array_callback)


def test_talker_listener_unbounded_array():
    launch_talker_listener(
        test_file='talker_unbounded_array.py',
        message_pkg='sensor_msgs',
        message_name='LaserScan',
        callback=listener_unbounded_array_callback)


def test_talker_listener_nested_array():
    launch_talker_listener(
        test_file='talker_nested_array.py',
        message_pkg='sensor_msgs',
        message_name='PointCloud2',
        callback=listener_nested_array_callback)
