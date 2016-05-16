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

import os
import sys
import time

from launch import LaunchDescriptor
from launch.launcher import DefaultLauncher
from launch.exit_handler import primary_exit_handler, ignore_signal_exit_handler

this_dir = os.path.normpath(os.path.dirname(os.path.abspath(__file__)))


def launch_talker_listener(message_pkg, message_name, max_runtime_in_seconds=5):
    talker_file = 'talker.py'
    listener_file = 'listener.py'

    launch_desc = LaunchDescriptor()
    launch_desc.add_process(
        cmd=[sys.executable,
             os.path.join(this_dir, talker_file),
             '-p', message_pkg,
             '-m', message_name,
             '-n', str(max_runtime_in_seconds)],
        name='test_talker__{}'.format(message_name),
        exit_handler=ignore_signal_exit_handler
    )

    launch_desc.add_process(
        cmd=[sys.executable,
             os.path.join(this_dir, listener_file),
             '-p', message_pkg,
             '-m', message_name,
             '-n', str(max_runtime_in_seconds)],
        name='test_listener__{}'.format(message_name),
        exit_handler=primary_exit_handler
    )
    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(launch_desc)
    rc = launcher.launch()

    loop_cnt = 0
    while loop_cnt < max_runtime_in_seconds and launcher.is_launch_running():
        time.sleep(1)
        loop_cnt += 1

    assert not rc, 'test_talker_listener__{} failed'.format(message_name)


def test_talker():
    launch_talker_listener(
        message_pkg='std_msgs',
        message_name='String'
    )


def test_talker_listener_nested():
    launch_talker_listener(
        message_pkg='geometry_msgs',
        message_name='TransformStamped'
    )


def test_talker_listener_fixed_array():
    launch_talker_listener(
        message_pkg='sensor_msgs',
        message_name='Imu'
    )


def test_talker_listener_unbounded_array():
    launch_talker_listener(
        message_pkg='sensor_msgs',
        message_name='LaserScan'
    )


def test_talker_listener_nested_array():
    launch_talker_listener(
        message_pkg='sensor_msgs',
        message_name='PointCloud2'
    )
