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


def launch_talker_listener(
        message_pkg, message_name,
        rmw_implementation, max_runtime_in_seconds=5):
    talker_file = 'talker.py'
    listener_file = 'listener.py'

    launch_desc = LaunchDescriptor()
    launch_desc.add_process(
        cmd=[sys.executable,
             os.path.join(this_dir, talker_file),
             '-p', message_pkg,
             '-m', message_name,
             '-r', rmw_implementation,
             '-n', str(max_runtime_in_seconds)],
        name='test_talker__{}__{}'.format(rmw_implementation, message_name),
        exit_handler=ignore_signal_exit_handler
    )

    launch_desc.add_process(
        cmd=[sys.executable,
             os.path.join(this_dir, listener_file),
             '-p', message_pkg,
             '-m', message_name,
             '-r', rmw_implementation,
             '-n', str(max_runtime_in_seconds)],
        name='test_listener__{}__{}'.format(rmw_implementation, message_name),
        exit_handler=primary_exit_handler
    )
    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(launch_desc)
    rc = launcher.launch()

    loop_cnt = 0
    while loop_cnt < max_runtime_in_seconds and launcher.is_launch_running():
        time.sleep(1)
        loop_cnt += 1

    return rc


def launch_talker_listener_all_rmw(message_pkg, message_name, nbmessages=5):
    from rclpy.impl.rmw_implementation_tools import get_rmw_implementations

    rmw_implementations = get_rmw_implementations()
    err_message = ''

    for rmw_implementation in rmw_implementations:
        rc = launch_talker_listener(
            message_pkg=message_pkg,
            message_name=message_name,
            rmw_implementation=rmw_implementation
        )
        if rc:
            err_message += 'test_talker_listener__{}__{} failed\n'.format(
                rmw_implementation, message_name)
    return err_message


def test_talker():
    err_message = launch_talker_listener_all_rmw(
        message_pkg='std_msgs',
        message_name='String'
    )

    assert err_message == '', err_message


def test_talker_listener_nested():
    err_message = launch_talker_listener_all_rmw(
        message_pkg='geometry_msgs',
        message_name='TransformStamped'
    )

    assert err_message == '', err_message


def test_talker_listener_fixed_array():
    err_message = launch_talker_listener_all_rmw(
        message_pkg='sensor_msgs',
        message_name='Imu'
    )

    assert err_message == '', err_message


def test_talker_listener_unbounded_array():
    err_message = launch_talker_listener_all_rmw(
        message_pkg='sensor_msgs',
        message_name='LaserScan'
    )

    assert err_message == '', err_message


def test_talker_listener_nested_array():
    err_message = launch_talker_listener_all_rmw(
        message_pkg='sensor_msgs',
        message_name='PointCloud2'
    )

    assert err_message == '', err_message
