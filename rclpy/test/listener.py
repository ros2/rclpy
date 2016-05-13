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

import argparse
import functools
import os
import sys

# this is needed to allow rclpy to be imported from the build folder
sys.path.insert(0, os.getcwd())


def listener_cb(msg, message_name, received_messages):
    if message_name == 'String':
        print('received: {})'.format(msg.data))
    elif message_name == 'TransformStamped':
        print('received: translation({},{},{})), rotation({},{},{},{})'.format(
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w))
    elif message_name == 'Imu':
        print('received: ({})'.format(msg.angular_velocity_covariance))
    elif message_name == 'LaserScan':
        print('received: ({})'.format(msg.intensities))
    elif message_name == 'PointCloud2':
        print('received: ({})'.format([f.name for f in msg.fields]))
    else:
        raise NotImplementedError('no test coverage for {}'.format(message_name))
    received_messages.append(msg)


def listener(message_pkg, message_name, rmw_implementation, number_of_cycles):
    import rclpy
    from rclpy.qos import qos_profile_default
    import importlib

    rclpy.init([])

    # TODO(wjwwood) move this import back to the module level when
    # it is possible to import the messages before rclpy.init().
    module = importlib.import_module(message_pkg + '.msg')
    msg_mod = getattr(module, message_name)
    assert msg_mod.__class__._TYPE_SUPPORT is not None

    node = rclpy.create_node('listener')

    received_messages = []

    chatter_callback = functools.partial(
        listener_cb, message_name=message_name, received_messages=received_messages)

    node.create_subscription(
        msg_mod,
        'chatter',
        chatter_callback,
        qos_profile_default)

    spin_count = 1
    print('talker: beginning loop')
    while rclpy.ok() and spin_count < number_of_cycles and len(received_messages) == 0:
        rclpy.spin_once(node)
        spin_count += 1
    rclpy.shutdown()

    assert len(received_messages) > 0,\
        'Should have received a {} message from talker'.format(message_name)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--message_pkg', default=None)
    parser.add_argument('-m', '--message_name', default=None)
    parser.add_argument('-r', '--rmw_implementation', default=None)
    parser.add_argument('-n', '--number_of_cycles', default=5, type=int)
    args = parser.parse_args()
    try:
        listener(
            message_pkg=args.message_pkg,
            message_name=args.message_name,
            rmw_implementation=args.rmw_implementation,
            number_of_cycles=args.number_of_cycles)
    except KeyboardInterrupt:
        print('talker stopped cleanly')
    except BaseException:
        print('exception in talker:', file=sys.stderr)
        raise
