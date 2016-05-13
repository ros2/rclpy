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
import os
import sys
import time

# this is needed to allow rclpy to be imported from the build folder
sys.path.insert(0, os.getcwd())


def fill_msg(msg, message_name, i):
    if message_name == 'String':
        msg.data = 'Hello World: {}'.format(i)
        print('talker sending: {}'.format(msg.data))
    elif message_name == 'TransformStamped':
        msg.transform.translation.x = float(i)
        msg.transform.translation.y = float(i + 1)
        msg.transform.translation.z = float(i + 2)
        msg.transform.rotation.x = float(i + 3)
        msg.transform.rotation.y = float(i + 4)
        msg.transform.rotation.z = float(i + 5)
        msg.transform.rotation.w = float(i + 6)
        print('talker sending: translation({},{},{}), rotation({},{},{},{})'.format(
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w))
    elif message_name == 'Imu':
        msg.angular_velocity_covariance = [float(x) for x in range(i, i + 9)]
        print('talker sending: ({})'.format(msg.angular_velocity_covariance))
    elif message_name == 'LaserScan':
        msg.intensities = [float(x) for x in range(i + 10)]
        print('talker sending: ({})'.format(msg.intensities))
    elif message_name == 'PointCloud2':
        from sensor_msgs.msg import PointField
        msg.fields = []
        for x in range(i + 2):
            tmpfield = PointField()
            tmpfield.name = 'toto' + str(x)
            msg.fields.append(tmpfield)
        print('talker sending: ({})'.format([f.name for f in msg.fields]))
    else:
        raise NotImplementedError('no test coverage for {}'.format(message_name))
    return msg


def talker(message_pkg, message_name, rmw_implementation, number_of_cycles):
    import rclpy
    from rclpy.qos import qos_profile_default
    import importlib

    rclpy.init([])

    # TODO(wjwwood) move this import back to the module level when
    # it is possible to import the messages before rclpy.init().
    module = importlib.import_module(message_pkg + '.msg')
    msg_mod = getattr(module, message_name)
    assert msg_mod.__class__._TYPE_SUPPORT is not None

    node = rclpy.create_node('talker')

    chatter_pub = node.create_publisher(msg_mod, 'chatter', qos_profile_default)

    msg = msg_mod()

    msg_count = 1
    print('talker: beginning loop')
    while rclpy.ok() and msg_count < number_of_cycles:
        msg = fill_msg(msg, message_name, msg_count)
        msg_count += 1
        chatter_pub.publish(msg)
        time.sleep(1)
    rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--message_pkg', default=None)
    parser.add_argument('-m', '--message_name', default=None)
    parser.add_argument('-r', '--rmw_implementation', default=None)
    parser.add_argument('-n', '--number_of_cycles', default=5, type=int)
    args = parser.parse_args()
    try:
        talker(
            message_pkg=args.message_pkg,
            message_name=args.message_name,
            rmw_implementation=args.rmw_implementation,
            number_of_cycles=args.number_of_cycles)
    except KeyboardInterrupt:
        print('talker stopped cleanly')
    except BaseException:
        print('exception in talker:', file=sys.stderr)
        raise
