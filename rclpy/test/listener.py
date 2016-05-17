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
import importlib
import os
import sys

# this is needed to allow rclpy to be imported from the build folder
sys.path.insert(0, os.getcwd())


def listener_cb(msg, message_name, received_messages):
    print('received: %r' % msg)
    received_messages.append(msg)


def listener(message_pkg, message_name, number_of_cycles):
    import rclpy
    from rclpy.qos import qos_profile_default

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
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-p', '--message_pkg', default='std_msgs',
                        help='name of the message package')
    parser.add_argument('-m', '--message_name', default='String',
                        help='name of the ROS message')
    parser.add_argument('-n', '--number_of_cycles', type=int, default=5,
                        help='number of sending attempts')
    args = parser.parse_args()
    try:
        listener(
            message_pkg=args.message_pkg,
            message_name=args.message_name,
            number_of_cycles=args.number_of_cycles)
    except KeyboardInterrupt:
        print('talker stopped cleanly')
    except BaseException:
        print('exception in talker:', file=sys.stderr)
        raise
