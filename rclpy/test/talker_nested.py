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

# this is needed to allow rclpy to be imported from the build folder
sys.path.insert(0, os.getcwd())


def talker_nested():
    import rclpy
    from rclpy.qos import qos_profile_default

    rclpy.init([])

    # TODO(wjwwood) move this import back to the module level when
    # it is possible to import the messages before rclpy.init().
    from geometry_msgs.msg import TransformStamped

    node = rclpy.create_node('talker_nested')

    chatter_nested_pub = node.create_publisher(
        TransformStamped, 'chatter_nested',
        qos_profile_default)

    msg = TransformStamped()
    i = 1
    print('talker: beginning loop')
    while True:
        msg.header.frame_id = str('blabla{}'.format(i))
        msg.child_frame_id = str('blablabla{}'.format(i + 1))
        msg.transform.translation.x = float(i + 2)
        msg.transform.rotation.x = float(i + 3)
        print('talker sending: frame_id: {}, child_frame: {}, translationx: {}, rotationx: {}'
              .format(msg.header.frame_id, msg.child_frame_id, msg.transform.translation.x,
                      msg.transform.rotation.x))
        chatter_nested_pub.publish(msg)
        i += 4
        time.sleep(1)

if __name__ == '__main__':
    try:
        talker_nested()
    except KeyboardInterrupt:
        print('talker stopped cleanly')
    except BaseException:
        print('exception in talker:', file=sys.stderr)
        raise
