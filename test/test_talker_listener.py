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

from functools import partial
from multiprocessing import Process
from time import sleep


def talker():
    import rclpy
    from rclpy.qos import qos_profile_default

    from std_msgs.msg import String
    assert String.__class__._TYPE_SUPPORT is not None
    rclpy.init([])

    node = rclpy.create_node('talker')

    chatter_pub = node.create_publisher(String, 'chatter', qos_profile_default)

    msg = String()

    i = 1
    while True:
        msg.data = 'Hello World: {0}'.format(i)
        i += 1
        chatter_pub.publish(msg)
        sleep(1)


def listener_callback(msg, talker_process, received_messages):
    import rclpy
    talker_process.terminate()
    received_messages.append(msg)
    rclpy.shutdown()


def test_rclpy_talker_listener():
    talker_process = Process(target=talker)
    talker_process.start()

    import rclpy
    from rclpy.qos import qos_profile_default

    rclpy.init([])

    node = rclpy.create_node('listener')

    received_messages = []

    chatter_callback = partial(
        listener_callback, talker_process=talker_process, received_messages=received_messages)

    from std_msgs.msg import String
    assert String.__class__._TYPE_SUPPORT is not None

    node.create_subscription(String, 'chatter', chatter_callback, qos_profile_default)

    rclpy.spin(node)

    talker_process.join()

    assert len(received_messages) > 0, "Should have received a message from talker"
