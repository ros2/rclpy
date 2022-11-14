# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from rclpy.node import Node
from rclpy.utilities import timeout_sec_to_nsec
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.signals import SignalHandlerGuardCondition


def wait_for_message(
    msg_type,
    node: 'Node',
    topic: str,
    timeout_sec=-1
):
    """
    Wait for the next incoming message.

    :param msg_type: message type
    :param node: node you've created
    :param topic: topic name you've defined
    :return (True, msg) if a message was successfully received, (False, ()) if message
        could not be obtained or shutdown was triggered asynchronously on the context.
    """
    context = node.context
    wait_set = _rclpy.WaitSet(1, 1, 0, 0, 0, 0, context.handle)
    wait_set.clear_entities()

    sub = node.create_subscription(msg_type, topic, lambda _: None, 1)
    wait_set.add_subscription(sub.handle)
    sigint_gc = SignalHandlerGuardCondition(context)
    wait_set.add_guard_condition(sigint_gc.handle)

    timeout_nsec = timeout_sec_to_nsec(timeout_sec)
    wait_set.wait(timeout_nsec)

    subs_ready = wait_set.get_ready_entities('subscription')
    guards_ready = wait_set.get_ready_entities('guard_condition')

    if guards_ready:
        if sigint_gc.pointer in guards_ready:
            return (False, None)

    if subs_ready:
        if sub.handle.pointer in subs_ready:
            msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
            return (True, msg_info[0])

    return (False, None)
