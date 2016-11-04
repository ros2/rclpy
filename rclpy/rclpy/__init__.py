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

import logging
import os
import sys

from rclpy.exceptions import InvalidRCLPYImplementation
from rclpy.impl import excepthook
from rclpy.impl import implementation_singleton
from rclpy.impl import rmw_implementation_tools
from rclpy.impl.rmw_implementation_tools import RCLPY_IMPLEMENTATION_ENV_NAME
from rclpy.node import Node

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

S_TO_NS = 1000 * 1000 * 1000

# install the excepthook
excepthook.install_rclpy_excepthook()


def init(args=None):
    rclpy_rmw_env = os.getenv(RCLPY_IMPLEMENTATION_ENV_NAME, None)
    if rclpy_rmw_env is not None:
        available_rmw_implementations = rmw_implementation_tools.get_rmw_implementations()
        if rclpy_rmw_env not in available_rmw_implementations:
            logger = logging.getLogger('rclpy')
            logger.error(
                "The rmw implementation specified in 'RCLPY_IMPLEMENTATION={0}', "
                "is not one of the available implementations: {1}"
                .format(rclpy_rmw_env, available_rmw_implementations)
            )
            raise InvalidRCLPYImplementation()
        rmw_implementation_tools.select_rmw_implementation(rclpy_rmw_env)
    # This line changes what is in "_rclpy" to be the rmw implementation module that was imported.
    implementation_singleton.set_rclpy_implementation(
        rmw_implementation_tools.import_rmw_implementation()
    )

    # Now we can use _rclpy to call the implementation specific rclpy_init().
    return _rclpy.rclpy_init(args if args is not None else sys.argv)


def create_node(node_name):
    node_handle = _rclpy.rclpy_create_node(node_name)
    return Node(node_handle)


def spin_once(node, timeout_sec=None):
    wait_set = _rclpy.rclpy_get_zero_initialized_wait_set()

    _rclpy.rclpy_wait_set_init(wait_set, len(node.subscriptions), 0, 0)

    _rclpy.rclpy_wait_set_clear_subscriptions(wait_set)
    for subscription in node.subscriptions:
        _rclpy.rclpy_wait_set_add_subscription(wait_set, subscription.subscription_handle)
    if timeout_sec is None:
        timeout = -1
    else:
        timeout = int(timeout_sec) * S_TO_NS

    _rclpy.rclpy_wait(wait_set, timeout)

    # TODO(wjwwood): properly implement this by checking the contents of the wait_set.
    for subscription in node.subscriptions:
        msg = _rclpy.rclpy_take(subscription.subscription_handle, subscription.msg_type)
        if msg:
            subscription.callback(msg)


def ok():
    return _rclpy.rclpy_ok()


def shutdown():
    return _rclpy.rclpy_shutdown()


def get_rmw_implementation_identifier():
    return _rclpy.rclpy_get_rmw_implementation_identifier()
