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

    _rclpy.rclpy_wait_set_init(
        wait_set,
        len(node.subscriptions), 0, len(node.timers),
        len(node.clients),
        len(node.services))

    for entity in ['subscription', 'client', 'service', 'timer']:
        _rclpy.rclpy_wait_set_clear_entities(entity, wait_set)
        if entity == 'subscription':
            for subscription in node.subscriptions:
                _rclpy.rclpy_wait_set_add_entity(
                    entity, wait_set, subscription.subscription_handle)
        elif entity == 'client':
            for client in node.clients:
                _rclpy.rclpy_wait_set_add_entity(
                    entity, wait_set, client.client_handle)
        elif entity == 'service':
            for service in node.services:
                _rclpy.rclpy_wait_set_add_entity(
                    entity, wait_set, service.service_handle)
        elif entity == 'timer':
            for timer in node.timers:
                _rclpy.rclpy_wait_set_add_entity(
                    entity, wait_set, timer.timer_handle)

    if timeout_sec is None:
        timeout = -1
    else:
        timeout = int(float(timeout_sec) * S_TO_NS)

    _rclpy.rclpy_wait(wait_set, timeout)

    sub_ready_list = _rclpy.rclpy_get_ready_subscriptions(wait_set)

    for sub in [s for s in node.subscriptions if s.subscription_pointer in sub_ready_list]:
        msg = _rclpy.rclpy_take(sub.subscription_handle, sub.msg_type)
        if msg:
            sub.callback(msg)
    client_ready_list = _rclpy.rclpy_get_ready_clients(wait_set)
    for cli in [c for c in node.clients if c.client_pointer in client_ready_list]:
        response = _rclpy.rclpy_take_response(
            cli.client_handle, cli.srv_type.Response, cli.sequence_number)
        if response:
            cli.response = response
            print('received response:\n%r\n' % response)

    service_ready_list = _rclpy.rclpy_get_ready_services(wait_set)
    for srv in [s for s in node.services if s.service_pointer in service_ready_list]:
        [request, header] = _rclpy.rclpy_take_request(srv.service_handle, srv.srv_type.Request)
        if request:
            response = srv.callback(request, srv.srv_type.Response())
            srv.send_response(response, header)

    timer_ready_list = _rclpy.rclpy_get_ready_timers(wait_set)
    for tmr in [t for t in node.timers if t.timer_pointer in timer_ready_list]:
        tmr.callback()
        _rclpy.rclpy_reset_timer(tmr.timer_handle)


def ok():
    return _rclpy.rclpy_ok()


def shutdown():
    return _rclpy.rclpy_shutdown()


def get_rmw_implementation_identifier():
    return _rclpy.rclpy_get_rmw_implementation_identifier()
