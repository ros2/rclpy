# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

import sys

from rclpy.constants import S_TO_NS
from rclpy.exceptions import NotInitializedException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier  # noqa
from rclpy.utilities import ok
from rclpy.utilities import shutdown  # noqa
from rclpy.utilities import try_shutdown  # noqa
from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name


def init(*, args=None):
    return _rclpy.rclpy_init(args if args is not None else sys.argv)


def create_node(node_name, *, namespace=None):
    namespace = namespace or ''
    failed = False
    if not ok():
        raise NotInitializedException('cannot create node')
    try:
        node_handle = _rclpy.rclpy_create_node(node_name, namespace)
    except ValueError:
        failed = True
    if failed:
        # these will raise more specific errors if the name or namespace is bad
        validate_node_name(node_name)
        # emulate what rcl_node_init() does to accept '' and relative namespaces
        if not namespace:
            namespace = '/'
        if not namespace.startswith('/'):
            namespace = '/' + namespace
        validate_namespace(namespace)
    return Node(node_handle)


def spin_once(node, *, timeout_sec=None):
    wait_set = _rclpy.rclpy_get_zero_initialized_wait_set()

    active_timer_list = [
        t for t in node.timers if not _rclpy.rclpy_is_timer_canceled(t.timer_handle)]
    _rclpy.rclpy_wait_set_init(
        wait_set,
        len(node.subscriptions),
        1,
        len(active_timer_list),
        len(node.clients),
        len(node.services))

    [sigint_gc, sigint_gc_handle] = _rclpy.rclpy_get_sigint_guard_condition()
    entities = {
        'subscription': (node.subscriptions, 'subscription_handle'),
        'client': (node.clients, 'client_handle'),
        'service': (node.services, 'service_handle'),
        'timer': (active_timer_list, 'timer_handle'),
    }
    for entity, (handles, handle_name) in entities.items():
        _rclpy.rclpy_wait_set_clear_entities(entity, wait_set)
        for h in handles:
            _rclpy.rclpy_wait_set_add_entity(
                entity, wait_set, h.__getattribute__(handle_name)
            )
    _rclpy.rclpy_wait_set_clear_entities('guard_condition', wait_set)
    _rclpy.rclpy_wait_set_add_entity('guard_condition', wait_set, sigint_gc)

    if timeout_sec is None:
        timeout = -1
    else:
        timeout = int(float(timeout_sec) * S_TO_NS)

    _rclpy.rclpy_wait(wait_set, timeout)

    guard_condition_ready_list = _rclpy.rclpy_get_ready_entities('guard_condition', wait_set)
    if sigint_gc_handle in guard_condition_ready_list:
        raise KeyboardInterrupt

    timer_ready_list = _rclpy.rclpy_get_ready_entities('timer', wait_set)
    for tmr in [t for t in active_timer_list if t.timer_pointer in timer_ready_list]:
        if _rclpy.rclpy_is_timer_ready(tmr.timer_handle):
            _rclpy.rclpy_call_timer(tmr.timer_handle)
            tmr.callback()

    sub_ready_list = _rclpy.rclpy_get_ready_entities('subscription', wait_set)
    for sub in [s for s in node.subscriptions if s.subscription_pointer in sub_ready_list]:
        msg = _rclpy.rclpy_take(sub.subscription_handle, sub.msg_type)
        if msg:
            sub.callback(msg)

    client_ready_list = _rclpy.rclpy_get_ready_entities('client', wait_set)
    for client in [c for c in node.clients if c.client_pointer in client_ready_list]:
        response = _rclpy.rclpy_take_response(
            client.client_handle, client.srv_type.Response, client.sequence_number)
        if response:
            # clients spawn their own thread to wait for a response in the wait_for_future function
            # users can either use this mechanism or monitor the content of
            # client.response themselves to check if a response have been received
            client.response = response

    service_ready_list = _rclpy.rclpy_get_ready_entities('service', wait_set)
    for srv in [s for s in node.services if s.service_pointer in service_ready_list]:
        request_and_header = _rclpy.rclpy_take_request(srv.service_handle, srv.srv_type.Request)
        if request_and_header is None:
            continue
        [request, header] = request_and_header
        if request:
            response = srv.callback(request, srv.srv_type.Response())
            srv.send_response(response, header)
