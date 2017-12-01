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

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.constants import S_TO_NS
from rclpy.exceptions import NotInitializedException
from rclpy.exceptions import NoTypeSupportImportedException
from rclpy.expand_topic_name import expand_topic_name
from rclpy.guard_condition import GuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.logging import get_logger
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_default, qos_profile_services_default
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.timer import WallTimer
from rclpy.utilities import ok
from rclpy.validate_full_topic_name import validate_full_topic_name
from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name
from rclpy.validate_topic_name import validate_topic_name


def check_for_type_support(msg_type):
    try:
        ts = msg_type.__class__._TYPE_SUPPORT
    except AttributeError as e:
        e.args = (
            e.args[0] +
            ' This might be a ROS 1 message type but it should be a ROS 2 message type.'
            ' Make sure to source your ROS 2 workspace after your ROS 1 workspace.',
            *e.args[1:])
        raise
    if ts is None:
        msg_type.__class__.__import_type_support__()
    if msg_type.__class__._TYPE_SUPPORT is None:
        raise NoTypeSupportImportedException()


class Node:

    def __init__(self, node_name, *, namespace=None):
        self._handle = None
        # TODO(dhood): get logger name from rcl, use namespace (with slashes converted)
        self._logger = get_logger(node_name)
        self.publishers = []
        self.subscriptions = []
        self.clients = []
        self.services = []
        self.timers = []
        self.guards = []
        self._default_callback_group = MutuallyExclusiveCallbackGroup()

        namespace = namespace or ''
        if not ok():
            raise NotInitializedException('cannot create node')
        try:
            self._handle = _rclpy.rclpy_create_node(node_name, namespace)
        except ValueError:
            # these will raise more specific errors if the name or namespace is bad
            validate_node_name(node_name)
            # emulate what rcl_node_init() does to accept '' and relative namespaces
            if not namespace:
                namespace = '/'
            if not namespace.startswith('/'):
                namespace = '/' + namespace
            validate_namespace(namespace)
            # Should not get to this point
            raise RuntimeError('rclpy_create_node failed for unknown reason')

    @property
    def handle(self):
        return self._handle

    @handle.setter
    def handle(self, value):
        raise AttributeError('handle cannot be modified after node creation')

    def get_name(self):
        return _rclpy.rclpy_get_node_name(self.handle)

    def get_namespace(self):
        return _rclpy.rclpy_get_node_namespace(self.handle)

    def get_logger(self):
        return self._logger

    def _validate_topic_or_service_name(self, topic_or_service_name, *, is_service=False):
        name = self.get_name()
        namespace = self.get_namespace()
        validate_node_name(name)
        validate_namespace(namespace)
        validate_topic_name(topic_or_service_name, is_service=is_service)
        expanded_topic_or_service_name = expand_topic_name(topic_or_service_name, name, namespace)
        validate_full_topic_name(expanded_topic_or_service_name, is_service=is_service)

    def create_publisher(self, msg_type, topic, *, qos_profile=qos_profile_default):
        # this line imports the typesupport for the message module if not already done
        check_for_type_support(msg_type)
        failed = False
        try:
            publisher_handle = _rclpy.rclpy_create_publisher(
                self.handle, msg_type, topic, qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(topic)
        publisher = Publisher(publisher_handle, msg_type, topic, qos_profile, self.handle)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(
            self, msg_type, topic, callback, *, qos_profile=qos_profile_default,
            callback_group=None):
        if callback_group is None:
            callback_group = self._default_callback_group
        # this line imports the typesupport for the message module if not already done
        check_for_type_support(msg_type)
        failed = False
        try:
            [subscription_handle, subscription_pointer] = _rclpy.rclpy_create_subscription(
                self.handle, msg_type, topic, qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(topic)

        subscription = Subscription(
            subscription_handle, subscription_pointer, msg_type,
            topic, callback, callback_group, qos_profile, self.handle)
        self.subscriptions.append(subscription)
        callback_group.add_entity(subscription)
        return subscription

    def create_client(
            self, srv_type, srv_name, *, qos_profile=qos_profile_services_default,
            callback_group=None):
        if callback_group is None:
            callback_group = self._default_callback_group
        check_for_type_support(srv_type)
        failed = False
        try:
            [client_handle, client_pointer] = _rclpy.rclpy_create_client(
                self.handle,
                srv_type,
                srv_name,
                qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(srv_name, is_service=True)
        client = Client(
            self.handle, client_handle, client_pointer, srv_type, srv_name, qos_profile,
            callback_group)
        self.clients.append(client)
        callback_group.add_entity(client)
        return client

    def create_service(
            self, srv_type, srv_name, callback, *, qos_profile=qos_profile_services_default,
            callback_group=None):
        if callback_group is None:
            callback_group = self._default_callback_group
        check_for_type_support(srv_type)
        failed = False
        try:
            [service_handle, service_pointer] = _rclpy.rclpy_create_service(
                self.handle,
                srv_type,
                srv_name,
                qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(srv_name, is_service=True)
        service = Service(
            self.handle, service_handle, service_pointer,
            srv_type, srv_name, callback, callback_group, qos_profile)
        self.services.append(service)
        callback_group.add_entity(service)
        return service

    def create_timer(self, timer_period_sec, callback, callback_group=None):
        timer_period_nsec = int(float(timer_period_sec) * S_TO_NS)
        if callback_group is None:
            callback_group = self._default_callback_group
        timer = WallTimer(callback, callback_group, timer_period_nsec)

        self.timers.append(timer)
        callback_group.add_entity(timer)
        return timer

    def create_guard_condition(self, callback, callback_group=None):
        if callback_group is None:
            callback_group = self._default_callback_group
        guard = GuardCondition(callback, callback_group)

        self.guards.append(guard)
        callback_group.add_entity(guard)
        return guard

    def destroy_publisher(self, publisher):
        for pub in self.publishers:
            if pub.publisher_handle == publisher.publisher_handle:
                _rclpy.rclpy_destroy_node_entity(pub.publisher_handle, self.handle)
                self.publishers.remove(pub)
                return True
        return False

    def destroy_subscription(self, subscription):
        for sub in self.subscriptions:
            if sub.subscription_handle == subscription.subscription_handle:
                _rclpy.rclpy_destroy_node_entity(sub.subscription_handle, self.handle)
                self.subscriptions.remove(sub)
                return True
        return False

    def destroy_client(self, client):
        for cli in self.clients:
            if cli.client_handle == client.client_handle:
                _rclpy.rclpy_destroy_node_entity(cli.client_handle, self.handle)
                self.clients.remove(cli)
                return True
        return False

    def destroy_service(self, service):
        for srv in self.services:
            if srv.service_handle == service.service_handle:
                _rclpy.rclpy_destroy_node_entity(srv.service_handle, self.handle)
                self.services.remove(srv)
                return True
        return False

    def destroy_timer(self, timer):
        for tmr in self.timers:
            if tmr.timer_handle == timer.timer_handle:
                _rclpy.rclpy_destroy_entity(tmr.timer_handle)
                self.timers.remove(tmr)
                return True
        return False

    def destroy_guard_condition(self, guard):
        for gc in self.guards:
            if gc.guard_handle == guard.guard_handle:
                _rclpy.rclpy_destroy_entity(gc.guard_handle)
                self.guards.remove(gc)
                return True
        return False

    def destroy_node(self):
        ret = True
        if self.handle is None:
            return ret

        while self.publishers:
            pub = self.publishers.pop()
            _rclpy.rclpy_destroy_node_entity(pub.publisher_handle, self.handle)
        while self.subscriptions:
            sub = self.subscriptions.pop()
            _rclpy.rclpy_destroy_node_entity(sub.subscription_handle, self.handle)
        while self.clients:
            cli = self.clients.pop()
            _rclpy.rclpy_destroy_node_entity(cli.client_handle, self.handle)
        while self.services:
            srv = self.services.pop()
            _rclpy.rclpy_destroy_node_entity(srv.service_handle, self.handle)
        while self.timers:
            tmr = self.timers.pop()
            _rclpy.rclpy_destroy_entity(tmr.timer_handle)
        while self.guards:
            gc = self.guards.pop()
            _rclpy.rclpy_destroy_entity(gc.guard_handle)

        _rclpy.rclpy_destroy_entity(self.handle)
        self._handle = None
        return ret

    def get_topic_names_and_types(self, no_demangle=False):
        return _rclpy.rclpy_get_topic_names_and_types(self.handle, no_demangle)

    def get_service_names_and_types(self):
        return _rclpy.rclpy_get_service_names_and_types(self.handle)

    def get_node_names(self):
        return _rclpy.rclpy_get_node_names(self.handle)

    def __del__(self):
        self.destroy_node()
