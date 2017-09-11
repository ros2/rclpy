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
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
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


class Node:

    def __init__(self, node_name, *, namespace=None):
        self.clients = []
        self._handle = None
        self.publishers = []
        self.services = []
        self.subscriptions = []
        self.timers = []
        self._default_callback_group = MutuallyExclusiveCallbackGroup()

        namespace = namespace or ''
        if not ok():
            raise NotInitializedException('cannot create node')
        try:
            node_handle = _rclpy.rclpy_create_node(node_name, namespace)
            self._handle = node_handle
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
        if msg_type.__class__._TYPE_SUPPORT is None:
            msg_type.__class__.__import_type_support__()
        if msg_type.__class__._TYPE_SUPPORT is None:
            raise NoTypeSupportImportedException
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
        if msg_type.__class__._TYPE_SUPPORT is None:
            msg_type.__class__.__import_type_support__()
        if msg_type.__class__._TYPE_SUPPORT is None:
            raise NoTypeSupportImportedException
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
        if srv_type.__class__._TYPE_SUPPORT is None:
            srv_type.__class__.__import_type_support__()
        if srv_type.__class__._TYPE_SUPPORT is None:
            raise NoTypeSupportImportedException
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
        if srv_type.__class__._TYPE_SUPPORT is None:
            srv_type.__class__.__import_type_support__()
        if srv_type.__class__._TYPE_SUPPORT is None:
            raise NoTypeSupportImportedException
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

    def destroy_publisher(self, publisher):
        for pub in self.publishers:
            if pub.publisher_handle == publisher.publisher_handle:
                _rclpy.rclpy_destroy_node_entity(
                    'publisher', pub.publisher_handle, self.handle)
                self.publishers.remove(pub)
                return True
        return False

    def destroy_subscription(self, subscription):
        for sub in self.subscriptions:
            if sub.subscription_handle == subscription.subscription_handle:
                _rclpy.rclpy_destroy_node_entity(
                    'subscription', sub.subscription_handle, self.handle)
                self.subscriptions.remove(sub)
                return True
        return False

    def destroy_service(self, service):
        for srv in self.services:
            if srv.service_handle == service.service_handle:
                _rclpy.rclpy_destroy_node_entity(
                    'service', srv.service_handle, self.handle)
                self.services.remove(srv)
                return True
        return False

    def destroy_client(self, client):
        for cli in self.clients:
            if cli.client_handle == client.client_handle:
                _rclpy.rclpy_destroy_node_entity(
                    'client', cli.client_handle, self.handle)
                self.clients.remove(cli)
                return True
        return False

    def destroy_timer(self, timer):
        for tmr in self.timers:
            if tmr.timer_handle == timer.timer_handle:
                _rclpy.rclpy_destroy_entity(
                    'timer', tmr.timer_handle)
                self.timers.remove(tmr)
                return True
        return False

    def destroy_node(self):
        ret = True
        if self.handle is None:
            return ret

        # ensure that the passed node contains a valid capsule
        class_ = self.handle.__class__
        if not class_ or class_.__name__ != 'PyCapsule':
            raise ValueError('The node handle must be a PyCapsule')

        for sub in list(self.subscriptions):
            destroyed = _rclpy.rclpy_destroy_node_entity(
                'subscription', sub.subscription_handle, self.handle)
            if destroyed:
                self.subscriptions.remove(sub)
            ret &= destroyed
        for pub in list(self.publishers):
            destroyed = _rclpy.rclpy_destroy_node_entity(
                'publisher', pub.publisher_handle, self.handle)
            if destroyed:
                self.publishers.remove(pub)
            ret &= destroyed
        for cli in list(self.clients):
            destroyed = _rclpy.rclpy_destroy_node_entity(
                'client', cli.client_handle, self.handle)
            if destroyed:
                self.clients.remove(cli)
            ret &= destroyed
        for srv in list(self.services):
            destroyed = _rclpy.rclpy_destroy_node_entity(
                'service', srv.service_handle, self.handle)
            if destroyed:
                self.services.remove(srv)
            ret &= destroyed
        for tmr in list(self.timers):
            destroyed = _rclpy.rclpy_destroy_entity('timer', tmr.timer_handle)
            if destroyed:
                self.timers.remove(tmr)
            ret &= destroyed
        destroyed = _rclpy.rclpy_destroy_entity('node', self.handle)
        if destroyed:
            self._handle = None
        ret &= destroyed
        return ret

    def get_topic_names_and_types(self, no_demangle=False):
        return _rclpy.rclpy_get_topic_names_and_types(self.handle, no_demangle)

    def get_service_names_and_types(self):
        return _rclpy.rclpy_get_service_names_and_types(self.handle)

    def get_node_names(self):
        return _rclpy.rclpy_get_node_names(self.handle)

    def __del__(self):
        self.destroy_node()
