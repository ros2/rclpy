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

import rclpy

from rclpy.client import Client
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_default
from rclpy.service import Service
from rclpy.subscription import Subscription


class Node:

    def __init__(self, handle):
        self.clients = []
        self.handle = handle
        self.publishers = []
        self.services = []
        self.subscriptions = []

    def create_publisher(self, msg_type, topic, qos_profile=qos_profile_default):
        # this line imports the typesupport for the message module if not already done
        if msg_type.__class__._TYPE_SUPPORT is None:
            msg_type.__class__.__import_type_support__()
        publisher_handle = rclpy._rclpy.rclpy_create_publisher(
            self.handle, msg_type, topic, qos_profile.get_c_qos_profile())
        publisher = Publisher(publisher_handle, msg_type, topic, qos_profile)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos_profile=qos_profile_default):
        # this line imports the typesupport for the message module if not already done
        if msg_type.__class__._TYPE_SUPPORT is None:
            msg_type.__class__.__import_type_support__()
        [subscription_handle, subscription_pointer] = rclpy._rclpy.rclpy_create_subscription(
            self.handle, msg_type, topic, qos_profile.get_c_qos_profile())

        subscription = Subscription(
            subscription_handle, subscription_pointer, msg_type,
            topic, callback, qos_profile)
        self.subscriptions.append(subscription)
        return subscription

    def create_client(self, srv_type, srv_name, qos_profile=qos_profile_default):
        if srv_type.__class__._TYPE_SUPPORT is None:
            srv_type.__class__.__import_type_support__()
        [client_handle, client_pointer] = rclpy._rclpy.rclpy_create_client(
            self.handle,
            srv_type,
            srv_name,
            qos_profile.get_c_qos_profile())
        client = Client(client_handle, client_pointer, srv_type, srv_name, qos_profile)
        self.clients.append(client)
        return client

    def create_service(self, srv_type, srv_name, callback, qos_profile=qos_profile_default):
        if srv_type.__class__._TYPE_SUPPORT is None:
            srv_type.__class__.__import_type_support__()
        [service_handle, service_pointer] = rclpy._rclpy.rclpy_create_service(
            self.handle,
            srv_type,
            srv_name,
            qos_profile.get_c_qos_profile())
        service = Service(
            service_handle, service_pointer, srv_type, srv_name, callback, qos_profile)
        self.services.append(service)
        return service

    def get_topic_names_and_types(self):
        return rclpy._rclpy.rclpy_get_topic_names_and_types(self.handle)

    def __del__(self):
        for sub in self.subscriptions:
            rclpy._rclpy.rclpy_destroy_entity(
                'subscription', sub.subscription_handle, self.handle)
        for pub in self.publishers:
            rclpy._rclpy.rclpy_destroy_entity(
                'publisher', pub.publisher_handle, self.handle)
        for cli in self.clients:
            rclpy._rclpy.rclpy_destroy_entity(
                'client', cli.client_handle, self.handle)
        for srv in self.services:
            rclpy._rclpy.rclpy_destroy_entity(
                'service', srv.service_handle, self.handle)
        rclpy._rclpy.rclpy_destroy_node(self.handle)
