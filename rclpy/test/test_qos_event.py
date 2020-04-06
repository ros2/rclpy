# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

import unittest
from unittest.mock import Mock

import rclpy
from rclpy.handle import Handle
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSPolicyKind
from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos_event import QoSLivelinessChangedInfo
from rclpy.qos_event import QoSLivelinessLostInfo
from rclpy.qos_event import QoSOfferedDeadlineMissedInfo
from rclpy.qos_event import QoSOfferedIncompatibleQoSInfo
from rclpy.qos_event import QoSPublisherEventType
from rclpy.qos_event import QoSRequestedDeadlineMissedInfo
from rclpy.qos_event import QoSRequestedIncompatibleQoSInfo
from rclpy.qos_event import QoSSubscriptionEventType
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos_event import UnsupportedEventTypeError
from rclpy.utilities import get_rmw_implementation_identifier

from test_msgs.msg import Empty as EmptyMsg


class TestQoSEvent(unittest.TestCase):
    is_fastrtps = False

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('TestQoSEvent',
                                      namespace='/rclpy/test',
                                      context=self.context)
        self.is_fastrtps = 'rmw_fastrtps' in get_rmw_implementation_identifier()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_publisher_constructor(self):
        callbacks = PublisherEventCallbacks()
        liveliness_callback = Mock()
        deadline_callback = Mock()
        incompatible_qos_callback = Mock()

        # No arg
        publisher = self.node.create_publisher(EmptyMsg, 'test_topic', 10)
        self.assertEqual(len(publisher.event_handlers), 0)
        self.node.destroy_publisher(publisher)

        # Arg with no callbacks
        publisher = self.node.create_publisher(
            EmptyMsg, 'test_topic', 10, event_callbacks=callbacks)
        self.assertEqual(len(publisher.event_handlers), 0)
        self.node.destroy_publisher(publisher)

        # Arg with one of the callbacks
        callbacks.deadline = deadline_callback
        publisher = self.node.create_publisher(
            EmptyMsg, 'test_topic', 10, event_callbacks=callbacks)
        self.assertEqual(len(publisher.event_handlers), 1)
        self.node.destroy_publisher(publisher)

        # Arg with two callbacks
        callbacks.liveliness = liveliness_callback
        publisher = self.node.create_publisher(
            EmptyMsg, 'test_topic', 10, event_callbacks=callbacks)
        self.assertEqual(len(publisher.event_handlers), 2)
        self.node.destroy_publisher(publisher)

        # Arg with three callbacks
        callbacks.incompatible_qos = incompatible_qos_callback
        try:
            publisher = self.node.create_publisher(
                EmptyMsg, 'test_topic', 10, event_callbacks=callbacks)
            self.assertEqual(len(publisher.event_handlers), 3)
            self.node.destroy_publisher(publisher)
        except UnsupportedEventTypeError:
            self.assertTrue(self.is_fastrtps)

    def test_subscription_constructor(self):
        callbacks = SubscriptionEventCallbacks()
        liveliness_callback = Mock()
        deadline_callback = Mock()
        message_callback = Mock()
        incompatible_qos_callback = Mock()

        # No arg
        subscription = self.node.create_subscription(EmptyMsg, 'test_topic', message_callback, 10)
        self.assertEqual(len(subscription.event_handlers), 0)
        self.node.destroy_subscription(subscription)

        # Arg with no callbacks
        subscription = self.node.create_subscription(
            EmptyMsg, 'test_topic', message_callback, 10, event_callbacks=callbacks)
        self.assertEqual(len(subscription.event_handlers), 0)
        self.node.destroy_subscription(subscription)

        # Arg with one of the callbacks
        callbacks.deadline = deadline_callback
        subscription = self.node.create_subscription(
            EmptyMsg, 'test_topic', message_callback, 10, event_callbacks=callbacks)
        self.assertEqual(len(subscription.event_handlers), 1)
        self.node.destroy_subscription(subscription)

        # Arg with two callbacks
        callbacks.liveliness = liveliness_callback
        subscription = self.node.create_subscription(
            EmptyMsg, 'test_topic', message_callback, 10, event_callbacks=callbacks)
        self.assertEqual(len(subscription.event_handlers), 2)
        self.node.destroy_subscription(subscription)

        # Arg with three callbacks
        callbacks.incompatible_qos = incompatible_qos_callback
        try:
            subscription = self.node.create_subscription(
                EmptyMsg, 'test_topic', message_callback, 10, event_callbacks=callbacks)
            self.assertEqual(len(subscription.event_handlers), 3)
            self.node.destroy_subscription(subscription)
        except UnsupportedEventTypeError:
            self.assertTrue(self.is_fastrtps)

    def _create_event_handle(self, parent_entity, event_type):
        with parent_entity.handle as parent_capsule:
            event_capsule = _rclpy.rclpy_create_event(event_type, parent_capsule)
        self.assertIsNotNone(event_capsule)
        return Handle(event_capsule)

    def _do_create_destroy(self, parent_entity, event_type):
        handle = self._create_event_handle(parent_entity, event_type)
        handle.destroy()

    def test_publisher_event_create_destroy(self):
        publisher = self.node.create_publisher(EmptyMsg, 'test_topic', 10)
        self._do_create_destroy(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED)
        self._do_create_destroy(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_LIVELINESS_LOST)
        try:
            self._do_create_destroy(
                publisher, QoSPublisherEventType.RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS)
        except UnsupportedEventTypeError:
            self.assertTrue(self.is_fastrtps)
        self.node.destroy_publisher(publisher)

    def test_subscription_event_create_destroy(self):
        message_callback = Mock()
        subscription = self.node.create_subscription(EmptyMsg, 'test_topic', message_callback, 10)
        self._do_create_destroy(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_LIVELINESS_CHANGED)
        self._do_create_destroy(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED)
        try:
            self._do_create_destroy(
                subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS)
        except UnsupportedEventTypeError:
            self.assertTrue(self.is_fastrtps)
        self.node.destroy_subscription(subscription)

    def test_call_publisher_rclpy_event_apis(self):
        # Go through the exposed apis and ensure that things don't explode when called
        # Make no assumptions about being able to actually receive the events
        publisher = self.node.create_publisher(EmptyMsg, 'test_topic', 10)
        wait_set = _rclpy.rclpy_get_zero_initialized_wait_set()
        with self.context.handle as context_handle:
            _rclpy.rclpy_wait_set_init(wait_set, 0, 0, 0, 0, 0, 3, context_handle)

        deadline_event_handle = self._create_event_handle(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED)
        with deadline_event_handle as capsule:
            deadline_event_index = _rclpy.rclpy_wait_set_add_entity('event', wait_set, capsule)
        self.assertIsNotNone(deadline_event_index)

        liveliness_event_handle = self._create_event_handle(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_LIVELINESS_LOST)
        with liveliness_event_handle as capsule:
            liveliness_event_index = _rclpy.rclpy_wait_set_add_entity('event', wait_set, capsule)
        self.assertIsNotNone(liveliness_event_index)

        try:
            incompatible_qos_event_handle = self._create_event_handle(
                publisher, QoSPublisherEventType.RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS)
            with incompatible_qos_event_handle as capsule:
                incompatible_qos_event_index = _rclpy.rclpy_wait_set_add_entity(
                        'event', wait_set, capsule)
            self.assertIsNotNone(incompatible_qos_event_index)
        except UnsupportedEventTypeError:
            self.assertTrue(self.is_fastrtps)

        # We live in our own namespace and have created no other participants, so
        # there can't be any of these events.
        _rclpy.rclpy_wait(wait_set, 0)
        self.assertFalse(_rclpy.rclpy_wait_set_is_ready('event', wait_set, deadline_event_index))
        self.assertFalse(_rclpy.rclpy_wait_set_is_ready('event', wait_set, liveliness_event_index))
        if not self.is_fastrtps:
            self.assertFalse(_rclpy.rclpy_wait_set_is_ready(
                'event', wait_set, incompatible_qos_event_index))

        # Calling take data even though not ready should provide me an empty initialized message
        # Tests data conversion utilities in C side
        try:
            with deadline_event_handle as event_capsule, publisher.handle as publisher_capsule:
                event_data = _rclpy.rclpy_take_event(
                    event_capsule,
                    publisher_capsule,
                    QoSPublisherEventType.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED)
            self.assertIsInstance(event_data, QoSOfferedDeadlineMissedInfo)
            self.assertEqual(event_data.total_count, 0)
            self.assertEqual(event_data.total_count_change, 0)
        except NotImplementedError:
            pass

        try:
            with liveliness_event_handle as event_capsule, publisher.handle as publisher_capsule:
                event_data = _rclpy.rclpy_take_event(
                    event_capsule,
                    publisher_capsule,
                    QoSPublisherEventType.RCL_PUBLISHER_LIVELINESS_LOST)
            self.assertIsInstance(event_data, QoSLivelinessLostInfo)
            self.assertEqual(event_data.total_count, 0)
            self.assertEqual(event_data.total_count_change, 0)
        except NotImplementedError:
            pass

        if not self.is_fastrtps:
            try:
                with incompatible_qos_event_handle as event_capsule, \
                        publisher.handle as publisher_capsule:
                    event_data = _rclpy.rclpy_take_event(
                        event_capsule,
                        publisher_capsule,
                        QoSPublisherEventType.RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS)
                self.assertIsInstance(event_data, QoSOfferedIncompatibleQoSInfo)
                self.assertEqual(event_data.total_count, 0)
                self.assertEqual(event_data.total_count_change, 0)
                self.assertEqual(event_data.last_policy_kind, QoSPolicyKind.INVALID)
            except NotImplementedError:
                pass

        self.node.destroy_publisher(publisher)

    def test_call_subscription_rclpy_event_apis(self):
        # Go through the exposed apis and ensure that things don't explode when called
        # Make no assumptions about being able to actually receive the events
        subscription = self.node.create_subscription(EmptyMsg, 'test_topic', Mock(), 10)
        wait_set = _rclpy.rclpy_get_zero_initialized_wait_set()
        with self.context.handle as context_handle:
            _rclpy.rclpy_wait_set_init(wait_set, 0, 0, 0, 0, 0, 3, context_handle)

        deadline_event_handle = self._create_event_handle(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED)
        with deadline_event_handle as capsule:
            deadline_event_index = _rclpy.rclpy_wait_set_add_entity('event', wait_set, capsule)
        self.assertIsNotNone(deadline_event_index)

        liveliness_event_handle = self._create_event_handle(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_LIVELINESS_CHANGED)
        with liveliness_event_handle as capsule:
            liveliness_event_index = _rclpy.rclpy_wait_set_add_entity('event', wait_set, capsule)
        self.assertIsNotNone(liveliness_event_index)

        try:
            incompatible_qos_event_handle = self._create_event_handle(
                subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS)
            with incompatible_qos_event_handle as capsule:
                incompatible_qos_event_index = _rclpy.rclpy_wait_set_add_entity(
                        'event', wait_set, capsule)
            self.assertIsNotNone(incompatible_qos_event_index)
        except UnsupportedEventTypeError:
            self.assertTrue(self.is_fastrtps)

        # We live in our own namespace and have created no other participants, so
        # there can't be any of these events.
        _rclpy.rclpy_wait(wait_set, 0)
        self.assertFalse(_rclpy.rclpy_wait_set_is_ready('event', wait_set, deadline_event_index))
        self.assertFalse(_rclpy.rclpy_wait_set_is_ready('event', wait_set, liveliness_event_index))
        if not self.is_fastrtps:
            self.assertFalse(_rclpy.rclpy_wait_set_is_ready(
                'event', wait_set, incompatible_qos_event_index))

        # Calling take data even though not ready should provide me an empty initialized message
        # Tests data conversion utilities in C side
        try:
            with deadline_event_handle as event_capsule, subscription.handle as parent_capsule:
                event_data = _rclpy.rclpy_take_event(
                    event_capsule,
                    parent_capsule,
                    QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED)
            self.assertIsInstance(event_data, QoSRequestedDeadlineMissedInfo)
            self.assertEqual(event_data.total_count, 0)
            self.assertEqual(event_data.total_count_change, 0)
        except NotImplementedError:
            pass

        try:
            with liveliness_event_handle as event_capsule, subscription.handle as parent_capsule:
                event_data = _rclpy.rclpy_take_event(
                    event_capsule,
                    parent_capsule,
                    QoSSubscriptionEventType.RCL_SUBSCRIPTION_LIVELINESS_CHANGED)
            self.assertIsInstance(event_data, QoSLivelinessChangedInfo)
            self.assertEqual(event_data.alive_count, 0)
            self.assertEqual(event_data.alive_count_change, 0)
            self.assertEqual(event_data.not_alive_count, 0)
            self.assertEqual(event_data.not_alive_count_change, 0)
        except NotImplementedError:
            pass

        if not self.is_fastrtps:
            try:
                with incompatible_qos_event_handle as event_capsule, \
                        subscription.handle as parent_capsule:
                    event_data = _rclpy.rclpy_take_event(
                        event_capsule,
                        parent_capsule,
                        QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS)
                self.assertIsInstance(event_data, QoSRequestedIncompatibleQoSInfo)
                self.assertEqual(event_data.total_count, 0)
                self.assertEqual(event_data.total_count_change, 0)
                self.assertEqual(event_data.last_policy_kind, QoSPolicyKind.INVALID)
            except NotImplementedError:
                pass

        self.node.destroy_subscription(subscription)
