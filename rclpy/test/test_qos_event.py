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
from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos_event import QoSLivelinessChangedInfo
from rclpy.qos_event import QoSLivelinessLostInfo
from rclpy.qos_event import QoSOfferedDeadlineMissedInfo
from rclpy.qos_event import QoSPublisherEventType
from rclpy.qos_event import QoSRequestedDeadlineMissedInfo
from rclpy.qos_event import QoSSubscriptionEventType
from rclpy.qos_event import SubscriptionEventCallbacks

from test_msgs.msg import Empty as EmptyMsg


class TestQoSEvent(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('TestQoSEvent', namespace='/rclpy/test', context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_publisher_constructor(self):
        callbacks = PublisherEventCallbacks()
        liveliness_callback = Mock()
        deadline_callback = Mock()

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

        # Arg with both callbacks
        callbacks.liveliness = liveliness_callback
        publisher = self.node.create_publisher(
            EmptyMsg, 'test_topic', 10, event_callbacks=callbacks)
        self.assertEqual(len(publisher.event_handlers), 2)
        self.node.destroy_publisher(publisher)

    def test_subscription_constructor(self):
        callbacks = SubscriptionEventCallbacks()
        liveliness_callback = Mock()
        deadline_callback = Mock()
        message_callback = Mock()

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

        # Arg with both callbacks
        callbacks.liveliness = liveliness_callback
        subscription = self.node.create_subscription(
            EmptyMsg, 'test_topic', message_callback, 10, event_callbacks=callbacks)
        self.assertEqual(len(subscription.event_handlers), 2)
        self.node.destroy_subscription(subscription)

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
        self.node.destroy_publisher(publisher)

    def test_subscription_event_create_destroy(self):
        message_callback = Mock()
        subscription = self.node.create_subscription(EmptyMsg, 'test_topic', message_callback, 10)
        self._do_create_destroy(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_LIVELINESS_CHANGED)
        self._do_create_destroy(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED)
        self.node.destroy_subscription(subscription)

    def test_call_publisher_rclpy_event_apis(self):
        # Go through the exposed apis and ensure that things don't explode when called
        # Make no assumptions about being able to actually receive the events
        publisher = self.node.create_publisher(EmptyMsg, 'test_topic', 10)
        wait_set = Handle(_rclpy.rclpy_get_zero_initialized_wait_set())
        wait_set.requires(self.context.handle)
        with self.context.handle as context_capsule, wait_set as wait_set_capsule:
            _rclpy.rclpy_wait_set_init(wait_set_capsule, 0, 0, 0, 0, 0, 2, context_capsule)

        deadline_event_handle = self._create_event_handle(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED)
        wait_set.requires(deadline_event_handle)
        with deadline_event_handle as event_capsule, wait_set as wait_set_capsule:
            deadline_event_index = _rclpy.rclpy_wait_set_add_entity(
                'event', wait_set_capsule, event_capsule)
        self.assertIsNotNone(deadline_event_index)

        liveliness_event_handle = self._create_event_handle(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_LIVELINESS_LOST)
        wait_set.requires(liveliness_event_handle)
        with liveliness_event_handle as event_capsule, wait_set as wait_set_capsule:
            liveliness_event_index = _rclpy.rclpy_wait_set_add_entity(
                'event', wait_set_capsule, event_capsule)
        self.assertIsNotNone(liveliness_event_index)

        # We live in our own namespace and have created no other participants, so
        # there can't be any of these events.
        with wait_set as wait_set_capsule:
            _rclpy.rclpy_wait(wait_set_capsule, 0)
            self.assertFalse(
                _rclpy.rclpy_wait_set_is_ready('event', wait_set_capsule, deadline_event_index))
            self.assertFalse(
                _rclpy.rclpy_wait_set_is_ready('event', wait_set_capsule, liveliness_event_index))

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

        self.node.destroy_publisher(publisher)

    def test_call_subscription_rclpy_event_apis(self):
        # Go through the exposed apis and ensure that things don't explode when called
        # Make no assumptions about being able to actually receive the events
        subscription = self.node.create_subscription(EmptyMsg, 'test_topic', Mock(), 10)
        wait_set = Handle(_rclpy.rclpy_get_zero_initialized_wait_set())
        wait_set.requires(self.context.handle)
        with self.context.handle as context_capsule, wait_set as wait_set_capsule:
            _rclpy.rclpy_wait_set_init(wait_set_capsule, 0, 0, 0, 0, 0, 2, context_capsule)

        deadline_event_handle = self._create_event_handle(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED)
        wait_set.requires(deadline_event_handle)
        with deadline_event_handle as event_capsule, wait_set as wait_set_capsule:
            deadline_event_index = _rclpy.rclpy_wait_set_add_entity(
                'event', wait_set_capsule, event_capsule)
        self.assertIsNotNone(deadline_event_index)

        liveliness_event_handle = self._create_event_handle(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_LIVELINESS_CHANGED)
        wait_set.requires(liveliness_event_handle)
        with liveliness_event_handle as event_capsule, wait_set as wait_set_capsule:
            liveliness_event_index = _rclpy.rclpy_wait_set_add_entity(
                'event', wait_set_capsule, event_capsule)
        self.assertIsNotNone(liveliness_event_index)

        # We live in our own namespace and have created no other participants, so
        # there can't be any of these events.
        with wait_set as wait_set_capsule:
            _rclpy.rclpy_wait(wait_set_capsule, 0)
            self.assertFalse(_rclpy.rclpy_wait_set_is_ready(
                'event', wait_set_capsule, deadline_event_index))
            self.assertFalse(_rclpy.rclpy_wait_set_is_ready(
                'event', wait_set_capsule, liveliness_event_index))

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

        self.node.destroy_subscription(subscription)
