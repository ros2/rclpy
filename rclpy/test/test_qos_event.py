# Copyright 2019 Open Source Robotics Foundation, Inc.
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
        publisher = self.node.create_publisher(EmptyMsg, 'test_topic')
        self.assertEqual(len(publisher.event_handlers), 0)
        self.node.destroy_publisher(publisher)

        # Arg with no callbacks
        publisher = self.node.create_publisher(EmptyMsg, 'test_topic', event_callbacks=callbacks)
        self.assertEqual(len(publisher.event_handlers), 0)
        self.node.destroy_publisher(publisher)

        # Arg with one of the callbacks
        callbacks.deadline = deadline_callback
        publisher = self.node.create_publisher(EmptyMsg, 'test_topic', event_callbacks=callbacks)
        self.assertEqual(len(publisher.event_handlers), 1)
        self.node.destroy_publisher(publisher)

        # Arg with both callbacks
        callbacks.liveliness = liveliness_callback
        publisher = self.node.create_publisher(EmptyMsg, 'test_topic', event_callbacks=callbacks)
        self.assertEqual(len(publisher.event_handlers), 2)
        self.node.destroy_publisher(publisher)

    def test_subscription_constructor(self):
        callbacks = SubscriptionEventCallbacks()
        liveliness_callback = Mock()
        deadline_callback = Mock()
        message_callback = Mock()

        # No arg
        subscription = self.node.create_subscription(EmptyMsg, 'test_topic', message_callback)
        self.assertEqual(len(subscription.event_handlers), 0)
        self.node.destroy_subscription(subscription)

        # Arg with no callbacks
        subscription = self.node.create_subscription(
            EmptyMsg, 'test_topic', message_callback, event_callbacks=callbacks)
        self.assertEqual(len(subscription.event_handlers), 0)
        self.node.destroy_subscription(subscription)

        # Arg with one of the callbacks
        callbacks.deadline = deadline_callback
        subscription = self.node.create_subscription(
            EmptyMsg, 'test_topic', message_callback, event_callbacks=callbacks)
        self.assertEqual(len(subscription.event_handlers), 1)
        self.node.destroy_subscription(subscription)

        # Arg with both callbacks
        callbacks.liveliness = liveliness_callback
        subscription = self.node.create_subscription(
            EmptyMsg, 'test_topic', message_callback, event_callbacks=callbacks)
        self.assertEqual(len(subscription.event_handlers), 2)
        self.node.destroy_subscription(subscription)

    def test_publisher_event_create_destroy(self):
        # Publisher event types
        publisher = self.node.create_publisher(EmptyMsg, 'test_topic')

        event_handle = _rclpy.rclpy_create_event(
            QoSPublisherEventType.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED,
            publisher.publisher_handle)
        self.assertIsNotNone(event_handle)
        _rclpy.rclpy_destroy_entity(event_handle)

        event_handle = _rclpy.rclpy_create_event(
            QoSPublisherEventType.RCL_PUBLISHER_LIVELINESS_LOST,
            publisher.publisher_handle)
        self.assertIsNotNone(event_handle)
        _rclpy.rclpy_destroy_entity(event_handle)

        self.node.destroy_publisher(publisher)

    def test_subscription_event_create_destroy(self):
        # Subscription event types
        message_callback = Mock()
        subscription = self.node.create_subscription(EmptyMsg, 'test_topic', message_callback)

        with subscription.handle as subscription_capsule:
            # self.assertFalse(subscription_capsule)
            event_handle = _rclpy.rclpy_create_event(
                QoSSubscriptionEventType.RCL_SUBSCRIPTION_LIVELINESS_CHANGED,
                subscription_capsule)
            self.assertIsNotNone(event_handle)
            _rclpy.rclpy_destroy_entity(event_handle)

            event_handle = _rclpy.rclpy_create_event(
                QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED,
                subscription_capsule)
            self.assertIsNotNone(event_handle)
            _rclpy.rclpy_destroy_entity(event_handle)

        self.node.destroy_subscription(subscription)

    def test_call_publisher_rclpy_event_apis(self):
        # Go through the exposed apis and ensure that things don't explode when called
        # Make no assumptions about being able to actually receive the events
        publisher = self.node.create_publisher(EmptyMsg, 'test_topic')
        wait_set = _rclpy.rclpy_get_zero_initialized_wait_set()
        _rclpy.rclpy_wait_set_init(wait_set, 0, 0, 0, 0, 0, 2, self.context.handle)

        deadline_event_handle = _rclpy.rclpy_create_event(
            QoSPublisherEventType.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED,
            publisher.publisher_handle)
        deadline_event_index = _rclpy.rclpy_wait_set_add_entity(
            'event', wait_set, deadline_event_handle)
        self.assertIsNotNone(deadline_event_index)

        liveliness_event_handle = _rclpy.rclpy_create_event(
            QoSPublisherEventType.RCL_PUBLISHER_LIVELINESS_LOST,
            publisher.publisher_handle)
        liveliness_event_index = _rclpy.rclpy_wait_set_add_entity(
            'event', wait_set, liveliness_event_handle)
        self.assertIsNotNone(liveliness_event_index)

        # We live in our own namespace and have created no other participants, so
        # there can't be any of these events.
        _rclpy.rclpy_wait(wait_set, 0)
        self.assertFalse(_rclpy.rclpy_wait_set_is_ready('event', wait_set, deadline_event_index))
        self.assertFalse(_rclpy.rclpy_wait_set_is_ready('event', wait_set, liveliness_event_index))

        # Calling take data even though not ready should provide me an empty initialized message
        # Tests data conversion utilities in C side
        event_data = _rclpy.rclpy_take_event(
            deadline_event_handle,
            publisher.publisher_handle,
            QoSPublisherEventType.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED)
        self.assertIsInstance(event_data, QoSOfferedDeadlineMissedInfo)
        self.assertEqual(event_data.total_count, 0)
        self.assertEqual(event_data.total_count_change, 0)

        event_data = _rclpy.rclpy_take_event(
            liveliness_event_handle,
            publisher.publisher_handle,
            QoSPublisherEventType.RCL_PUBLISHER_LIVELINESS_LOST)
        self.assertIsInstance(event_data, QoSLivelinessLostInfo)
        self.assertEqual(event_data.total_count, 0)
        self.assertEqual(event_data.total_count_change, 0)

        self.node.destroy_publisher(publisher)

    def test_call_subscription_rclpy_event_apis(self):
        # Go through the exposed apis and ensure that things don't explode when called
        # Make no assumptions about being able to actually receive the events
        subscription = self.node.create_subscription(EmptyMsg, 'test_topic', Mock())
        wait_set = _rclpy.rclpy_get_zero_initialized_wait_set()
        _rclpy.rclpy_wait_set_init(wait_set, 0, 0, 0, 0, 0, 2, self.context.handle)

        subscription_capsule = subscription.handle._get_capsule()
        deadline_event_handle = _rclpy.rclpy_create_event(
            QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED,
            subscription_capsule)
        deadline_event_index = _rclpy.rclpy_wait_set_add_entity(
            'event', wait_set, deadline_event_handle)
        self.assertIsNotNone(deadline_event_index)

        liveliness_event_handle = _rclpy.rclpy_create_event(
            QoSSubscriptionEventType.RCL_SUBSCRIPTION_LIVELINESS_CHANGED,
            subscription_capsule)
        liveliness_event_index = _rclpy.rclpy_wait_set_add_entity(
            'event', wait_set, liveliness_event_handle)
        self.assertIsNotNone(liveliness_event_index)

        # We live in our own namespace and have created no other participants, so
        # there can't be any of these events.
        _rclpy.rclpy_wait(wait_set, 0)
        self.assertFalse(_rclpy.rclpy_wait_set_is_ready('event', wait_set, deadline_event_index))
        self.assertFalse(_rclpy.rclpy_wait_set_is_ready('event', wait_set, liveliness_event_index))

        # Calling take data even though not ready should provide me an empty initialized message
        # Tests data conversion utilities in C side
        event_data = _rclpy.rclpy_take_event(
            deadline_event_handle,
            subscription_capsule,
            QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED)
        self.assertIsInstance(event_data, QoSRequestedDeadlineMissedInfo)
        self.assertEqual(event_data.total_count, 0)
        self.assertEqual(event_data.total_count_change, 0)

        event_data = _rclpy.rclpy_take_event(
            liveliness_event_handle,
            subscription_capsule,
            QoSSubscriptionEventType.RCL_SUBSCRIPTION_LIVELINESS_CHANGED)
        self.assertIsInstance(event_data, QoSLivelinessChangedInfo)
        self.assertEqual(event_data.alive_count, 0)
        self.assertEqual(event_data.alive_count_change, 0)
        self.assertEqual(event_data.not_alive_count, 0)
        self.assertEqual(event_data.not_alive_count_change, 0)

        subscription.handle._return_capsule()
        self.node.destroy_subscription(subscription)
