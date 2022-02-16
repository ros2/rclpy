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

import gc
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSPolicyKind
from rclpy.qos import QoSProfile
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
from rclpy.task import Future

from test_msgs.msg import Empty as EmptyMsg


class TestQoSEvent(unittest.TestCase):
    topic_name = 'test_topic'

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('TestQoSEvent',
                                      namespace='/rclpy/test',
                                      context=self.context)

    def tearDown(self):
        # These tests create a bunch of events by hand instead of using Node APIs,
        # so they won't be cleaned up when calling `node.destroy_node()`, but they could still
        # keep the node alive from one test to the next.
        # Invoke the garbage collector to destroy them.
        gc.collect()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_publisher_constructor(self):
        callbacks = PublisherEventCallbacks()
        liveliness_callback = Mock()
        deadline_callback = Mock()
        incompatible_qos_callback = Mock()
        expected_num_event_handlers = 1

        # No arg
        publisher = self.node.create_publisher(EmptyMsg, self.topic_name, 10)
        self.assertEqual(len(publisher.event_handlers), expected_num_event_handlers)
        self.node.destroy_publisher(publisher)

        # Arg with no callbacks
        publisher = self.node.create_publisher(
            EmptyMsg, self.topic_name, 10, event_callbacks=callbacks)
        self.assertEqual(len(publisher.event_handlers), expected_num_event_handlers)
        self.node.destroy_publisher(publisher)

        # Arg with one of the callbacks
        callbacks.deadline = deadline_callback
        expected_num_event_handlers += 1
        publisher = self.node.create_publisher(
            EmptyMsg, self.topic_name, 10, event_callbacks=callbacks)
        self.assertEqual(len(publisher.event_handlers), expected_num_event_handlers)
        self.node.destroy_publisher(publisher)

        # Arg with two callbacks
        callbacks.liveliness = liveliness_callback
        expected_num_event_handlers += 1
        publisher = self.node.create_publisher(
            EmptyMsg, self.topic_name, 10, event_callbacks=callbacks)
        self.assertEqual(len(publisher.event_handlers), expected_num_event_handlers)
        self.node.destroy_publisher(publisher)

        # Arg with three callbacks
        callbacks.incompatible_qos = incompatible_qos_callback
        publisher = self.node.create_publisher(
            EmptyMsg, self.topic_name, 10, event_callbacks=callbacks)
        self.assertEqual(len(publisher.event_handlers), 3)
        self.node.destroy_publisher(publisher)

    def test_subscription_constructor(self):
        callbacks = SubscriptionEventCallbacks()
        liveliness_callback = Mock()
        deadline_callback = Mock()
        message_callback = Mock()
        incompatible_qos_callback = Mock()
        expected_num_event_handlers = 1

        # No arg
        subscription = self.node.create_subscription(
            EmptyMsg, self.topic_name, message_callback, 10)
        self.assertEqual(len(subscription.event_handlers), expected_num_event_handlers)
        self.node.destroy_subscription(subscription)

        # Arg with no callbacks
        subscription = self.node.create_subscription(
            EmptyMsg, self.topic_name, message_callback, 10, event_callbacks=callbacks)
        self.assertEqual(len(subscription.event_handlers), expected_num_event_handlers)
        self.node.destroy_subscription(subscription)

        # Arg with one of the callbacks
        callbacks.deadline = deadline_callback
        expected_num_event_handlers += 1
        subscription = self.node.create_subscription(
            EmptyMsg, self.topic_name, message_callback, 10, event_callbacks=callbacks)
        self.assertEqual(len(subscription.event_handlers), expected_num_event_handlers)
        self.node.destroy_subscription(subscription)

        # Arg with two callbacks
        callbacks.liveliness = liveliness_callback
        expected_num_event_handlers += 1
        subscription = self.node.create_subscription(
            EmptyMsg, self.topic_name, message_callback, 10, event_callbacks=callbacks)
        self.assertEqual(len(subscription.event_handlers), expected_num_event_handlers)
        self.node.destroy_subscription(subscription)

        # Arg with three callbacks
        callbacks.incompatible_qos = incompatible_qos_callback
        subscription = self.node.create_subscription(
            EmptyMsg, self.topic_name, message_callback, 10, event_callbacks=callbacks)
        self.assertEqual(len(subscription.event_handlers), 3)
        self.node.destroy_subscription(subscription)

    def test_default_incompatible_qos_callbacks(self):
        original_logger = rclpy.logging._root_logger

        pub_log_msg = None
        sub_log_msg = None
        log_msgs_future = Future()

        class MockLogger:

            def get_child(self, name):
                return self

            def warn(self, message, once=False):
                nonlocal pub_log_msg, sub_log_msg, log_msgs_future

                if message.startswith('New subscription discovered'):
                    pub_log_msg = message
                elif message.startswith('New publisher discovered'):
                    sub_log_msg = message

                if pub_log_msg is not None and sub_log_msg is not None:
                    log_msgs_future.set_result(True)

        rclpy.logging._root_logger = MockLogger()

        qos_profile_publisher = QoSProfile(
            depth=10, durability=QoSDurabilityPolicy.VOLATILE)
        self.node.create_publisher(EmptyMsg, self.topic_name, qos_profile_publisher)

        message_callback = Mock()
        qos_profile_subscription = QoSProfile(
            depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            EmptyMsg, self.topic_name, message_callback, qos_profile_subscription)

        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        rclpy.spin_until_future_complete(self.node, log_msgs_future, executor, 10.0)

        self.assertEqual(
            pub_log_msg,
            "New subscription discovered on topic '{}', requesting incompatible QoS. "
            'No messages will be sent to it. '
            'Last incompatible policy: DURABILITY'.format(self.topic_name))
        self.assertEqual(
            sub_log_msg,
            "New publisher discovered on topic '{}', offering incompatible QoS. "
            'No messages will be received from it. '
            'Last incompatible policy: DURABILITY'.format(self.topic_name))

        rclpy.logging._root_logger = original_logger

    def _create_event_handle(self, parent_entity, event_type):
        with parent_entity.handle:
            event = _rclpy.QoSEvent(parent_entity.handle, event_type)
        self.assertIsNotNone(event)
        return event

    def _do_create_destroy(self, parent_entity, event_type):
        handle = self._create_event_handle(parent_entity, event_type)
        handle.destroy_when_not_in_use()

    def test_publisher_event_create_destroy(self):
        publisher = self.node.create_publisher(EmptyMsg, self.topic_name, 10)
        self._do_create_destroy(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED)
        self._do_create_destroy(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_LIVELINESS_LOST)
        self._do_create_destroy(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS)
        self.node.destroy_publisher(publisher)

    def test_subscription_event_create_destroy(self):
        message_callback = Mock()
        subscription = self.node.create_subscription(
            EmptyMsg, self.topic_name, message_callback, 10)
        self._do_create_destroy(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_LIVELINESS_CHANGED)
        self._do_create_destroy(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED)
        self._do_create_destroy(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS)
        self.node.destroy_subscription(subscription)

    def test_call_publisher_rclpy_event_apis(self):
        # Go through the exposed apis and ensure that things don't explode when called
        # Make no assumptions about being able to actually receive the events
        publisher = self.node.create_publisher(EmptyMsg, self.topic_name, 10)
        with self.context.handle:
            wait_set = _rclpy.WaitSet(0, 0, 0, 0, 0, 3, self.context.handle)

        deadline_event_handle = self._create_event_handle(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED)
        with deadline_event_handle:
            deadline_event_index = wait_set.add_event(deadline_event_handle)
        self.assertIsNotNone(deadline_event_index)

        liveliness_event_handle = self._create_event_handle(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_LIVELINESS_LOST)
        with liveliness_event_handle:
            liveliness_event_index = wait_set.add_event(
                    liveliness_event_handle)
        self.assertIsNotNone(liveliness_event_index)

        incompatible_qos_event_handle = self._create_event_handle(
            publisher, QoSPublisherEventType.RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS)
        with incompatible_qos_event_handle:
            incompatible_qos_event_index = wait_set.add_event(
                    incompatible_qos_event_handle)
        self.assertIsNotNone(incompatible_qos_event_index)

        # We live in our own namespace and have created no other participants, so
        # there can't be any of these events.
        wait_set.wait(0)
        self.assertFalse(wait_set.is_ready('event', deadline_event_index))
        self.assertFalse(wait_set.is_ready('event', liveliness_event_index))
        self.assertFalse(wait_set.is_ready('event', incompatible_qos_event_index))

        # Calling take data even though not ready should provide me an empty initialized message
        # Tests data conversion utilities in C side
        try:
            with deadline_event_handle:
                event_data = deadline_event_handle.take_event()
            self.assertIsInstance(event_data, QoSOfferedDeadlineMissedInfo)
            self.assertEqual(event_data.total_count, 0)
            self.assertEqual(event_data.total_count_change, 0)
        except NotImplementedError:
            pass

        try:
            with liveliness_event_handle:
                event_data = liveliness_event_handle.take_event()
            self.assertIsInstance(event_data, QoSLivelinessLostInfo)
            self.assertEqual(event_data.total_count, 0)
            self.assertEqual(event_data.total_count_change, 0)
        except NotImplementedError:
            pass

        try:
            with incompatible_qos_event_handle:
                event_data = incompatible_qos_event_handle.take_event()
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
        subscription = self.node.create_subscription(EmptyMsg, self.topic_name, Mock(), 10)
        with self.context.handle:
            wait_set = _rclpy.WaitSet(0, 0, 0, 0, 0, 3, self.context.handle)

        deadline_event_handle = self._create_event_handle(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED)
        with deadline_event_handle:
            deadline_event_index = wait_set.add_event(deadline_event_handle)
        self.assertIsNotNone(deadline_event_index)

        liveliness_event_handle = self._create_event_handle(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_LIVELINESS_CHANGED)
        with liveliness_event_handle:
            liveliness_event_index = wait_set.add_event(liveliness_event_handle)
        self.assertIsNotNone(liveliness_event_index)

        incompatible_qos_event_handle = self._create_event_handle(
            subscription, QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS)
        with incompatible_qos_event_handle:
            incompatible_qos_event_index = wait_set.add_event(
                    incompatible_qos_event_handle)
        self.assertIsNotNone(incompatible_qos_event_index)

        # We live in our own namespace and have created no other participants, so
        # there can't be any of these events.
        wait_set.wait(0)
        self.assertFalse(wait_set.is_ready('event', deadline_event_index))
        self.assertFalse(wait_set.is_ready('event', liveliness_event_index))
        self.assertFalse(wait_set.is_ready('event', incompatible_qos_event_index))

        # Calling take data even though not ready should provide me an empty initialized message
        # Tests data conversion utilities in C side
        try:
            with deadline_event_handle:
                event_data = deadline_event_handle.take_event()
            self.assertIsInstance(event_data, QoSRequestedDeadlineMissedInfo)
            self.assertEqual(event_data.total_count, 0)
            self.assertEqual(event_data.total_count_change, 0)
        except NotImplementedError:
            pass

        try:
            with liveliness_event_handle:
                event_data = liveliness_event_handle.take_event()
            self.assertIsInstance(event_data, QoSLivelinessChangedInfo)
            self.assertEqual(event_data.alive_count, 0)
            self.assertEqual(event_data.alive_count_change, 0)
            self.assertEqual(event_data.not_alive_count, 0)
            self.assertEqual(event_data.not_alive_count_change, 0)
        except NotImplementedError:
            pass

        try:
            with incompatible_qos_event_handle:
                event_data = incompatible_qos_event_handle.take_event()
            self.assertIsInstance(event_data, QoSRequestedIncompatibleQoSInfo)
            self.assertEqual(event_data.total_count, 0)
            self.assertEqual(event_data.total_count_change, 0)
            self.assertEqual(event_data.last_policy_kind, QoSPolicyKind.INVALID)
        except NotImplementedError:
            pass

        self.node.destroy_subscription(subscription)
