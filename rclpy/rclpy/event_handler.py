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

from enum import IntEnum
from typing import Callable
from typing import List
from typing import Optional
import warnings

import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.logging import get_logger
from rclpy.qos import qos_policy_name_from_kind
from rclpy.waitable import NumberOfEntities
from rclpy.waitable import Waitable


QoSPublisherEventType = _rclpy.rcl_publisher_event_type_t
QoSSubscriptionEventType = _rclpy.rcl_subscription_event_type_t


# Payload type for Subscription Deadline callback.
QoSRequestedDeadlineMissedInfo = _rclpy.rmw_requested_deadline_missed_status_t

# Payload type for Subscription Liveliness callback.
QoSLivelinessChangedInfo = _rclpy.rmw_liveliness_changed_status_t

# Payload type for Subscription Message Lost callback.
QoSMessageLostInfo = _rclpy.rmw_message_lost_status_t

# Payload type for Subscription Incompatible QoS callback.
QoSRequestedIncompatibleQoSInfo = _rclpy.rmw_requested_qos_incompatible_event_status_t

# Payload type for Subscription matched callback.
QoSSubscriptionMatchedInfo = _rclpy.rmw_matched_status_t

# Payload type for Publisher Deadline callback.
QoSOfferedDeadlineMissedInfo = _rclpy.rmw_offered_deadline_missed_status_t

# Payload type for Publisher Liveliness callback.
QoSLivelinessLostInfo = _rclpy.rmw_liveliness_lost_status_t

# Payload type for Publisher matched callback.
QoSPublisherMatchedInfo = _rclpy.rmw_matched_status_t

"""
Payload type for Publisher Incompatible QoS callback.

Mirrors rmw_offered_incompatible_qos_status_t from rmw/types.h
"""
QoSOfferedIncompatibleQoSInfo = QoSRequestedIncompatibleQoSInfo

# Payload type for Incompatible Type callback.
IncompatibleTypeInfo = _rclpy.rmw_incompatible_type_status_t


"""Raised when registering a callback for an event type that is not supported."""
UnsupportedEventTypeError = _rclpy.UnsupportedEventTypeError


class EventHandler(Waitable):
    """Waitable type to handle QoS events."""

    def __init__(
        self,
        *,
        callback_group: CallbackGroup,
        callback: Callable,
        event_type: IntEnum,
        parent_impl,
    ):
        # Waitable init adds self to callback_group
        super().__init__(callback_group)
        self.event_type = event_type
        self.callback = callback

        with parent_impl:
            self.__event = _rclpy.EventHandle(parent_impl, event_type)

        self._ready_to_take_data = False
        self._event_index = None

    # Start Waitable API
    def is_ready(self, wait_set):
        """Return True if entities are ready in the wait set."""
        if self._event_index is None:
            return False
        if wait_set.is_ready('event', self._event_index):
            self._ready_to_take_data = True
        return self._ready_to_take_data

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        if self._ready_to_take_data:
            self._ready_to_take_data = False
            with self.__event:
                return self.__event.take_event()
        return None

    async def execute(self, taken_data):
        """Execute work after data has been taken from a ready wait set."""
        if not taken_data:
            return
        await rclpy.executors.await_or_execute(self.callback, taken_data)

    def get_num_entities(self):
        """Return number of each type of entity used."""
        return NumberOfEntities(num_events=1)

    def add_to_wait_set(self, wait_set):
        """Add entites to wait set."""
        with self.__event:
            self._event_index = wait_set.add_event(self.__event)

    def __enter__(self):
        """Mark event as in-use to prevent destruction while waiting on it."""
        self.__event.__enter__()

    def __exit__(self, t, v, tb):
        """Mark event as not-in-use to allow destruction after waiting on it."""
        self.__event.__exit__(t, v, tb)

    def destroy(self):
        self.__event.destroy_when_not_in_use()


class QoSEventHandler(EventHandler):

    def __init_subclass__(cls, **kwargs):
        warnings.warn('QoSEventHandler foo is deprecated, use EventHandler instead.',
                      DeprecationWarning, stacklevel=2)
        super().__init_subclass__(**kwargs)

    def __init__(self, *args, **kwargs):
        warnings.warn('QoSEventHandler is deprecated, use EventHandler instead.',
                      DeprecationWarning, stacklevel=2)
        super().__init__(*args, **kwargs)


class SubscriptionEventCallbacks:
    """Container to provide middleware event callbacks for a Subscription."""

    def __init__(
        self,
        *,
        deadline: Optional[Callable[[QoSRequestedDeadlineMissedInfo], None]] = None,
        incompatible_qos: Optional[Callable[[QoSRequestedIncompatibleQoSInfo], None]] = None,
        liveliness: Optional[Callable[[QoSLivelinessChangedInfo], None]] = None,
        message_lost: Optional[Callable[[QoSMessageLostInfo], None]] = None,
        incompatible_type: Optional[Callable[[IncompatibleTypeInfo], None]] = None,
        matched: Optional[Callable[[QoSSubscriptionMatchedInfo], None]] = None,
        use_default_callbacks: bool = True,
    ) -> None:
        """
        Create a SubscriptionEventCallbacks container.

        :param deadline: A user-defined callback that is called when a topic misses our
            requested Deadline.
        :param incompatible_qos: A user-defined callback that is called when a Publisher
            with incompatible QoS policies is discovered on subscribed topic.
        :param liveliness: A user-defined callback that is called when the Liveliness of
            a Publisher on subscribed topic changes.
        :param message_lost: A user-defined callback that is called when a messages is lost.
        :param incompatible_type: A user-defined callback that is called when a topic type
            doesn't match.
        :param matched: A user-defined callback that is called when a Publisher is connected or
             disconnected.
        :param use_default_callbacks: Whether or not to use default callbacks when the user
            doesn't supply one
        """
        self.deadline = deadline
        self.incompatible_qos = incompatible_qos
        self.liveliness = liveliness
        self.message_lost = message_lost
        self.incompatible_type = incompatible_type
        self.matched = matched
        self.use_default_callbacks = use_default_callbacks

    def create_event_handlers(
        self, callback_group: CallbackGroup, subscription: _rclpy.Subscription, topic_name: str,
    ) -> List[EventHandler]:
        with subscription:
            logger = get_logger(subscription.get_logger_name())

        event_handlers = []
        if self.deadline:
            event_handlers.append(EventHandler(
                callback_group=callback_group,
                callback=self.deadline,
                event_type=QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED,
                parent_impl=subscription))

        incompatible_qos_callback = None
        if self.incompatible_qos:
            incompatible_qos_callback = self.incompatible_qos
        elif self.use_default_callbacks:
            # Register default callback when not specified
            def _default_incompatible_qos_callback(event):
                policy_name = qos_policy_name_from_kind(event.last_policy_kind)
                logger.warn(
                    "New publisher discovered on topic '{}', offering incompatible QoS. "
                    'No messages will be received from it. '
                    'Last incompatible policy: {}'.format(topic_name, policy_name))
            incompatible_qos_callback = _default_incompatible_qos_callback
        try:
            if incompatible_qos_callback is not None:
                event_type = QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS
                event_handlers.append(EventHandler(
                    callback_group=callback_group,
                    callback=incompatible_qos_callback,
                    event_type=event_type,
                    parent_impl=subscription))
        except UnsupportedEventTypeError:
            pass

        if self.liveliness:
            event_handlers.append(EventHandler(
                callback_group=callback_group,
                callback=self.liveliness,
                event_type=QoSSubscriptionEventType.RCL_SUBSCRIPTION_LIVELINESS_CHANGED,
                parent_impl=subscription))

        if self.message_lost:
            event_handlers.append(EventHandler(
                callback_group=callback_group,
                callback=self.message_lost,
                event_type=QoSSubscriptionEventType.RCL_SUBSCRIPTION_MESSAGE_LOST,
                parent_impl=subscription))

        incompatible_type_callback = None
        if self.incompatible_type:
            incompatible_type_callback = self.incompatible_type
        elif self.use_default_callbacks:
            # Register default callback when not specified
            def _default_incompatible_type_callback(event):
                logger.warn(
                    "Incompatible type on topic '{}', no messages will be sent to it."
                    .format(topic_name))
            incompatible_type_callback = _default_incompatible_type_callback
        try:
            if incompatible_type_callback is not None:
                event_handlers.append(EventHandler(
                    callback_group=callback_group,
                    callback=incompatible_type_callback,
                    event_type=QoSSubscriptionEventType.RCL_SUBSCRIPTION_INCOMPATIBLE_TYPE,
                    parent_impl=subscription))

        except UnsupportedEventTypeError:
            pass

        if self.matched:
            event_handlers.append(QoSEventHandler(
                callback_group=callback_group,
                callback=self.matched,
                event_type=QoSSubscriptionEventType.RCL_SUBSCRIPTION_MATCHED,
                parent_impl=subscription))

        return event_handlers


class PublisherEventCallbacks:
    """Container to provide middleware event callbacks for a Publisher."""

    def __init__(
        self,
        *,
        deadline: Optional[Callable[[QoSOfferedDeadlineMissedInfo], None]] = None,
        liveliness: Optional[Callable[[QoSLivelinessLostInfo], None]] = None,
        incompatible_qos: Optional[Callable[[QoSRequestedIncompatibleQoSInfo], None]] = None,
        incompatible_type: Optional[Callable[[IncompatibleTypeInfo], None]] = None,
        matched: Optional[Callable[[QoSPublisherMatchedInfo], None]] = None,
        use_default_callbacks: bool = True,
    ) -> None:
        """
        Create and return a PublisherEventCallbacks container.

        :param deadline: A user-defined callback that is called when the Publisher misses
            its offered Deadline.
        :param liveliness: A user-defined callback that is called when this Publisher
            fails to signal its Liveliness and is reported as not-alive.
        :param incompatible_qos: A user-defined callback that is called when a Subscription
            with incompatible QoS policies is discovered on subscribed topic.
        :param incompatible_type: A user-defined callback that is called when a topic type
            doesn't match.
        :param matched: A user-defined callback that is called when a Subscription is connected or
            disconnected.
        :param use_default_callbacks: Whether or not to use default callbacks when the user
            doesn't supply one
        """
        self.deadline = deadline
        self.liveliness = liveliness
        self.incompatible_qos = incompatible_qos
        self.incompatible_type = incompatible_type
        self.matched = matched
        self.use_default_callbacks = use_default_callbacks

    def create_event_handlers(
        self, callback_group: CallbackGroup, publisher: _rclpy.Publisher, topic_name: str,
    ) -> List[EventHandler]:
        with publisher:
            logger = get_logger(publisher.get_logger_name())

        event_handlers = []
        if self.deadline:
            event_handlers.append(EventHandler(
                callback_group=callback_group,
                callback=self.deadline,
                event_type=QoSPublisherEventType.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED,
                parent_impl=publisher))

        if self.liveliness:
            event_handlers.append(EventHandler(
                callback_group=callback_group,
                callback=self.liveliness,
                event_type=QoSPublisherEventType.RCL_PUBLISHER_LIVELINESS_LOST,
                parent_impl=publisher))

        incompatible_qos_callback = None
        if self.incompatible_qos:
            incompatible_qos_callback = self.incompatible_qos
        elif self.use_default_callbacks:
            # Register default callback when not specified
            def _default_incompatible_qos_callback(event):
                policy_name = qos_policy_name_from_kind(event.last_policy_kind)
                logger.warn(
                    "New subscription discovered on topic '{}', requesting incompatible QoS. "
                    'No messages will be sent to it. '
                    'Last incompatible policy: {}'.format(topic_name, policy_name))
            incompatible_qos_callback = _default_incompatible_qos_callback
        try:
            if incompatible_qos_callback is not None:
                event_handlers.append(EventHandler(
                    callback_group=callback_group,
                    callback=incompatible_qos_callback,
                    event_type=QoSPublisherEventType.RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS,
                    parent_impl=publisher))

        except UnsupportedEventTypeError:
            pass

        incompatible_type_callback = None
        if self.incompatible_type:
            incompatible_type_callback = self.incompatible_type
        elif self.use_default_callbacks:
            # Register default callback when not specified
            def _default_incompatible_type_callback(event):
                logger.warn(
                    "Incompatible type on topic '{}', no messages will be sent to it."
                    .format(topic_name))
            incompatible_type_callback = _default_incompatible_type_callback
        try:
            if incompatible_type_callback is not None:
                event_handlers.append(EventHandler(
                    callback_group=callback_group,
                    callback=incompatible_type_callback,
                    event_type=QoSPublisherEventType.RCL_PUBLISHER_INCOMPATIBLE_TYPE,
                    parent_impl=publisher))

        except UnsupportedEventTypeError:
            pass

        if self.matched:
            event_handlers.append(QoSEventHandler(
                callback_group=callback_group,
                callback=self.matched,
                event_type=QoSPublisherEventType.RCL_PUBLISHER_MATCHED,
                parent_impl=publisher))

        return event_handlers
