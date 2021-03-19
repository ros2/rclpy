// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLPY__QOS_EVENTS_HPP_
#define RCLPY__QOS_EVENTS_HPP_

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rclpy
{

/// Create an event for QoS event handling.
/**
 * This function will create an event handle for the given subscription or publisher parent.
 *
 * Raises UnsupportedEventTypeError if the event type is not supported.
 * Raises TypeError if arguments are not of the correct types i.e. a subscription capsule,
 *   as returned by rclpy_subscription_create(), for a \p pyparent and an
 *   rclpy.qos_events.QoSSubscriptionEventType enum value as \p pyevent_type or
 *   a publisher capsule, as returned by rclpy_publisher_create(), for a \p pyparent and
 *   an rclpy.qos_events.QoSPublisherEventType enum value as \p pyevent_type.
 * Raises MemoryError if the event can't be allocated.
 * Raises RCLError if event initialization failed in rcl.
 *
 * \param[in] pyevent_type Type of event to create of rclpy.qos_events.QoSSubscriptionEventType
 *   or rclpy.qos_events.QoSPublisherEventType type matching the given \p pyparent.
 * \param[in] pyparent Capsule containing the parent publisher or subscription.
 * \return Capsule containing an rcl_event_t.
 */
py::object
create_event(py::object pyevent_type, py::capsule pyparent);

/// Get pending data from a ready QoS event.
/**
 * After having determined that a middleware event is ready, get the callback payload.
 *
 * Raises ValueError if \p pyevent is not an event capsule.
 * Raises MemoryError if event data can't be allocated.
 * Raises TypeError if arguments are not of the correct types.
 * Raises RCLError if taking event data failed in rcl.
 *
 * \param[in] pyevent Event object as returned by rclpy_create_event().
 * \param[in] pyevent_type Type of event, matching the one used for \p pyevent creation.
 * \param[in] pyparent Capsule containing the same publisher or subscription used for
 *   \p pyevent creation.
 * \return Event data as an instance of a suitable rclpy.qos_event type, or None
 *   if no event was taken.
 */
py::object
take_event(py::capsule pyevent, py::capsule pyparent, py::object pyevent_type);

}  // namespace rclpy

#endif  // RCLPY__QOS_EVENTS_HPP_
