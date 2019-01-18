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

# import threading
import time

from rclpy.impl.implementation_singleton import rclpy_action_implementation as _rclpy_action
# from rclpy.task import Future

# TODO(jacobperron): Move check_for_type_support to it's own module (e.g. type_support)
from rclpy.node import check_for_type_support
from rclpy.qos import qos_profile_action_status_default
from rclpy.qos import qos_profile_default, qos_profile_services_default
from rclpy.waitable import Waitable


class ActionClient(Waitable):
    """ROS Action client."""

    def __init__(
        self,
        node,
        action_type,
        action_name,
        *,
        callback_group=None,
        goal_service_qos_profile=qos_profile_services_default,
        result_service_qos_profile=qos_profile_services_default,
        cancel_service_qos_profile=qos_profile_services_default,
        feedback_sub_qos_profile=qos_profile_default,
        status_sub_qos_profile=qos_profile_action_status_default
    ):
        """Constructor.

        :param node: The ROS node to add the action client to.
        :param action_type: Type of the action.
        :param action_name: Name of the action.
            Used as part of the underlying topic and service names.
        :param callback_group: Callback group to add the action client to.
            If None, then the node's default callback group is used.
        :param goal_service_qos_profile: QoS profile for the goal service.
        :param result_service_qos_profile: QoS profile for the result service.
        :param cancel_service_qos_profile: QoS profile for the cancel service.
        :param feedback_sub_qos_profile: QoS profile for the feedback subscriber.
        :param status_sub_qos_profile: QoS profile for the status subscriber.
        """
        if callback_group is None:
            callback_group = node.default_callback_group

        super().__init__(callback_group)

        # Import the typesupport for the action module if not already done
        check_for_type_support(action_type)
        self.node = node
        self.client_handle = _rclpy_action.rclpy_action_create_client(
            node.handle,
            action_type,
            action_name,
            goal_service_qos_profile,
            result_service_qos_profile,
            cancel_service_qos_profile,
            feedback_sub_qos_profile,
            status_sub_qos_profile
        )

        self.node.add_waitable(self)

    # Start Waitable API
    def is_ready(self, wait_set):
        """Return True if entities are ready in the wait set."""
        # TODO: Return tuple of ready flags and use them in take_data() accordingly
        return _rclpy_action.rclpy_action_wait_set_is_ready(self.client_handle, wait_set)

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        raise NotImplementedError('Must be implemented by subclass')

    async def execute(self, taken_data):
        """Execute work after data has been taken from a ready wait set."""
        raise NotImplementedError('Must be implemented by subclass')

    def get_num_entities(self):
        """Return number of each type of entity used."""
        return _rclpy_action.rclpy_action_wait_set_get_num_entities(self.client_handle, wait_set)

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        _rclpy_action.rclpy_action_wait_set_add(self.client_handle, wait_set)
    # End Waitable API

    def send_goal(self, goal):
        """
        Send a goal and wait for the result.

        Do not call this method in a callback or a deadlock may occur.

        :param goal: The goal request
        :return: The result response
        """
        pass


    def send_goal_async(self, goal):
        """
        Send a goal and asyncronously get the result.

        :param goal: The goal request
        :return: a Future instance to a goal handle that completes when the goal request
            has been accepted or rejected.
        :rtype: :class:`rclpy.task.Future` instance
        """
        pass

    def cancel_goal(self, goal_handle):
        """
        Send a cancel request for an active goal and wait for the response.

        Do not call this method in a callback or a deadlock may occur.

        :param goal_handle: Handle to the goal to cancel.
        :return: The result response.
        """
        pass

    def cancel_goal_async(self, goal_handle):
        """
        Send a cancel request for an active goal and asyncronously get the result.

        :param goal_handle: Handle to the goal to cancel.
        :return: a Future instance that completes when the cancel request has been processed.
        :rtype: :class:`rclpy.task.Future` instance
        """
        pass

    def server_is_ready(self):
        """
        Check if there is an action server ready to process requests from this client.

        :return: True if an action server is ready, False otherwise.
        """
        return _rclpy_action.rclpy_action_server_is_available(self.node.handle, self.client_handle)

    def wait_for_server(self, timeout_sec=None):
        """
        Wait for an action sever to be ready.

        Returns as soon as an action server is ready for this client.
        :param timeout_sec: Number of seconds to wait until an action server is available.
            If None, then wait indefinitely.
        :return: True if an action server is available, False if the timeout is exceeded.
        """
        # TODO(jacobperron): Remove arbitrary sleep time and return as soon as server is ready
        #                    See https://github.com/ros2/rclpy/issues/58
        sleep_time = 0.25
        if timeout_sec is None:
            timeout_sec = float('inf')
        while self.node.context.ok() and not self.server_is_ready() and timeout_sec > 0.0:
            time.sleep(sleep_time)
            timeout_sec -= sleep_time

        return self.server_is_ready()

    def destroy(self):
        if self.client_handle is None:
            return
        _rclpy_action.rclpy_action_destroy_entity(self.client_handle, self.node.handle)
        self.client_handle = None

    def __del__(self):
        self.destroy()
