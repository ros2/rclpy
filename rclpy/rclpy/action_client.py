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

import rclpy
from rclpy.impl.implementation_singleton import rclpy_action_implementation as _rclpy_action
# TODO(jacobperron): Move check_for_type_support to it's own module (e.g. type_support)
from rclpy.node import check_for_type_support
from rclpy.qos import qos_profile_action_status_default
from rclpy.qos import qos_profile_default, qos_profile_services_default
from rclpy.task import Future
from rclpy.waitable import NumberOfEntities, Waitable


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
        self.action_type = action_type
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

        self._is_ready = False
        self._pending_goal_requests = {}
        self._pending_cancel_requests = {}
        self._pending_result_requests = {}

        callback_group.add_entity(self)
        self.node.add_waitable(self)

    def _remove_pending_request(self, future, pending_requests):
        """
        Remove a future from the list of pending requests.

        This prevents a future from receiving a request and executing its done callbacks.
        :param future: a future returned from :meth:`call_async`
        :type future: rclpy.task.Future
        """
        for seq, req_future in pending_requests.items():
            if future == req_future:
                try:
                    del pending_requests[seq]
                except KeyError:
                    pass
                break

    def _remove_pending_goal_request(self, future):
        self._remove_pending_request(future, self._pending_goal_requests)

    def _remove_pending_cancel_request(self, future):
        self._remove_pending_request(future, self._pending_cancel_requests)

    def _remove_pending_result_request(self, future):
        self._remove_pending_request(future, self._pending_result_requests)

    # Start Waitable API
    def is_ready(self, wait_set):
        """Return True if entities are ready in the wait set."""
        ready_entities = _rclpy_action.rclpy_action_wait_set_is_ready(self.client_handle, wait_set)
        self._is_feedback_ready = ready_entities[0]
        self._is_status_ready = ready_entities[1]
        self._is_goal_response_ready = ready_entities[2]
        self._is_cancel_response_ready = ready_entities[3]
        self._is_result_response_ready = ready_entities[4]
        return any(ready_entities)

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        data = {}
        if self._is_feedback_ready:
            feedback_msg = _rclpy_action.rclpy_action_take_feedback(
                self.client_handle, self.action_type.Feedback)
            data['feedback'] = feedback_msg

        if self._is_status_ready:
           status_msg  = _rclpy_action.rclpy_action_take_status(
                self.client_handle, self.action_type.GoalStatusMessage)
           data['status'] = status_msg

        if self._is_goal_response_ready:
            sequence_number, goal_response = _rclpy_action.rclpy_action_take_goal_response(
                self.client_handle, self.action_type.GoalRequestService.Response)
            data['goal'] = (sequence_number, goal_response)

        if self._is_cancel_response_ready:
           sequence_number, cancel_response  = _rclpy_action.rclpy_action_take_cancel_response(
                self.client_handle, self.action_type.CancelGoalService.Response)
           data['cancel'] = (sequence_number, cancel_response)

        if self._is_result_response_ready:
            sequence_number, result_response = _rclpy_action.rclpy_action_take_result_response(
                self.client_handle, self.action_type.GoalResultService.Response)
            data['result'] = (sequence_number, result_response)

        if not any(data):
            return None
        return data

    async def execute(self, taken_data):
        """Execute work after data has been taken from a ready wait set."""
        if 'goal' in taken_data:
            sequence_number, goal_response = taken_data['goal']
            self._pending_goal_requests[sequence_number].set_result(goal_response)
        # TODO(jacobperron): implement

    def get_num_entities(self):
        """Return number of each type of entity used."""
        return _rclpy_action.rclpy_action_wait_set_get_num_entities(self.client_handle)

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        _rclpy_action.rclpy_action_wait_set_add(self.client_handle, wait_set)
    # End Waitable API

    def send_goal(self, goal, **kwargs):
        """
        Send a goal and wait for the result.

        Do not call this method in a callback or a deadlock may occur.

        :param goal: The goal request
        :return: The result response
        """
        future = self.send_goal_async(goal, kwargs)
        while self.node.context.ok() and not future.done():
            time.sleep(0.1)
        return future.result()

    def send_goal_async(self, goal, feedback_callback=None):
        """
        Send a goal and asyncronously get the result.

        :param goal: The goal request
        :return: a Future instance to a goal handle that completes when the goal request
            has been accepted or rejected.
        :rtype: :class:`rclpy.task.Future` instance
        """
        sequence_number = _rclpy_action.rclpy_action_send_goal_request(self.client_handle, goal)
        if sequence_number in self._pending_goal_requests:
            raise RuntimeError(
                'Sequence ({}) conflicts with pending goal request'.format(sequence_number))

        future = Future()
        self._pending_goal_requests[sequence_number] = future
        future.add_done_callback(self._remove_pending_goal_request)

        return future

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
        """Destroy the underlying action client handle."""
        if self.client_handle is None:
            return
        _rclpy_action.rclpy_action_destroy_entity(self.client_handle, self.node.handle)
        self.node.remove_waitable(self)
        self.client_handle = None

    def __del__(self):
        """Destroy the underlying action client handle."""
        self.destroy()
