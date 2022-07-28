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

import threading
import time
import uuid
import weakref

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

from rclpy.executors import await_or_execute
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import qos_profile_action_status_default
from rclpy.qos import qos_profile_services_default
from rclpy.qos import QoSProfile
from rclpy.task import Future
from rclpy.type_support import check_for_type_support
from rclpy.waitable import NumberOfEntities, Waitable

from unique_identifier_msgs.msg import UUID


class ClientGoalHandle():
    """Goal handle for working with Action Clients."""

    def __init__(self, action_client, goal_id, goal_response):
        self._action_client = action_client
        self._goal_id = goal_id
        self._goal_response = goal_response
        self._status = GoalStatus.STATUS_UNKNOWN

    def __eq__(self, other):
        return self._goal_id == other.goal_id

    def __ne__(self, other):
        return self._goal_id != other.goal_id

    def __repr__(self):
        return 'ClientGoalHandle <id={0}, accepted={1}, status={2}>'.format(
                self.goal_id.uuid,
                self.accepted,
                self.status)

    @property
    def goal_id(self):
        return self._goal_id

    @property
    def stamp(self):
        return self._goal_response.stamp

    @property
    def accepted(self):
        return self._goal_response.accepted

    @property
    def status(self):
        return self._status

    def cancel_goal(self):
        """
        Send a cancel request for the goal and wait for the response.

        Do not call this method in a callback or a deadlock may occur.

        :return: The cancel response.
        """
        return self._action_client._cancel_goal(self)

    def cancel_goal_async(self):
        """
        Asynchronous request for the goal be canceled.

        :return: a Future instance that completes when the server responds.
        :rtype: :class:`rclpy.task.Future` instance
        """
        return self._action_client._cancel_goal_async(self)

    def get_result(self):
        """
        Request the result for the goal and wait for the response.

        Do not call this method in a callback or a deadlock may occur.

        :return: The result response.
        """
        return self._action_client._get_result(self)

    def get_result_async(self):
        """
        Asynchronously request the goal result.

        :return: a Future instance that completes when the result is ready.
        :rtype: :class:`rclpy.task.Future` instance
        """
        return self._action_client._get_result_async(self)


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
        feedback_sub_qos_profile=QoSProfile(depth=10),
        status_sub_qos_profile=qos_profile_action_status_default
    ):
        """
        Create an ActionClient.

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
        self._node = node
        self._action_type = action_type
        self._action_name = action_name
        with node.handle:
            self._client_handle = _rclpy.ActionClient(
                node.handle,
                action_type,
                action_name,
                goal_service_qos_profile.get_c_qos_profile(),
                result_service_qos_profile.get_c_qos_profile(),
                cancel_service_qos_profile.get_c_qos_profile(),
                feedback_sub_qos_profile.get_c_qos_profile(),
                status_sub_qos_profile.get_c_qos_profile()
            )

        self._is_ready = False

        # key: UUID in bytes, value: weak reference to ClientGoalHandle
        self._goal_handles = {}
        # key: goal request sequence_number, value: Future for goal response
        self._pending_goal_requests = {}
        # key: goal request sequence_number, value: UUID
        self._goal_sequence_number_to_goal_id = {}
        # key: cancel request sequence number, value: Future for cancel response
        self._pending_cancel_requests = {}
        # key: result request sequence number, value: Future for result response
        self._pending_result_requests = {}
        # key: result request sequence_number, value: UUID
        self._result_sequence_number_to_goal_id = {}
        # key: UUID in bytes, value: callback function
        self._feedback_callbacks = {}

        callback_group.add_entity(self)
        self._node.add_waitable(self)
        self._logger = self._node.get_logger().get_child('action_client')

    def _generate_random_uuid(self):
        return UUID(uuid=list(uuid.uuid4().bytes))

    def _remove_pending_request(self, future, pending_requests):
        """
        Remove a future from the list of pending requests.

        This prevents a future from receiving a request and executing its done callbacks.
        :param future: a future returned from one of :meth:`send_goal_async`,
            :meth:`_cancel_goal_async`, or :meth:`_get_result_async`.
        :type future: rclpy.task.Future
        :param pending_requests: The list of pending requests.
        :type pending_requests: dict
        :return: The sequence number associated with the removed future, or
            None if the future was not found in the list.
        """
        for seq, req_future in list(pending_requests.items()):
            if future == req_future:
                try:
                    del pending_requests[seq]
                except KeyError:
                    pass
                else:
                    self.remove_future(future)
                    return seq
        return None

    def _remove_pending_goal_request(self, future):
        seq = self._remove_pending_request(future, self._pending_goal_requests)
        if seq in self._goal_sequence_number_to_goal_id:
            del self._goal_sequence_number_to_goal_id[seq]

    def _remove_pending_cancel_request(self, future):
        self._remove_pending_request(future, self._pending_cancel_requests)

    def _remove_pending_result_request(self, future):
        seq = self._remove_pending_request(future, self._pending_result_requests)
        if seq in self._result_sequence_number_to_goal_id:
            goal_uuid = bytes(self._result_sequence_number_to_goal_id[seq].uuid)
            del self._result_sequence_number_to_goal_id[seq]
            # remove feeback_callback if user is aware of result and it's been received
            if goal_uuid in self._feedback_callbacks:
                del self._feedback_callbacks[goal_uuid]

    # Start Waitable API
    def is_ready(self, wait_set):
        """Return True if one or more entities are ready in the wait set."""
        ready_entities = self._client_handle.is_ready(wait_set)
        self._is_feedback_ready = ready_entities[0]
        self._is_status_ready = ready_entities[1]
        self._is_goal_response_ready = ready_entities[2]
        self._is_cancel_response_ready = ready_entities[3]
        self._is_result_response_ready = ready_entities[4]
        return any(ready_entities)

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        data = {}
        if self._is_goal_response_ready:
            taken_data = self._client_handle.take_goal_response(
                self._action_type.Impl.SendGoalService.Response)
            # If take fails, then we get (None, None)
            if all(taken_data):
                data['goal'] = taken_data

        if self._is_cancel_response_ready:
            taken_data = self._client_handle.take_cancel_response(
                self._action_type.Impl.CancelGoalService.Response)
            # If take fails, then we get (None, None)
            if all(taken_data):
                data['cancel'] = taken_data

        if self._is_result_response_ready:
            taken_data = self._client_handle.take_result_response(
                self._action_type.Impl.GetResultService.Response)
            # If take fails, then we get (None, None)
            if all(taken_data):
                data['result'] = taken_data

        if self._is_feedback_ready:
            taken_data = self._client_handle.take_feedback(
                self._action_type.Impl.FeedbackMessage)
            # If take fails, then we get None
            if taken_data is not None:
                data['feedback'] = taken_data

        if self._is_status_ready:
            taken_data = self._client_handle.take_status(
                self._action_type.Impl.GoalStatusMessage)
            # If take fails, then we get None
            if taken_data is not None:
                data['status'] = taken_data

        return data

    async def execute(self, taken_data):
        """
        Execute work after data has been taken from a ready wait set.

        This will set results for Future objects for any received service responses and
        call any user-defined callbacks (e.g. feedback).
        """
        if 'goal' in taken_data:
            sequence_number, goal_response = taken_data['goal']
            if sequence_number in self._goal_sequence_number_to_goal_id:
                goal_handle = ClientGoalHandle(
                    self,
                    self._goal_sequence_number_to_goal_id[sequence_number],
                    goal_response)

                if goal_handle.accepted:
                    goal_uuid = bytes(goal_handle.goal_id.uuid)
                    if goal_uuid in self._goal_handles:
                        raise RuntimeError(
                            'Two goals were accepted with the same ID ({})'.format(goal_handle))
                    self._goal_handles[goal_uuid] = weakref.ref(goal_handle)

                self._pending_goal_requests[sequence_number].set_result(goal_handle)
            else:
                self._logger.warning(
                    'Ignoring unexpected goal response. There may be more than '
                    f"one action server for the action '{self._action_name}'"
                )

        if 'cancel' in taken_data:
            sequence_number, cancel_response = taken_data['cancel']
            if sequence_number in self._pending_cancel_requests:
                self._pending_cancel_requests[sequence_number].set_result(cancel_response)
            else:
                self._logger.warning(
                    'Ignoring unexpected cancel response. There may be more than '
                    f"one action server for the action '{self._action_name}'"
                )

        if 'result' in taken_data:
            sequence_number, result_response = taken_data['result']
            if sequence_number in self._pending_result_requests:
                self._pending_result_requests[sequence_number].set_result(result_response)
            else:
                self._logger.warning(
                    'Ignoring unexpected result response. There may be more than '
                    f"one action server for the action '{self._action_name}'"
                )

        if 'feedback' in taken_data:
            feedback_msg = taken_data['feedback']
            goal_uuid = bytes(feedback_msg.goal_id.uuid)
            # Call a registered callback if there is one
            if goal_uuid in self._feedback_callbacks:
                await await_or_execute(self._feedback_callbacks[goal_uuid], feedback_msg)

        if 'status' in taken_data:
            # Update the status of all goal handles maintained by this Action Client
            for status_msg in taken_data['status'].status_list:
                goal_uuid = bytes(status_msg.goal_info.goal_id.uuid)
                status = status_msg.status

                if goal_uuid in self._goal_handles:
                    goal_handle = self._goal_handles[goal_uuid]()
                    if goal_handle is not None:
                        goal_handle._status = status
                        # Remove "done" goals from the list
                        if (GoalStatus.STATUS_SUCCEEDED == status or
                                GoalStatus.STATUS_CANCELED == status or
                                GoalStatus.STATUS_ABORTED == status):
                            del self._goal_handles[goal_uuid]
                    else:
                        # Weak reference is None
                        del self._goal_handles[goal_uuid]

    def get_num_entities(self):
        """Return number of each type of entity used in the wait set."""
        num_entities = self._client_handle.get_num_entities()
        return NumberOfEntities(*num_entities)

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        self._client_handle.add_to_waitset(wait_set)

    def __enter__(self):
        return self._client_handle.__enter__()

    def __exit__(self, t, v, tb):
        self._client_handle.__exit__(t, v, tb)

    # End Waitable API

    def send_goal(self, goal, **kwargs):
        """
        Send a goal and wait for the result.

        Do not call this method in a callback or a deadlock may occur.

        See :meth:`send_goal_async` for more info about keyword arguments.

        Unlike :meth:`send_goal_async`, this method returns the final result of the
        action (not a goal handle).

        :param goal: The goal request.
        :type goal: action_type.Goal
        :return: The result response.
        :rtype: action_type.Result
        :raises: TypeError if the type of the passed goal isn't an instance of
          the Goal type of the provided action when the service was
          constructed.
        """
        if not isinstance(goal, self._action_type.Goal):
            raise TypeError()

        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        send_goal_future = self.send_goal_async(goal, **kwargs)
        send_goal_future.add_done_callback(unblock)

        event.wait()
        if send_goal_future.exception() is not None:
            raise send_goal_future.exception()

        goal_handle = send_goal_future.result()

        result = self._get_result(goal_handle)

        return result

    def send_goal_async(self, goal, feedback_callback=None, goal_uuid=None):
        """
        Send a goal and asynchronously get the result.

        The result of the returned Future is set to a ClientGoalHandle when receipt of the goal
        is acknowledged by an action server.

        :param goal: The goal request.
        :type goal: action_type.Goal
        :param feedback_callback: Callback function for feedback associated with the goal.
        :type feedback_callback: function
        :param goal_uuid: Universally unique identifier for the goal.
            If None, then a random UUID is generated.
        :type: unique_identifier_msgs.UUID
        :return: a Future instance to a goal handle that completes when the goal request
            has been accepted or rejected.
        :rtype: :class:`rclpy.task.Future` instance
        :raises: TypeError if the type of the passed goal isn't an instance of
          the Goal type of the provided action when the service was
          constructed.
        """
        if not isinstance(goal, self._action_type.Goal):
            raise TypeError()

        request = self._action_type.Impl.SendGoalService.Request()
        request.goal_id = self._generate_random_uuid() if goal_uuid is None else goal_uuid
        request.goal = goal
        sequence_number = self._client_handle.send_goal_request(request)
        if sequence_number in self._pending_goal_requests:
            raise RuntimeError(
                'Sequence ({}) conflicts with pending goal request'.format(sequence_number))

        if feedback_callback is not None:
            # TODO(jacobperron): Move conversion function to a general-use package
            goal_uuid = bytes(request.goal_id.uuid)
            self._feedback_callbacks[goal_uuid] = feedback_callback

        future = Future()
        self._pending_goal_requests[sequence_number] = future
        self._goal_sequence_number_to_goal_id[sequence_number] = request.goal_id
        future.add_done_callback(self._remove_pending_goal_request)
        # Add future so executor is aware
        self.add_future(future)

        return future

    def _cancel_goal(self, goal_handle):
        """
        Send a cancel request for an active goal and wait for the response.

        Do not call this method in a callback or a deadlock may occur.

        :param goal_handle: Handle to the goal to cancel.
        :type goal_handle: :class:`ClientGoalHandle`
        :return: The cancel response.
        """
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        future = self._cancel_goal_async(goal_handle)
        future.add_done_callback(unblock)

        event.wait()
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    def _cancel_goal_async(self, goal_handle):
        """
        Send a cancel request for an active goal and asynchronously get the result.

        :param goal_handle: Handle to the goal to cancel.
        :type goal_handle: :class:`ClientGoalHandle`
        :return: a Future instance that completes when the cancel request has been processed.
        :rtype: :class:`rclpy.task.Future` instance
        """
        if not isinstance(goal_handle, ClientGoalHandle):
            raise TypeError(
                'Expected type ClientGoalHandle but received {}'.format(type(goal_handle)))

        cancel_request = CancelGoal.Request()
        cancel_request.goal_info.goal_id = goal_handle.goal_id
        sequence_number = self._client_handle.send_cancel_request(cancel_request)
        if sequence_number in self._pending_cancel_requests:
            raise RuntimeError(
                'Sequence ({}) conflicts with pending cancel request'.format(sequence_number))

        future = Future()
        self._pending_cancel_requests[sequence_number] = future
        future.add_done_callback(self._remove_pending_cancel_request)
        # Add future so executor is aware
        self.add_future(future)

        return future

    def _get_result(self, goal_handle):
        """
        Request the result for an active goal and wait for the response.

        Do not call this method in a callback or a deadlock may occur.

        :param goal_handle: Handle to the goal to get the result for.
        :type goal_handle: :class:`ClientGoalHandle`
        :return: The result response.
        """
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        future = self._get_result_async(goal_handle)
        future.add_done_callback(unblock)

        event.wait()
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    def _get_result_async(self, goal_handle):
        """
        Request the result for an active goal asynchronously.

        :param goal_handle: Handle to the goal to cancel.
        :type goal_handle: :class:`ClientGoalHandle`
        :return: a Future instance that completes when the get result request has been processed.
        :rtype: :class:`rclpy.task.Future` instance
        """
        if not isinstance(goal_handle, ClientGoalHandle):
            raise TypeError(
                'Expected type ClientGoalHandle but received {}'.format(type(goal_handle)))

        result_request = self._action_type.Impl.GetResultService.Request()
        result_request.goal_id = goal_handle.goal_id
        sequence_number = self._client_handle.send_result_request(result_request)
        if sequence_number in self._pending_result_requests:
            raise RuntimeError(
                'Sequence ({}) conflicts with pending result request'.format(sequence_number))

        future = Future()
        self._pending_result_requests[sequence_number] = future
        self._result_sequence_number_to_goal_id[sequence_number] = result_request.goal_id
        future.add_done_callback(self._remove_pending_result_request)
        # Add future so executor is aware
        self.add_future(future)

        return future

    def server_is_ready(self):
        """
        Check if there is an action server ready to process requests from this client.

        :return: True if an action server is ready, False otherwise.
        """
        with self._node.handle:
            return self._client_handle.is_action_server_available()

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
        while self._node.context.ok() and not self.server_is_ready() and timeout_sec > 0.0:
            time.sleep(sleep_time)
            timeout_sec -= sleep_time

        return self.server_is_ready()

    def destroy(self):
        """Destroy the underlying action client handle."""
        self._client_handle.destroy_when_not_in_use()
        self._node.remove_waitable(self)
