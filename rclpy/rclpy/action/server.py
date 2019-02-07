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

from enum import Enum
import functools
import threading

from action_msgs.msg import GoalInfo, GoalStatus

from rclpy.executors import await_or_execute
from rclpy.impl.implementation_singleton import rclpy_action_implementation as _rclpy_action
from rclpy.qos import qos_profile_action_status_default
from rclpy.qos import qos_profile_default, qos_profile_services_default
from rclpy.task import Future
from rclpy.type_support import check_for_type_support
from rclpy.waitable import NumberOfEntities, Waitable


class GoalResponse(Enum):
    """Possible goal responses."""

    # Reject the goal.
    REJECT = 1
    # Accept the goal and start executing it right away.
    ACCEPT_AND_EXECUTE = 2
    # Accept the goal and defer execution until a later time.
    ACCEPT_AND_DEFER = 3


class CancelResponse(Enum):
    """Possible cancel responses."""

    REJECT = 1
    ACCEPT = 2


class GoalEvent(Enum):
    """Goal events that cause state transitions."""

    EXECUTE = 1
    CANCEL = 2
    SET_SUCCEEDED = 3
    SET_ABORTED = 4
    SET_CANCELED = 5


class ServerGoalHandle:
    """Goal handle for working with Action Servers."""

    def __init__(self, action_server, goal_info, goal_request):
        """
        Accept a new goal with the given action server.

        Instances of this class should only be constructed in the ActionServer class.
        Instances for accepted goals will be passed to the user-defined goal execution functions.

        If the goal fails to be accepted, then a RuntimeError is raised.

        :param action_server: The ActionServer to accept the goal.
        :param goal_info: GoalInfo message.
        :param goal_request: The original goal request message from an ActionClient.
        """
        self._handle = _rclpy_action.rclpy_action_accept_new_goal(
            action_server._handle, goal_info)
        self._action_server = action_server
        self._goal_info = goal_info
        self._goal_request = goal_request
        self._cancel_requested = False
        self._result_future = Future()
        action_server.add_future(self._result_future)
        self._lock = threading.Lock()

    def __eq__(self, other):
        return self.goal_id == other.goal_id

    def __ne__(self, other):
        return self.goal_id != other.goal_id

    @property
    def request(self):
        return self._goal_request

    @property
    def goal_id(self):
        return self._goal_info.goal_id

    @property
    def is_cancel_requested(self):
        with self._lock:
            return self._cancel_requested

    @property
    def status(self):
        with self._lock:
            if self._handle is None:
                return GoalStatus.STATUS_UNKNOWN
            return _rclpy_action.rclpy_action_goal_handle_get_status(self._handle)

    def _notify_cancel_requested(self):
        with self._lock:
            self._cancel_requested = True

    def _update_state(self, event):
        with self._lock:
            # Ignore updates for already destructed goal handles
            if self._handle is None:
                return

            _rclpy_action.rclpy_action_update_goal_state(self._handle, event.value)
            # If it's a terminal state, then also notify the action server
            if not _rclpy_action.rclpy_action_goal_handle_is_active(self._handle):
                self._action_server.notify_goal_done()

    def is_active(self):
        with self._lock:
            if self._handle is None:
                return False
            return _rclpy_action.rclpy_action_goal_handle_is_active(self._handle)

    def publish_feedback(self, feedback_msg):
        with self._lock:
            # Ignore for already destructed goal handles
            if self._handle is None:
                return
            _rclpy_action.rclpy_action_publish_feedback(self._action_sever._handle, feedback_msg)

    def set_succeeded(self):
        self._update_state(GoalEvent.SET_SUCCEEDED)

    def set_aborted(self):
        self._update_state(GoalEvent.SET_ABORTED)

    def set_canceled(self):
        self._update_state(GoalEvent.SET_CANCELED)

    def destroy(self):
        if self._handle is None:
            return
        self._action_server.remove_future(self._result_future)
        _rclpy_action.rclpy_action_destroy_server_goal_handle(self._handle)
        self._handle = None


def default_goal_callback(goal_request):
    """Accept all goals."""
    return GoalResponse.ACCEPT_AND_EXECUTE


def default_cancel_callback(cancel_request):
    """No cancellations."""
    return CancelResponse.REJECT


class ActionServer(Waitable):
    """ROS Action server."""

    def __init__(
        self,
        node,
        action_type,
        action_name,
        execute_callback,
        *,
        callback_group=None,
        goal_callback=default_goal_callback,
        cancel_callback=default_cancel_callback,
        goal_service_qos_profile=qos_profile_services_default,
        result_service_qos_profile=qos_profile_services_default,
        cancel_service_qos_profile=qos_profile_services_default,
        feedback_pub_qos_profile=qos_profile_default,
        status_pub_qos_profile=qos_profile_action_status_default,
        result_timeout=900
    ):
        """
        Constructor.

        :param node: The ROS node to add the action server to.
        :param action_type: Type of the action.
        :param action_name: Name of the action.
            Used as part of the underlying topic and service names.
        :param execute_callback: Callback function for processing accepted goals.
        :param callback_group: Callback group to add the action server to.
            If None, then the node's default callback group is used.
        :param goal_callback: Callback function for handling new goal requests.
        :param cancel_callback: Callback function for handling cancel requests.
        :param goal_service_qos_profile: QoS profile for the goal service.
        :param result_service_qos_profile: QoS profile for the result service.
        :param cancel_service_qos_profile: QoS profile for the cancel service.
        :param feedback_pub_qos_profile: QoS profile for the feedback publisher.
        :param status_pub_qos_profile: QoS profile for the status publisher.
        :param result_timeout: Goals that have results longer than this number of seconds
            are discarded.
        """
        if callback_group is None:
            callback_group = node.default_callback_group

        super().__init__(callback_group)

        self._lock = threading.Lock()

        self.register_goal_callback(goal_callback)
        self.register_cancel_callback(cancel_callback)
        self.register_execute_callback(execute_callback)

        # Import the typesupport for the action module if not already done
        check_for_type_support(action_type)
        self._node = node
        self._action_type = action_type
        self._handle = _rclpy_action.rclpy_action_create_server(
            node.handle,
            node.get_clock().handle,
            action_type,
            action_name,
            goal_service_qos_profile.get_c_qos_profile(),
            result_service_qos_profile.get_c_qos_profile(),
            cancel_service_qos_profile.get_c_qos_profile(),
            feedback_pub_qos_profile.get_c_qos_profile(),
            status_pub_qos_profile.get_c_qos_profile(),
            result_timeout,
        )

        # key: UUID in bytes, value: GoalHandle
        self._goal_handles = {}

        callback_group.add_entity(self)
        self._node.add_waitable(self)

    async def _execute_goal_request(self, request_header_and_message):
        request_header, goal_request = request_header_and_message
        goal_uuid = goal_request.action_goal_id
        goal_info = GoalInfo()
        goal_info.goal_id = goal_uuid
        goal_info.stamp = self._node.get_clock().now().to_msg()

        self._node.get_logger().debug('New goal request with ID: {0}'.format(goal_uuid.uuid))

        # Check if goal ID is already being tracked by this action server
        with self._lock:
            goal_id_exists = _rclpy_action.rclpy_action_server_goal_exists(self._handle, goal_info)

        accepted = False
        if not goal_id_exists:
            # Call user goal callback
            response = await await_or_execute(self._goal_callback, goal_request)
            accepted = ((GoalResponse.ACCEPT_AND_EXECUTE == response) or
                        (GoalResponse.ACCEPT_AND_DEFER == response))

        if accepted:
            # Create a goal handle
            try:
                with self._lock:
                    goal_handle = ServerGoalHandle(self, goal_info, goal_request)
            except RuntimeError as e:
                self._node.get_logger().error(
                    'Failed to accept new goal with ID {0}: {1}'.format(goal_uuid.uuid, e))
                accepted = False
            else:
                self._goal_handles[bytes(goal_uuid.uuid)] = goal_handle

        # Send response
        response_msg = self._action_type.GoalRequestService.Response()
        response_msg.accepted = accepted
        response_msg.stamp = self._node.get_clock().now().to_msg()
        _rclpy_action.rclpy_action_send_goal_response(
            self._handle,
            request_header,
            response_msg,
        )

        if not accepted:
            self._node.get_logger().debug('New goal rejected: {0}'.format(goal_uuid.uuid))
            return

        self._node.get_logger().debug('New goal accepted: {0}'.format(goal_uuid.uuid))

        goal_handle._update_state(GoalEvent.EXECUTE)
        # Call user execute callback
        execute_result = await await_or_execute(self._execute_callback, goal_handle)
        # If user did not trigger a terminal state, assume success
        if goal_handle.is_active():
            self._node.get_logger().warn(
                'Goal state not set, assuming success. Goal ID: {0}'.format(goal_uuid.uuid))
            goal_handle.set_succeeded()
        self._node.get_logger().debug(
            'Goal with ID {0} finished with state {1}'.format(goal_uuid.uuid, goal_handle.status))
        # Set result
        execute_result.action_status = goal_handle.status
        goal_handle._result_future.set_result(execute_result)

    async def _execute_cancel_request(self, request_header_and_message):
        request_header, cancel_request = request_header_and_message

        self._node.get_logger().debug('Cancel request received: {0}'.format(cancel_request))

        with self._lock:
            # Get list of goals that are requested to be canceled
            cancel_response = _rclpy_action.rclpy_action_process_cancel_request(
                self._handle, cancel_request, self._action_type.CancelGoalService.Response)

        for goal_info in cancel_response.goals_canceling:
            goal_uuid = bytes(goal_info.goal_id.uuid)
            if goal_uuid not in self._goal_handles:
                # Possibly the user doesn't care to track the goal handle
                # Remove from response
                cancel_response.goals_canceling.remove(goal_info)
                continue
            goal_handle = self._goal_handles[goal_uuid]
            response = await await_or_execute(self._cancel_callback, goal_handle)

            accepted = CancelResponse.ACCEPT == response
            if accepted:
                # Notify goal handle
                goal_handle._notify_cancel_requested()
            else:
                # Remove from response
                cancel_response.goals_canceling.remove(goal_info)

        _rclpy_action.rclpy_action_send_cancel_response(
            self._handle,
            request_header,
            cancel_response,
        )

    async def _execute_get_result_request(self, request_header_and_message):
        request_header, result_request = request_header_and_message
        goal_uuid = result_request.action_goal_id.uuid

        self._node.get_logger().debug(
            'Result request received for goal with ID: {0}'.format(goal_uuid))

        # If no goal with the requested ID exists, then return UNKNOWN status
        if bytes(goal_uuid) not in self._goal_handles:
            self._node.get_logger().debug(
                'Sending result response for unknown goal ID: {0}'.format(goal_uuid))
            result_response = self._action_type.Result()
            result_response.status = GoalStatus.STATUS_UNKNOWN
            _rclpy_action.rclpy_action_send_result_response(
                self._handle,
                request_header,
                result_response,
            )
            return

        # There is an accepted goal matching the goal ID, register a callback to send the
        # response as soon as it's ready
        self._goal_handles[bytes(goal_uuid)]._result_future.add_done_callback(
            functools.partial(self._send_result_response, request_header))

    async def _execute_expire_goals(self, expired_goals):
        for goal in expired_goals:
            goal_uuid = bytes(goal.goal_id.uuid)
            del self._goal_handles[goal_uuid]

    def _send_result_response(self, request_header, future):
        _rclpy_action.rclpy_action_send_result_response(
            self._handle,
            request_header,
            future.result(),
        )

    # Start Waitable API
    def is_ready(self, wait_set):
        """Return True if one or more entities are ready in the wait set."""
        with self._lock:
            ready_entities = _rclpy_action.rclpy_action_wait_set_is_ready(self._handle, wait_set)
        self._is_goal_request_ready = ready_entities[0]
        self._is_cancel_request_ready = ready_entities[1]
        self._is_result_request_ready = ready_entities[2]
        self._is_goal_expired = ready_entities[3]
        return any(ready_entities)

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        data = {}
        if self._is_goal_request_ready:
            with self._lock:
                data['goal'] = _rclpy_action.rclpy_action_take_goal_request(
                    self._handle,
                    self._action_type.GoalRequestService.Request,
                )

        if self._is_cancel_request_ready:
            with self._lock:
                data['cancel'] = _rclpy_action.rclpy_action_take_cancel_request(
                    self._handle,
                    self._action_type.CancelGoalService.Request,
                )

        if self._is_result_request_ready:
            with self._lock:
                data['result'] = _rclpy_action.rclpy_action_take_result_request(
                    self._handle,
                    self._action_type.GoalResultService.Request,
                )

        if self._is_goal_expired:
            with self._lock:
                data['expired'] = _rclpy_action.rclpy_action_expire_goals(
                    self._handle,
                    len(self._goal_handles),
                )

        if not any(data):
            return None
        return data

    async def execute(self, taken_data):
        """
        Execute work after data has been taken from a ready wait set.

        This will set results for Future objects for any received service responses and
        call any user-defined callbacks (e.g. feedback).
        """
        if 'goal' in taken_data:
            await self._execute_goal_request(taken_data['goal'])

        if 'cancel' in taken_data:
            await self._execute_cancel_request(taken_data['cancel'])

        if 'result' in taken_data:
            await self._execute_get_result_request(taken_data['result'])

        if 'expired' in taken_data:
            await self._execute_expire_goals(taken_data['expired'])

    def get_num_entities(self):
        """Return number of each type of entity used in the wait set."""
        num_entities = _rclpy_action.rclpy_action_wait_set_get_num_entities(self._handle)
        return NumberOfEntities(
            num_entities[0],
            num_entities[1],
            num_entities[2],
            num_entities[3],
            num_entities[4])

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        with self._lock:
            _rclpy_action.rclpy_action_wait_set_add(self._handle, wait_set)
    # End Waitable API

    def notify_goal_done(self):
        with self._lock:
            _rclpy_action.rclpy_action_notify_goal_done(self._handle)

    def register_goal_callback(self, goal_callback):
        """
        Register a callback for handling new goal requests.

        The purpose of the goal callback is to decide if a new goal should be accepted or
        rejected.
        The callback should take the goal request message as a parameter and must return a
        :class:`GoalResponse` value.

        There can only be one goal callback per :class:`ActionServer`, therefore calling this
        function will replace any previously registered callback.

        :param goal_callback: Callback function, if `None`, then unregisters any previously
            registered callback.
        """
        self._goal_callback = goal_callback

    def register_cancel_callback(self, cancel_callback):
        """
        Register a callback for handling cancel requests.

        The purpose of the cancel callback is to decide if a request to cancel an on-going
        (or queued) goal should be accepted or rejected.
        The callback should take one parameter containing the cancel request and must return a
        :class:`CancelResponse` value.

        There can only be one cancel callback per :class:`ActionServer`, therefore calling this
        function will replace any previously registered callback.

        :param cancel_callback: Callback function, if `None`, then unregisters any previously
            registered callback.
        """
        self._cancel_callback = cancel_callback

    def register_execute_callback(self, execute_callback):
        """
        Register a callback for executing action goals.

        The purpose of the execute callback is to execute the action goal and return a result
        when finished.

        The callback should take one parameter containing goal request and must return a
        result instance (i.e. `action_type.Result`).

        There can only be one execute callback per :class:`ActionServer`, therefore calling this
        function will replace any previously registered callback.

        :param execute_callback: Callback function, if `None`, then unregisters any previously
            registered callback.
        """
        self._execute_callback = execute_callback

    def destroy(self):
        """Destroy the underlying action server handle."""
        if self._handle is None or self._node.handle is None:
            return

        for goal_handle in self._goal_handles.values():
            goal_handle.destroy()

        _rclpy_action.rclpy_action_destroy_entity(self._handle, self._node.handle)
        self._node.remove_waitable(self)
        self._handle = None

    def __del__(self):
        """Destroy the underlying action server handle."""
        self.destroy()
