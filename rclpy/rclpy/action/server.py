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
import traceback

from action_msgs.msg import GoalInfo, GoalStatus

from rclpy.executors import await_or_execute
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import qos_profile_action_status_default
from rclpy.qos import qos_profile_services_default
from rclpy.qos import QoSProfile
from rclpy.task import Future
from rclpy.type_support import check_for_type_support
from rclpy.waitable import NumberOfEntities, Waitable

# Re-export exception defined in _rclpy C extension.
RCLError = _rclpy.RCLError


class GoalResponse(Enum):
    """Possible goal responses."""

    REJECT = 1
    ACCEPT = 2


class CancelResponse(Enum):
    """Possible cancel responses."""

    REJECT = 1
    ACCEPT = 2


GoalEvent = _rclpy.GoalEvent


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
        :param goal_request: The user defined goal request message from an ActionClient.
        """
        self._goal_handle = _rclpy.ActionGoalHandle(action_server._handle, goal_info)
        self._action_server = action_server
        self._goal_info = goal_info
        self._goal_request = goal_request
        self._cancel_requested = False
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
    def is_active(self):
        with self._lock:
            if self._goal_handle is None:
                return False
            return self._goal_handle.is_active()

    @property
    def is_cancel_requested(self):
        return GoalStatus.STATUS_CANCELING == self.status

    @property
    def status(self):
        with self._lock:
            if self._goal_handle is None:
                return GoalStatus.STATUS_UNKNOWN
            return self._goal_handle.get_status()

    def _update_state(self, event):
        with self._lock:
            # Ignore updates for already destructed goal handles
            if self._goal_handle is None:
                return

            # Update state
            self._goal_handle.update_goal_state(event)

            # Publish state change
            self._action_server._handle.publish_status()

            # If it's a terminal state, then also notify the action server
            if not self._goal_handle.is_active():
                self._action_server.notify_goal_done()

    def execute(self, execute_callback=None):
        # It's possible that there has been a request to cancel the goal prior to executing.
        # In this case we want to avoid the illegal state transition to EXECUTING
        # but still call the users execute callback to let them handle canceling the goal.
        if not self.is_cancel_requested:
            self._update_state(_rclpy.GoalEvent.EXECUTE)
        self._action_server.notify_execute(self, execute_callback)

    def publish_feedback(self, feedback):
        if not isinstance(feedback, self._action_server.action_type.Feedback):
            raise TypeError()

        with self._lock:
            # Ignore for already destructed goal handles
            if self._goal_handle is None:
                return

            # Populate the feedback message with metadata about this goal
            # and the user defined message
            feedback_message = self._action_server.action_type.Impl.FeedbackMessage()
            feedback_message.goal_id = self.goal_id
            feedback_message.feedback = feedback

            # Publish
            self._action_server._handle.publish_feedback(feedback_message)

    def succeed(self):
        self._update_state(_rclpy.GoalEvent.SUCCEED)

    def abort(self):
        self._update_state(_rclpy.GoalEvent.ABORT)

    def canceled(self):
        self._update_state(_rclpy.GoalEvent.CANCELED)

    def destroy(self):
        with self._lock:
            if self._goal_handle is None:
                return
            self._goal_handle.destroy_when_not_in_use()
            self._goal_handle = None


def default_handle_accepted_callback(goal_handle):
    """Execute the goal."""
    goal_handle.execute()


def default_goal_callback(goal_request):
    """Accept all goals."""
    return GoalResponse.ACCEPT


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
        handle_accepted_callback=default_handle_accepted_callback,
        cancel_callback=default_cancel_callback,
        goal_service_qos_profile=qos_profile_services_default,
        result_service_qos_profile=qos_profile_services_default,
        cancel_service_qos_profile=qos_profile_services_default,
        feedback_pub_qos_profile=QoSProfile(depth=10),
        status_pub_qos_profile=qos_profile_action_status_default,
        result_timeout=900
    ):
        """
        Create an ActionServer.

        :param node: The ROS node to add the action server to.
        :param action_type: Type of the action.
        :param action_name: Name of the action.
            Used as part of the underlying topic and service names.
        :param execute_callback: Callback function for processing accepted goals.
            This is called if when :class:`ServerGoalHandle.execute()` is called for
            a goal handle that is being tracked by this action server.
        :param callback_group: Callback group to add the action server to.
            If None, then the node's default callback group is used.
        :param goal_callback: Callback function for handling new goal requests.
        :param handle_accepted_callback: Callback function for handling newly accepted goals.
            Passes an instance of `ServerGoalHandle` as an argument.
        :param cancel_callback: Callback function for handling cancel requests.
        :param goal_service_qos_profile: QoS profile for the goal service.
        :param result_service_qos_profile: QoS profile for the result service.
        :param cancel_service_qos_profile: QoS profile for the cancel service.
        :param feedback_pub_qos_profile: QoS profile for the feedback publisher.
        :param status_pub_qos_profile: QoS profile for the status publisher.
        :param result_timeout: How long in seconds a result is kept by the server after a goal
            reaches a terminal state.
        """
        if callback_group is None:
            callback_group = node.default_callback_group

        super().__init__(callback_group)

        self._lock = threading.Lock()

        self.register_handle_accepted_callback(handle_accepted_callback)
        self.register_goal_callback(goal_callback)
        self.register_cancel_callback(cancel_callback)
        self.register_execute_callback(execute_callback)

        # Import the typesupport for the action module if not already done
        check_for_type_support(action_type)
        self._node = node
        self._action_type = action_type
        with node.handle, node.get_clock().handle:
            self._handle = _rclpy.ActionServer(
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

        # key: UUID in bytes, value: Future
        self._result_futures = {}

        callback_group.add_entity(self)
        self._node.add_waitable(self)
        self._logger = self._node.get_logger().get_child('action_server')

    async def _execute_goal_request(self, request_header_and_message):
        request_header, goal_request = request_header_and_message
        goal_uuid = goal_request.goal_id
        goal_info = GoalInfo()
        goal_info.goal_id = goal_uuid

        self._logger.debug('New goal request with ID: {0}'.format(goal_uuid.uuid))

        # Check if goal ID is already being tracked by this action server
        with self._lock:
            goal_id_exists = self._handle.goal_exists(goal_info)

        accepted = False
        if not goal_id_exists:
            # Call user goal callback
            response = await await_or_execute(self._goal_callback, goal_request.goal)
            if not isinstance(response, GoalResponse):
                self._logger.warning(
                    'Goal request callback did not return a GoalResponse type. Rejecting goal.')
            else:
                accepted = GoalResponse.ACCEPT == response

        if accepted:
            # Stamp time of acceptance
            goal_info.stamp = self._node.get_clock().now().to_msg()

            # Create a goal handle
            try:
                with self._lock:
                    goal_handle = ServerGoalHandle(self, goal_info, goal_request.goal)
            except RuntimeError as e:
                self._logger.error(
                    'Failed to accept new goal with ID {0}: {1}'.format(goal_uuid.uuid, e))
                accepted = False
            else:
                self._goal_handles[bytes(goal_uuid.uuid)] = goal_handle
                self._result_futures[bytes(goal_uuid.uuid)] = Future()
                self.add_future(self._result_futures[bytes(goal_uuid.uuid)])

        # Send response
        response_msg = self._action_type.Impl.SendGoalService.Response()
        response_msg.accepted = accepted
        response_msg.stamp = goal_info.stamp

        try:
            # If the client goes away anytime before this, sending the goal response may fail.
            # Catch the exception here and go on so we don't crash.
            self._handle.send_goal_response(request_header, response_msg)
        except RCLError:
            self._logger.warn('Failed to send goal response (the client may have gone away)')
            return

        if not accepted:
            self._logger.debug('New goal rejected: {0}'.format(goal_uuid.uuid))
            return

        self._logger.debug('New goal accepted: {0}'.format(goal_uuid.uuid))

        # Provide the user a reference to the goal handle
        await await_or_execute(self._handle_accepted_callback, goal_handle)

    async def _execute_goal(self, execute_callback, goal_handle):
        goal_uuid = goal_handle.goal_id.uuid
        self._logger.debug('Executing goal with ID {0}'.format(goal_uuid))

        try:
            # Execute user callback
            execute_result = await await_or_execute(execute_callback, goal_handle)
        except Exception as ex:
            # Create an empty result so that we can still send a response to the client
            execute_result = self._action_type.Result()
            self._logger.error('Error raised in execute callback: {0}'.format(ex))
            traceback.print_exc()

        # If user did not trigger a terminal state, assume aborted
        if goal_handle.is_active:
            self._logger.warning(
                'Goal state not set, assuming aborted. Goal ID: {0}'.format(goal_uuid))
            goal_handle.abort()

        self._logger.debug(
            'Goal with ID {0} finished with state {1}'.format(goal_uuid, goal_handle.status))

        # Set result
        result_response = self._action_type.Impl.GetResultService.Response()
        result_response.status = goal_handle.status
        result_response.result = execute_result
        self._result_futures[bytes(goal_uuid)].set_result(result_response)

    async def _execute_cancel_request(self, request_header_and_message):
        request_header, cancel_request = request_header_and_message

        self._logger.debug('Cancel request received: {0}'.format(cancel_request))

        with self._lock:
            # Get list of goals that are requested to be canceled
            cancel_response = self._handle.process_cancel_request(
                cancel_request, self._action_type.Impl.CancelGoalService.Response)

        for goal_info in cancel_response.goals_canceling:
            goal_uuid = bytes(goal_info.goal_id.uuid)
            if goal_uuid not in self._goal_handles:
                # Possibly the user doesn't care to track the goal handle
                # Remove from response
                cancel_response.goals_canceling.remove(goal_info)
                continue
            goal_handle = self._goal_handles[goal_uuid]
            response = await await_or_execute(self._cancel_callback, goal_handle)

            if CancelResponse.ACCEPT == response:
                # Notify goal handle
                try:
                    # If the goal's just succeeded after user cancel callback
                    # that will generate an exception from invalid transition.
                    goal_handle._update_state(GoalEvent.CANCEL_GOAL)
                except RCLError as ex:
                    self._logger.debug(
                        'Failed to cancel goal in cancel callback: {0}'.format(ex))
                    # Remove from response since goal has been succeeded
                    cancel_response.goals_canceling.remove(goal_info)
            else:
                # Remove from response
                cancel_response.goals_canceling.remove(goal_info)

        try:
            # If the client goes away anytime before this, sending the goal response may fail.
            # Catch the exception here and go on so we don't crash.
            self._handle.send_cancel_response(request_header, cancel_response)
        except RCLError:
            self._logger.warn('Failed to send cancel response (the client may have gone away)')

    async def _execute_get_result_request(self, request_header_and_message):
        request_header, result_request = request_header_and_message
        goal_uuid = result_request.goal_id.uuid

        self._logger.debug(
            'Result request received for goal with ID: {0}'.format(goal_uuid))

        # If no goal with the requested ID exists, then return UNKNOWN status
        if bytes(goal_uuid) not in self._goal_handles:
            self._logger.debug(
                'Sending result response for unknown goal ID: {0}'.format(goal_uuid))
            result_response = self._action_type.Impl.GetResultService.Response()
            result_response.status = GoalStatus.STATUS_UNKNOWN
            self._handle.send_result_response(request_header, result_response)
            return

        # There is an accepted goal matching the goal ID, register a callback to send the
        # response as soon as it's ready
        self._result_futures[bytes(goal_uuid)].add_done_callback(
            functools.partial(self._send_result_response, request_header))

    async def _execute_expire_goals(self, expired_goals):
        for goal in expired_goals:
            goal_uuid = bytes(goal.goal_id.uuid)
            self._goal_handles[goal_uuid].destroy()
            del self._goal_handles[goal_uuid]
            self.remove_future(self._result_futures[goal_uuid])
            del self._result_futures[goal_uuid]

    def _send_result_response(self, request_header, future):
        try:
            # If the client goes away anytime before this, sending the result response may fail.
            # Catch the exception here and go on so we don't crash.
            self._handle.send_result_response(request_header, future.result())
        except RCLError:
            self._logger.warn('Failed to send result response (the client may have gone away)')

    @property
    def action_type(self):
        return self._action_type

    # Start Waitable API
    def is_ready(self, wait_set):
        """Return True if one or more entities are ready in the wait set."""
        with self._lock:
            ready_entities = self._handle.is_ready(wait_set)
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
                taken_data = self._handle.take_goal_request(
                    self._action_type.Impl.SendGoalService.Request,
                )
                # If take fails, then we get (None, None)
                if all(taken_data):
                    data['goal'] = taken_data

        if self._is_cancel_request_ready:
            with self._lock:
                taken_data = self._handle.take_cancel_request(
                    self._action_type.Impl.CancelGoalService.Request,
                )
                # If take fails, then we get (None, None)
                if all(taken_data):
                    data['cancel'] = taken_data

        if self._is_result_request_ready:
            with self._lock:
                taken_data = self._handle.take_result_request(
                    self._action_type.Impl.GetResultService.Request,
                )
                # If take fails, then we get (None, None)
                if all(taken_data):
                    data['result'] = taken_data

        if self._is_goal_expired:
            with self._lock:
                data['expired'] = self._handle.expire_goals(len(self._goal_handles))

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
        num_entities = self._handle.get_num_entities()
        return NumberOfEntities(
            num_entities[0],
            num_entities[1],
            num_entities[2],
            num_entities[3],
            num_entities[4])

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        with self._lock:
            self._handle.add_to_waitset(wait_set)

    def __enter__(self):
        return self._handle.__enter__()

    def __exit__(self, t, v, tb):
        self._handle.__exit__(t, v, tb)

    # End Waitable API

    def notify_execute(self, goal_handle, execute_callback):
        # Use provided callback, defaulting to a previously registered callback
        if execute_callback is None:
            if self._execute_callback is None:
                return
            execute_callback = self._execute_callback

        # Schedule user callback for execution
        self._node.executor.create_task(self._execute_goal, execute_callback, goal_handle)

    def notify_goal_done(self):
        with self._lock:
            self._handle.notify_goal_done()

    def register_handle_accepted_callback(self, handle_accepted_callback):
        """
        Register a callback for handling newly accepted goals.

        The provided function is called whenever a new goal has been accepted by this
        action server.
        The function should expect an instance of :class:`ServerGoalHandle` as an argument, which
        represents a handle to the goal that was accepted.
        The goal handle can be used to interact with the goal, e.g. publish feedback,
        update the status, or execute a deferred goal.

        There can only be one handle accepted callback per :class:`ActionServer`, therefore
        calling this function will replace any previously registered callback.

        :param goal_callback: Callback function, if `None`, then unregisters any previously
            registered callback.
        """
        if handle_accepted_callback is None:
            handle_accepted_callback = default_handle_accepted_callback
        self._handle_accepted_callback = handle_accepted_callback

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
        if goal_callback is None:
            goal_callback = default_goal_callback
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
        if cancel_callback is None:
            cancel_callback = default_cancel_callback
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

        :param execute_callback: Callback function. Must not be `None`.
        """
        if not callable(execute_callback):
            raise TypeError('Failed to register goal execution callback: not callable')
        self._execute_callback = execute_callback

    def destroy(self):
        """Destroy the underlying action server handle."""
        for goal_handle in self._goal_handles.values():
            goal_handle.destroy()

        """Remove the underlying result future."""
        for result_future in self._result_futures.values():
            self.remove_future(result_future)

        self._handle.destroy_when_not_in_use()
        self._node.remove_waitable(self)
