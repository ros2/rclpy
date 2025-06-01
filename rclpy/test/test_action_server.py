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

import time
from typing import Any
from typing import List
from typing import TYPE_CHECKING
import unittest
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
import rclpy.context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.service_introspection import ServiceIntrospectionState
from rclpy.task import Future

from service_msgs.msg import ServiceEventInfo

from test_msgs.action import Fibonacci

from unique_identifier_msgs.msg import UUID


class MockActionClient:

    def __init__(self, node: Node):
        self.reset()
        self.goal_srv: Client[Fibonacci.Impl.SendGoalService.Request,
                              Fibonacci.Impl.SendGoalService.Response] = node.create_client(
            Fibonacci.Impl.SendGoalService, '/fibonacci/_action/send_goal')
        self.cancel_srv: Client[Fibonacci.Impl.CancelGoalService.Request,
                                Fibonacci.Impl.CancelGoalService.Response] = node.create_client(
            Fibonacci.Impl.CancelGoalService, '/fibonacci/_action/cancel_goal')
        self.result_srv: Client[Fibonacci.Impl.GetResultService.Request,
                                Fibonacci.Impl.GetResultService.Response] = node.create_client(
            Fibonacci.Impl.GetResultService, '/fibonacci/_action/get_result')
        self.feedback_sub = node.create_subscription(
            Fibonacci.Impl.FeedbackMessage,
            '/fibonacci/_action/feedback',
            self.feedback_callback,
            1)
        self.status_sub = node.create_subscription(
            Fibonacci.Impl.GoalStatusMessage, '/fibonacci/_action/status', self.status_callback, 1)

    def reset(self) -> None:
        self.feedback_msg = None
        self.status_msg = None

    def feedback_callback(self, feedback_msg: Fibonacci.Impl.FeedbackMessage) -> None:
        self.feedback_msg = feedback_msg

    def status_callback(self, status_msg: Fibonacci.Impl.GoalStatusMessage) -> None:
        self.status_msg = status_msg

    def send_goal(self, goal_msg: Fibonacci.Impl.SendGoalService.Request
                  ) -> Future[Fibonacci.Impl.SendGoalService.Response]:
        return self.goal_srv.call_async(goal_msg)

    def cancel_goal(self, cancel_msg: Fibonacci.Impl.CancelGoalService.Request
                    ) -> Future[Fibonacci.Impl.CancelGoalService.Response]:
        return self.cancel_srv.call_async(cancel_msg)

    def get_result(self, goal_uuid: UUID) -> Future[Fibonacci.Impl.GetResultService.Response]:
        result_request = Fibonacci.Impl.GetResultService.Request()
        result_request.goal_id = goal_uuid
        return self.result_srv.call_async(result_request)


class TestActionServer(unittest.TestCase):

    if TYPE_CHECKING:
        context: rclpy.context.Context
        executor: SingleThreadedExecutor
        node: rclpy.node.Node
        mock_action_client: MockActionClient

    def setUp(self) -> None:
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.node = rclpy.create_node('TestActionServer', context=self.context)
        self.mock_action_client = MockActionClient(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)

    def timed_spin(self, duration: float) -> None:
        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

    def execute_goal_callback(self, goal_handle: ServerGoalHandle[Any, Fibonacci.Result, Any]
                              ) -> Fibonacci.Result:
        goal_handle.succeed()
        return Fibonacci.Result()

    def test_constructor_defaults(self) -> None:
        # Defaults
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
        )
        action_server.destroy()

    def test_constructor_no_defaults(self) -> None:
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=lambda req: GoalResponse.REJECT,
            handle_accepted_callback=lambda gh: None,
            cancel_callback=lambda req: CancelResponse.REJECT,
            goal_service_qos_profile=rclpy.qos.QoSProfile(depth=10),
            result_service_qos_profile=rclpy.qos.QoSProfile(depth=10),
            cancel_service_qos_profile=rclpy.qos.QoSProfile(depth=10),
            feedback_pub_qos_profile=rclpy.qos.QoSProfile(depth=10),
            status_pub_qos_profile=rclpy.qos.QoSProfile(depth=10),
            result_timeout=300,
        )
        action_server.destroy()

    def test_get_num_entities(self) -> None:
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
        )
        num_entities = action_server.get_num_entities()
        self.assertEqual(num_entities.num_subscriptions, 0)
        self.assertEqual(num_entities.num_guard_conditions, 0)
        self.assertEqual(num_entities.num_timers, 1)
        self.assertEqual(num_entities.num_clients, 0)
        self.assertEqual(num_entities.num_services, 3)
        action_server.destroy()

    def test_single_goal_accept(self) -> None:
        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_order = 10

        def goal_callback(goal: CancelGoal.Request) -> GoalResponse:
            nonlocal goal_order
            self.assertEqual(goal.order, goal_order)
            return GoalResponse.ACCEPT

        handle_accepted_callback_triggered = False

        def handle_accepted_callback(goal_handle: ServerGoalHandle[Fibonacci.Goal,
                                                                   Fibonacci.Result,
                                                                   Fibonacci.Feedback]) -> None:
            nonlocal goal_order
            nonlocal goal_uuid
            nonlocal handle_accepted_callback_triggered
            handle_accepted_callback_triggered = True
            self.assertEqual(goal_handle.status, GoalStatus.STATUS_ACCEPTED)
            self.assertEqual(goal_handle.goal_id, goal_uuid)
            self.assertEqual(goal_handle.request.order, goal_order)

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            goal_callback=goal_callback,
            handle_accepted_callback=handle_accepted_callback,
        )

        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = goal_uuid
        goal_msg.goal.order = goal_order
        future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, self.executor)
        future_result = future.result()
        assert future_result
        self.assertTrue(future_result.accepted)
        self.assertTrue(handle_accepted_callback_triggered)
        action_server.destroy()

    def test_single_goal_reject(self) -> None:
        goal_order = 10

        def goal_callback(goal: CancelGoal.Request) -> GoalResponse:
            nonlocal goal_order
            self.assertEqual(goal.order, goal_order)
            return GoalResponse.REJECT

        def handle_accepted_callback(goal_handle: ServerGoalHandle[Any, Any, Any]) -> None:
            # Since the goal is rejected, we don't expect this function to be called
            self.assertFalse(True)

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            goal_callback=goal_callback,
            handle_accepted_callback=handle_accepted_callback,
        )

        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        goal_msg.goal.order = goal_order
        future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, self.executor)
        result = future.result()
        assert result
        self.assertFalse(result.accepted)
        action_server.destroy()

    def test_goal_callback_invalid_return(self) -> None:

        def goal_callback(goal: CancelGoal.Request) -> str:
            return 'Invalid return type'

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            goal_callback=goal_callback,  # type: ignore[arg-type]
            handle_accepted_callback=lambda gh: None,
        )

        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, self.executor)
        # An invalid return type in the goal callback should translate to a rejected goal
        result = future.result()
        assert result
        self.assertFalse(result.accepted)
        action_server.destroy()

    def test_multi_goal_accept(self) -> None:
        executor = MultiThreadedExecutor(context=self.context)
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            callback_group=ReentrantCallbackGroup(),
            handle_accepted_callback=lambda gh: None,
        )

        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        future0 = self.mock_action_client.send_goal(goal_msg)
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        future1 = self.mock_action_client.send_goal(goal_msg)
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        future2 = self.mock_action_client.send_goal(goal_msg)

        rclpy.spin_until_future_complete(self.node, future0, executor)
        rclpy.spin_until_future_complete(self.node, future1, executor)
        rclpy.spin_until_future_complete(self.node, future2, executor)

        future0_result = future0.result()
        future1_result = future1.result()
        future2_result = future2.result()
        assert future0_result
        assert future1_result
        assert future2_result

        self.assertTrue(future0_result.accepted)
        self.assertTrue(future1_result.accepted)
        self.assertTrue(future2_result.accepted)
        action_server.destroy()

    def test_duplicate_goal(self) -> None:
        executor = MultiThreadedExecutor(context=self.context)
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            callback_group=ReentrantCallbackGroup(),
            handle_accepted_callback=lambda gh: None,
        )

        # Send a goal with the same ID twice
        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        future0 = self.mock_action_client.send_goal(goal_msg)
        future1 = self.mock_action_client.send_goal(goal_msg)

        rclpy.spin_until_future_complete(self.node, future0, executor)
        rclpy.spin_until_future_complete(self.node, future1, executor)

        # Exactly one of the goals should be accepted
        future0_result = future0.result()
        assert future0_result
        future1_result = future1.result()
        assert future1_result
        self.assertNotEqual(future0_result.accepted, future1_result.accepted)
        action_server.destroy()

    def test_cancel_goal_accept(self) -> None:

        def execute_callback(goal_handle: ServerGoalHandle[Any, Fibonacci.Result, Any]
                             ) -> Fibonacci.Result:
            # Wait, to give the opportunity to cancel
            time.sleep(3.0)
            self.assertTrue(goal_handle.is_cancel_requested)
            goal_handle.canceled()
            return Fibonacci.Result()

        def cancel_callback(request: CancelGoal.Request) -> CancelResponse:
            return CancelResponse.ACCEPT

        executor = MultiThreadedExecutor(context=self.context)
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            callback_group=ReentrantCallbackGroup(),
            execute_callback=execute_callback,
            handle_accepted_callback=lambda gh: None,
            cancel_callback=cancel_callback,
        )

        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_order = 10
        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = goal_uuid
        goal_msg.goal.order = goal_order
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, executor)
        goal_handle = goal_future.result()
        assert goal_handle
        self.assertTrue(goal_handle.accepted)

        cancel_srv = CancelGoal.Request()
        cancel_srv.goal_info.goal_id = goal_uuid
        cancel_srv.goal_info.stamp.sec = 0
        cancel_srv.goal_info.stamp.nanosec = 0
        cancel_future = self.mock_action_client.cancel_goal(cancel_srv)
        rclpy.spin_until_future_complete(self.node, cancel_future, executor)
        cancel_result = cancel_future.result()
        assert cancel_result
        self.assertEqual(len(cancel_result.goals_canceling), 1)
        assert all(cancel_result.goals_canceling[0].goal_id.uuid == goal_uuid.uuid)

        action_server.destroy()
        executor.shutdown()

    def test_cancel_goal_reject(self) -> None:

        def execute_callback(goal_handle: ServerGoalHandle[Any, Fibonacci.Feedback, Any]
                             ) -> Fibonacci.Result:
            # Wait, to give the opportunity to cancel
            time.sleep(3.0)
            self.assertFalse(goal_handle.is_cancel_requested)
            goal_handle.canceled()
            return Fibonacci.Result()

        def cancel_callback(request: CancelGoal.Request) -> CancelResponse:
            return CancelResponse.REJECT

        executor = MultiThreadedExecutor(context=self.context)
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            callback_group=ReentrantCallbackGroup(),
            execute_callback=execute_callback,
            handle_accepted_callback=lambda gh: None,
            cancel_callback=cancel_callback,
        )

        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_order = 10
        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = goal_uuid
        goal_msg.goal.order = goal_order
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, executor)
        goal_handle = goal_future.result()
        assert goal_handle
        self.assertTrue(goal_handle.accepted)

        cancel_srv = CancelGoal.Request()
        cancel_srv.goal_info.goal_id = goal_uuid
        cancel_srv.goal_info.stamp.sec = 0
        cancel_srv.goal_info.stamp.nanosec = 0
        cancel_future = self.mock_action_client.cancel_goal(cancel_srv)
        rclpy.spin_until_future_complete(self.node, cancel_future, executor)
        cancel_result = cancel_future.result()
        assert cancel_result
        self.assertEqual(len(cancel_result.goals_canceling), 0)

        action_server.destroy()
        executor.shutdown()

    def test_cancel_defered_goal(self) -> None:
        server_goal_handle = None

        def handle_accepted_callback(gh: ServerGoalHandle[Fibonacci.Goal,
                                                          Fibonacci.Result,
                                                          Fibonacci.Feedback]) -> None:
            nonlocal server_goal_handle
            server_goal_handle = gh

        def cancel_callback(request: CancelGoal.Request) -> CancelResponse:
            return CancelResponse.ACCEPT

        def execute_callback(gh: ServerGoalHandle[Fibonacci.Goal,
                                                  Fibonacci.Result,
                                                  Fibonacci.Feedback]
                             ) -> Fibonacci.Result:
            # The goal should already be in state CANCELING
            self.assertTrue(gh.is_cancel_requested)
            gh.canceled()
            return Fibonacci.Result()

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            callback_group=ReentrantCallbackGroup(),
            execute_callback=execute_callback,
            handle_accepted_callback=handle_accepted_callback,
            cancel_callback=cancel_callback,
        )

        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = goal_uuid
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
        send_goal_response = goal_future.result()
        assert send_goal_response
        self.assertTrue(send_goal_response.accepted)
        assert server_goal_handle
        self.assertIsNotNone(server_goal_handle)
        self.assertEqual(server_goal_handle.status, GoalStatus.STATUS_ACCEPTED)

        # Cancel the goal, before execution
        cancel_srv = CancelGoal.Request()
        cancel_srv.goal_info.goal_id = goal_uuid
        cancel_srv.goal_info.stamp.sec = 0
        cancel_srv.goal_info.stamp.nanosec = 0
        cancel_future = self.mock_action_client.cancel_goal(cancel_srv)
        rclpy.spin_until_future_complete(self.node, cancel_future, self.executor)
        cancel_result = cancel_future.result()
        assert cancel_result
        self.assertEqual(len(cancel_result.goals_canceling), 1)

        self.assertEqual(server_goal_handle.status, GoalStatus.STATUS_CANCELING)

        # Execute the goal
        server_goal_handle.execute()

        # Get the result and expect it to have canceled status
        get_result_future = self.mock_action_client.get_result(goal_uuid)
        rclpy.spin_until_future_complete(self.node, get_result_future, self.executor)
        result = get_result_future.result()
        assert result
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(server_goal_handle.status, GoalStatus.STATUS_CANCELED)
        action_server.destroy()

    def test_execute_succeed(self) -> None:

        def execute_callback(goal_handle: ServerGoalHandle[Fibonacci.Goal,
                                                           Fibonacci.Result,
                                                           Fibonacci.Feedback]
                             ) -> Fibonacci.Result:
            self.assertEqual(goal_handle.status, GoalStatus.STATUS_EXECUTING)
            result = Fibonacci.Result()
            result.sequence.extend([1, 1, 2, 3, 5])
            goal_handle.succeed()
            return result

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=execute_callback,
        )

        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = goal_uuid
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
        goal_handle = goal_future.result()
        assert goal_handle
        self.assertTrue(goal_handle.accepted)

        get_result_future = self.mock_action_client.get_result(goal_uuid)
        rclpy.spin_until_future_complete(self.node, get_result_future, self.executor)
        result_response = get_result_future.result()
        assert result_response
        self.assertEqual(result_response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result_response.result.sequence.tolist(), [1, 1, 2, 3, 5])
        action_server.destroy()

    def test_execute_abort(self) -> None:

        def execute_callback(goal_handle: ServerGoalHandle[Fibonacci.Goal,
                                                           Fibonacci.Result,
                                                           Fibonacci.Feedback]
                             ) -> Fibonacci.Result:
            self.assertEqual(goal_handle.status, GoalStatus.STATUS_EXECUTING)
            result = Fibonacci.Result()
            result.sequence.extend([1, 1, 2, 3, 5])
            goal_handle.abort()
            return result

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=execute_callback,
        )

        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = goal_uuid
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
        goal_handle = goal_future.result()
        assert goal_handle
        self.assertTrue(goal_handle.accepted)

        get_result_future = self.mock_action_client.get_result(goal_uuid)
        rclpy.spin_until_future_complete(self.node, get_result_future, self.executor)
        result_response = get_result_future.result()
        assert result_response
        self.assertEqual(result_response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(result_response.result.sequence.tolist(), [1, 1, 2, 3, 5])
        action_server.destroy()

    def test_execute_no_terminal_state(self) -> None:

        def execute_callback(goal_handle: ServerGoalHandle[Fibonacci.Goal,
                                                           Fibonacci.Result,
                                                           Fibonacci.Feedback]
                             ) -> Fibonacci.Result:
            # Do not set the goal handles state
            result = Fibonacci.Result()
            result.sequence.extend([1, 1, 2, 3, 5])
            return result

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=execute_callback,
        )

        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = goal_uuid
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
        goal_handle = goal_future.result()
        assert goal_handle
        self.assertTrue(goal_handle.accepted)

        get_result_future = self.mock_action_client.get_result(goal_uuid)
        rclpy.spin_until_future_complete(self.node, get_result_future, self.executor)
        result_response = get_result_future.result()
        # Goal status should default to STATUS_ABORTED
        assert result_response
        self.assertEqual(result_response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(result_response.result.sequence.tolist(), [1, 1, 2, 3, 5])
        action_server.destroy()

    def test_execute_raises_exception(self) -> None:

        def execute_callback(goal_handle: ServerGoalHandle[Fibonacci.Goal,
                                                           Fibonacci.Result,
                                                           Fibonacci.Feedback]
                             ) -> Fibonacci.Result:
            # User callback raises
            raise RuntimeError('test user callback raises')

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=execute_callback,
        )

        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = goal_uuid
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
        goal_handle = goal_future.result()
        assert goal_handle
        self.assertTrue(goal_handle.accepted)

        get_result_future = self.mock_action_client.get_result(goal_uuid)
        rclpy.spin_until_future_complete(self.node, get_result_future, self.executor)
        result_response = get_result_future.result()
        # Goal status should default to STATUS_ABORTED
        assert result_response
        self.assertEqual(result_response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(result_response.result.sequence.tolist(), [])
        action_server.destroy()

    def test_expire_goals_none(self) -> None:

        # 1 second timeout
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            result_timeout=1,
        )

        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)

        self.assertEqual(1, len(action_server._goal_handles))

        # After less than one second there should still be a goal handle
        self.timed_spin(0.5)
        self.assertEqual(1, len(action_server._goal_handles))
        action_server.destroy()

    def test_expire_goals_one(self) -> None:

        # 1 second timeout
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            result_timeout=1,
        )

        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)

        self.assertEqual(1, len(action_server._goal_handles))

        # After two seconds the internal goal handle should be destroyed
        self.timed_spin(2.1)
        self.assertEqual(0, len(action_server._goal_handles))
        action_server.destroy()

    def test_expire_goals_multi(self) -> None:
        # 1 second timeout
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            result_timeout=1,
        )

        # Send multiple goals
        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        self.mock_action_client.send_goal(goal_msg)
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        self.mock_action_client.send_goal(goal_msg)
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        self.mock_action_client.send_goal(goal_msg)
        self.timed_spin(0.5)

        self.assertEqual(3, len(action_server._goal_handles))

        # After two seconds the internal goal handles should be destroyed
        self.timed_spin(2.1)
        self.assertEqual(0, len(action_server._goal_handles))
        action_server.destroy()

    def test_feedback(self) -> None:

        def execute_with_feedback(goal_handle: ServerGoalHandle[Any, Fibonacci.Feedback, Any]
                                  ) -> Fibonacci.Result:
            feedback = Fibonacci.Feedback()
            feedback.sequence = [1, 1, 2, 3]
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            return Fibonacci.Result()

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=execute_with_feedback,
        )

        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        goal_future = self.mock_action_client.send_goal(goal_msg)

        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)

        self.assertIsNotNone(self.mock_action_client.feedback_msg)
        assert self.mock_action_client.feedback_msg
        self.assertEqual(
            [1, 1, 2, 3],
            self.mock_action_client.feedback_msg.feedback.sequence.tolist())
        action_server.destroy()

    def test_different_feedback_type_raises(self) -> None:

        def execute_with_feedback(goal_handle: ServerGoalHandle[Any, Fibonacci.Result, Any]
                                  ) -> Fibonacci.Result:
            try:
                goal_handle.publish_feedback('different feedback type')  # type: ignore[arg-type]
            except TypeError:
                feedback = Fibonacci.Feedback()
                feedback.sequence = [1, 1, 2, 3]
                goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            return Fibonacci.Result()

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=execute_with_feedback,
        )

        try:
            goal_msg = Fibonacci.Impl.SendGoalService.Request()
            goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
            goal_future = self.mock_action_client.send_goal(goal_msg)

            rclpy.spin_until_future_complete(
                self.node, goal_future, self.executor)

            feedback_msg = self.mock_action_client.feedback_msg
            assert feedback_msg
            self.assertIsNotNone(feedback_msg)
            self.assertEqual(
                [1, 1, 2, 3], feedback_msg.feedback.sequence.tolist())
        finally:
            action_server.destroy()

    def test_action_introspection_default_status(self) -> None:
        goal_order = 10

        def goal_callback(goal: CancelGoal.Request) -> GoalResponse:
            nonlocal goal_order
            self.assertEqual(goal.order, goal_order)
            return GoalResponse.REJECT

        def handle_accepted_callback(goal_handle: ServerGoalHandle[Any, Any, Any]) -> None:
            # Since the goal is rejected, we don't expect this function to be called
            self.assertFalse(True)

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            goal_callback=goal_callback,
            handle_accepted_callback=handle_accepted_callback,
        )

        # There is no need to check if introspection is enabled for all internal services,
        # as the implementation in the RCL interface operates on the three internal services
        # simultaneously.

        self.event_messages: List[Fibonacci.Impl.SendGoalService.Event] = []

        def sub_callback(msg: Fibonacci.Impl.SendGoalService.Event) -> None:
            self.event_messages.append(msg)

        send_goal_service_event_sub = self.node.create_subscription(
            Fibonacci.Impl.SendGoalService.Event,
            '/fibonacci/_action/send_goal/_service_event',
            sub_callback, 3)

        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        goal_msg.goal.order = goal_order
        future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, self.executor)
        future_result = future.result()
        assert future_result
        self.assertFalse(future_result.accepted)

        # By default, action client introspection is disabled. So no service event message can
        # be received.
        start = time.monotonic()
        end = start + 1.0
        while len(self.event_messages) < 1:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            now = time.monotonic()
            if now >= end:
                break

        self.assertEqual(len(self.event_messages), 0)

        send_goal_service_event_sub.destroy()
        action_server.destroy()

    def test_configure_introspection_content(self) -> None:
        goal_order = 10

        def goal_callback(goal: CancelGoal.Request) -> GoalResponse:
            nonlocal goal_order
            self.assertEqual(goal.order, goal_order)
            return GoalResponse.REJECT

        def handle_accepted_callback(goal_handle: ServerGoalHandle[Any, Any, Any]) -> None:
            # Since the goal is rejected, we don't expect this function to be called
            self.assertFalse(True)

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            goal_callback=goal_callback,
            handle_accepted_callback=handle_accepted_callback,
        )

        action_server.configure_introspection(self.node.get_clock(), qos_profile_system_default,
                                              ServiceIntrospectionState.CONTENTS)

        # There is no need to check if introspection is enabled for all internal services,
        # as the implementation in the RCL interface operates on the three internal services
        # simultaneously.

        self.event_messages = []

        def sub_callback(msg: Fibonacci.Impl.SendGoalService.Event) -> None:
            self.event_messages.append(msg)

        send_goal_service_event_sub = self.node.create_subscription(
            Fibonacci.Impl.SendGoalService.Event,
            '/fibonacci/_action/send_goal/_service_event',
            sub_callback, 3)

        goal_msg = Fibonacci.Impl.SendGoalService.Request()
        goal_msg.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        goal_msg.goal.order = goal_order
        future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, self.executor)
        future_result = future.result()
        assert future_result
        self.assertFalse(future_result.accepted)

        start = time.monotonic()
        end = start + 5.0
        while len(self.event_messages) < 1:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            now = time.monotonic()
            self.assertTrue(now < end)
        self.assertEqual(len(self.event_messages), 1)

        self.assertEqual(self.event_messages[0].info.event_type, ServiceEventInfo.REQUEST_RECEIVED)

        # For ServiceIntrospectionState.CONTENTS mode, the request or response section must
        # contain data. In this case, the request section must contain data.
        self.assertEqual(len(self.event_messages[0].request), 1)
        self.assertEqual(len(self.event_messages[0].response), 0)

        send_goal_service_event_sub.destroy()
        action_server.destroy()


if __name__ == '__main__':
    unittest.main()
