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
import unittest
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from test_msgs.action import Fibonacci

from unique_identifier_msgs.msg import UUID


class MockActionClient():

    def __init__(self, node):
        self.reset()
        self.goal_srv = node.create_client(
            Fibonacci.GoalRequestService, '/fibonacci/_action/send_goal')
        self.cancel_srv = node.create_client(
            Fibonacci.CancelGoalService, '/fibonacci/_action/cancel_goal')
        self.result_srv = node.create_client(
            Fibonacci.GoalResultService, '/fibonacci/_action/get_result')
        self.feedback_sub = node.create_subscription(
            Fibonacci.Feedback, '/fibonacci/_action/feedback', self.feedback_callback)
        self.status_sub = node.create_subscription(
            Fibonacci.GoalStatusMessage, '/fibonacci/_action/status', self.status_callback)

    def reset(self):
        self.feedback_msg = None
        self.status_msg = None

    def feedback_callback(self, feedback_msg):
        self.feedback_msg = feedback_msg

    def status_callback(self, status_msg):
        self.status_msg = status_msg

    def send_goal(self, goal_msg):
        return self.goal_srv.call_async(goal_msg)

    def cancel_goal(self, cancel_msg):
        return self.cancel_srv.call_async(cancel_msg)

    def get_result(self, goal_uuid):
        result_request = Fibonacci.GoalResultService.Request()
        result_request.action_goal_id = goal_uuid
        return self.result_srv.call_async(result_request)


class TestActionServer(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.node = rclpy.create_node('TestActionServer', context=self.context)
        self.mock_action_client = MockActionClient(self.node)

    def tearDown(self):
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)

    def timed_spin(self, duration):
        start_time = time.time()
        current_time = start_time
        while (current_time - start_time) < duration:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            current_time = time.time()

    def execute_goal_callback(self, goal_handle):
        goal_handle.set_succeeded()
        return Fibonacci.Result()

    def test_constructor_defaults(self):
        # Defaults
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
        )
        action_server.destroy()

    def test_constructor_no_defaults(self):
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=lambda req: GoalResponse.REJECT,
            cancel_callback=lambda req: CancelResponse.REJECT,
            goal_service_qos_profile=rclpy.qos.qos_profile_default,
            result_service_qos_profile=rclpy.qos.qos_profile_default,
            cancel_service_qos_profile=rclpy.qos.qos_profile_default,
            feedback_pub_qos_profile=rclpy.qos.qos_profile_default,
            status_pub_qos_profile=rclpy.qos.qos_profile_default,
            result_timeout=300,
        )
        action_server.destroy()

    def test_get_num_entities(self):
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

    def test_single_goal_accept(self):
        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_order = 10

        def goal_callback(goal):
            nonlocal goal_uuid
            nonlocal goal_order
            self.assertEqual(goal.action_goal_id, goal_uuid)
            self.assertEqual(goal.order, goal_order)
            return GoalResponse.ACCEPT_AND_EXECUTE

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            goal_callback=goal_callback,
        )

        goal_msg = Fibonacci.Goal()
        goal_msg.action_goal_id = goal_uuid
        goal_msg.order = goal_order
        future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, self.executor)
        self.assertTrue(future.result().accepted)
        action_server.destroy()

    def test_single_goal_reject(self):
        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_order = 10

        def goal_callback(goal):
            nonlocal goal_uuid
            nonlocal goal_order
            self.assertEqual(goal.action_goal_id, goal_uuid)
            self.assertEqual(goal.order, goal_order)
            return GoalResponse.REJECT

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            goal_callback=goal_callback,
        )

        goal_msg = Fibonacci.Goal()
        goal_msg.action_goal_id = goal_uuid
        goal_msg.order = goal_order
        future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, self.executor)
        self.assertFalse(future.result().accepted)
        action_server.destroy()

    def test_multi_goal_accept(self):
        executor = MultiThreadedExecutor(context=self.context)
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        goal_msg = Fibonacci.Goal()
        goal_msg.action_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        future0 = self.mock_action_client.send_goal(goal_msg)
        goal_msg.action_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        future1 = self.mock_action_client.send_goal(goal_msg)
        goal_msg.action_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        future2 = self.mock_action_client.send_goal(goal_msg)

        rclpy.spin_until_future_complete(self.node, future0, executor)
        rclpy.spin_until_future_complete(self.node, future1, executor)
        rclpy.spin_until_future_complete(self.node, future2, executor)

        self.assertTrue(future0.result().accepted)
        self.assertTrue(future1.result().accepted)
        self.assertTrue(future2.result().accepted)
        action_server.destroy()

    def test_duplicate_goal(self):
        executor = MultiThreadedExecutor(context=self.context)
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # Send a goal with the same ID twice
        goal_msg = Fibonacci.Goal()
        goal_msg.action_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        future0 = self.mock_action_client.send_goal(goal_msg)
        future1 = self.mock_action_client.send_goal(goal_msg)

        rclpy.spin_until_future_complete(self.node, future0, executor)
        rclpy.spin_until_future_complete(self.node, future1, executor)

        # Exactly one of the goals should be accepted
        self.assertNotEqual(future0.result().accepted, future1.result().accepted)
        action_server.destroy()

    def test_cancel_goal_accept(self):

        def execute_callback(goal_handle):
            # Wait, to give the opportunity to cancel
            time.sleep(3.0)
            self.assertTrue(goal_handle.is_cancel_requested)
            goal_handle.set_canceled()
            return Fibonacci.Result()

        def cancel_callback(request):
            return CancelResponse.ACCEPT

        executor = MultiThreadedExecutor(context=self.context)
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            callback_group=ReentrantCallbackGroup(),
            execute_callback=execute_callback,
            cancel_callback=cancel_callback,
        )

        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_order = 10
        goal_msg = Fibonacci.Goal()
        goal_msg.action_goal_id = goal_uuid
        goal_msg.order = goal_order
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, executor)
        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted)

        cancel_srv = CancelGoal.Request()
        cancel_srv.goal_info.goal_id = goal_uuid
        cancel_srv.goal_info.stamp.sec = 0
        cancel_srv.goal_info.stamp.nanosec = 0
        cancel_future = self.mock_action_client.cancel_goal(cancel_srv)
        rclpy.spin_until_future_complete(self.node, cancel_future, executor)
        cancel_result = cancel_future.result()
        self.assertEqual(len(cancel_result.goals_canceling), 1)
        self.assertEqual(cancel_result.goals_canceling[0].goal_id.uuid, goal_uuid.uuid)

        action_server.destroy()
        executor.shutdown()

    def test_cancel_goal_reject(self):

        def execute_callback(goal_handle):
            # Wait, to give the opportunity to cancel
            time.sleep(3.0)
            self.assertFalse(goal_handle.is_cancel_requested)
            goal_handle.set_canceled()
            return Fibonacci.Result()

        def cancel_callback(request):
            return CancelResponse.REJECT

        executor = MultiThreadedExecutor(context=self.context)
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            callback_group=ReentrantCallbackGroup(),
            execute_callback=execute_callback,
            cancel_callback=cancel_callback,
        )

        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_order = 10
        goal_msg = Fibonacci.Goal()
        goal_msg.action_goal_id = goal_uuid
        goal_msg.order = goal_order
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, executor)
        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted)

        cancel_srv = CancelGoal.Request()
        cancel_srv.goal_info.goal_id = goal_uuid
        cancel_srv.goal_info.stamp.sec = 0
        cancel_srv.goal_info.stamp.nanosec = 0
        cancel_future = self.mock_action_client.cancel_goal(cancel_srv)
        rclpy.spin_until_future_complete(self.node, cancel_future, executor)
        cancel_result = cancel_future.result()
        self.assertEqual(len(cancel_result.goals_canceling), 0)

        action_server.destroy()
        executor.shutdown()

    def test_send_result_succeeded(self):

        def execute_callback(goal_handle):
            result = Fibonacci.Result()
            result.sequence.extend([1, 1, 2, 3, 5])
            goal_handle.set_succeeded()
            return result

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=execute_callback,
        )

        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_msg = Fibonacci.Goal()
        goal_msg.action_goal_id = goal_uuid
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted)

        get_result_future = self.mock_action_client.get_result(goal_uuid)
        rclpy.spin_until_future_complete(self.node, get_result_future, self.executor)
        result = get_result_future.result()
        self.assertEqual(result.action_status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.sequence, [1, 1, 2, 3, 5])
        action_server.destroy()

    def test_send_result_aborted(self):

        def execute_callback(goal_handle):
            result = Fibonacci.Result()
            result.sequence.extend([1, 1, 2, 3, 5])
            goal_handle.set_aborted()
            return result

        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=execute_callback,
        )

        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        goal_msg = Fibonacci.Goal()
        goal_msg.action_goal_id = goal_uuid
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted)

        get_result_future = self.mock_action_client.get_result(goal_uuid)
        rclpy.spin_until_future_complete(self.node, get_result_future, self.executor)
        result = get_result_future.result()
        self.assertEqual(result.action_status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(result.sequence, [1, 1, 2, 3, 5])
        action_server.destroy()

    def test_expire_goals_none(self):

        # 1 second timeout
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            result_timeout=1,
        )

        goal_msg = Fibonacci.Goal()
        goal_msg.action_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)

        self.assertEqual(1, len(action_server._goal_handles))

        # After less than one second there should still be a goal handle
        self.timed_spin(0.5)
        self.assertEqual(1, len(action_server._goal_handles))
        action_server.destroy()

    def test_expire_goals_one(self):

        # 1 second timeout
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            result_timeout=1,
        )

        goal_msg = Fibonacci.Goal()
        goal_msg.action_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        goal_future = self.mock_action_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, self.executor)

        self.assertEqual(1, len(action_server._goal_handles))

        # After two seconds the internal goal handle should be destroyed
        self.timed_spin(2.1)
        self.assertEqual(0, len(action_server._goal_handles))
        action_server.destroy()

    def test_expire_goals_multi(self):
        # 1 second timeout
        action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_goal_callback,
            result_timeout=1,
        )

        # Send multiple goals
        goal_msg = Fibonacci.Goal()
        goal_msg.action_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        self.mock_action_client.send_goal(goal_msg)
        goal_msg.action_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        self.mock_action_client.send_goal(goal_msg)
        goal_msg.action_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        self.mock_action_client.send_goal(goal_msg)
        self.timed_spin(0.5)

        self.assertEqual(3, len(action_server._goal_handles))

        # After two seconds the internal goal handles should be destroyed
        self.timed_spin(2.1)
        self.assertEqual(0, len(action_server._goal_handles))
        action_server.destroy()


if __name__ == '__main__':
    unittest.main()
