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

import rclpy
from rclpy.action_client import ActionClient
from rclpy.executors import SingleThreadedExecutor

from test_msgs.action import Fibonacci

from unique_identifier_msgs.msg import UUID


# TODO(jacobperron) Reduce fudge once wait_for_service uses node graph events
TIME_FUDGE = 0.3


class MockActionServer():

    def __init__(self, node):
        self.goal_srv = node.create_service(
            Fibonacci.GoalRequestService, '/fibonacci/_action/send_goal', self.goal_callback)
        self.cancel_srv = node.create_service(
            Fibonacci.CancelGoalService, '/fibonacci/_action/cancel_goal', self.cancel_callback)
        self.result_srv = node.create_service(
            Fibonacci.GoalResultService, '/fibonacci/_action/get_result', self.result_callback)
        self.feedback_pub = node.create_publisher(
            Fibonacci.Feedback, '/fibonacci/_action/feedback')

    def goal_callback(self, request, response):
        response.accepted = True
        return response

    def cancel_callback(self, request, response):
        response.goals_canceling.append(request.goal_info)
        return response

    def result_callback(self, request, response):
        return response

    def publish_feedback(self, goal_id):
        feedback = Fibonacci.Feedback()
        feedback.action_goal_id = goal_id
        self.feedback_pub.publish(feedback)


class TestActionClient(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.node = rclpy.create_node('TestActionClient', context=cls.context)
        cls.mock_action_server = MockActionServer(cls.node)
        cls.feedback = None

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def feedback_callback(self, feedback):
        self.feedback = feedback

    def timed_spin(self, duration):
        start_time = time.time()
        current_time = start_time
        while (current_time - start_time) < duration:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            current_time = time.time()

    def test_constructor_defaults(self):
        # Defaults
        ac = ActionClient(self.node, Fibonacci, 'fibonacci')
        ac.destroy()

    def test_constructor_no_defaults(self):
        ac = ActionClient(
            self.node,
            Fibonacci,
            'fibonacci',
            goal_service_qos_profile=rclpy.qos.qos_profile_default,
            result_service_qos_profile=rclpy.qos.qos_profile_default,
            cancel_service_qos_profile=rclpy.qos.qos_profile_default,
            feedback_sub_qos_profile=rclpy.qos.qos_profile_default,
            status_sub_qos_profile=rclpy.qos.qos_profile_default
        )
        ac.destroy()

    def test_get_num_entities(self):
        ac = ActionClient(self.node, Fibonacci, 'fibonacci')
        num_entities = ac.get_num_entities()
        self.assertEqual(num_entities.num_subscriptions, 2)
        self.assertEqual(num_entities.num_guard_conditions, 0)
        self.assertEqual(num_entities.num_timers, 0)
        self.assertEqual(num_entities.num_clients, 3)
        self.assertEqual(num_entities.num_services, 0)

    def test_wait_for_server_nowait(self):
        ac = ActionClient(self.node, Fibonacci, 'not_fibonacci')
        try:
            start = time.monotonic()
            self.assertFalse(ac.wait_for_server(timeout_sec=0.0))
            end = time.monotonic()
            self.assertGreater(0.0, end - start - TIME_FUDGE)
            self.assertLess(0.0, end - start + TIME_FUDGE)
        finally:
            ac.destroy()

    def test_wait_for_server_timeout(self):
        ac = ActionClient(self.node, Fibonacci, 'not_fibonacci')
        try:
            start = time.monotonic()
            self.assertFalse(ac.wait_for_server(timeout_sec=2.0))
            end = time.monotonic()
            self.assertGreater(2.0, end - start - TIME_FUDGE)
            self.assertLess(2.0, end - start + TIME_FUDGE)
        finally:
            ac.destroy()

    def test_wait_for_server_exists(self):
        ac = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            start = time.monotonic()
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))
            end = time.monotonic()
            self.assertGreater(0.0, end - start - TIME_FUDGE)
            self.assertLess(0.0, end - start + TIME_FUDGE)
        finally:
            ac.destroy()

    def test_send_goal_async(self):
        ac = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))
            goal = Fibonacci.Goal()
            future = ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future, self.executor)
            self.assertTrue(future.done())
            goal_handle = future.result()
            self.assertTrue(goal_handle.accepted)
        finally:
            ac.destroy()

    def test_send_goal_async_with_feedback_after_goal(self):
        ac = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Send a goal and then publish feedback
            goal_uuid = UUID(uuid=uuid.uuid4().bytes)
            future = ac.send_goal_async(
                Fibonacci.Goal(),
                feedback_callback=self.feedback_callback,
                goal_uuid=goal_uuid)
            rclpy.spin_until_future_complete(self.node, future, self.executor)

            # Publish feedback after goal has been accepted
            self.mock_action_server.publish_feedback(goal_uuid)
            self.timed_spin(1.0)
            self.assertNotEqual(self.feedback, None)
        finally:
            ac.destroy()

    def test_send_goal_async_with_feedback_before_goal(self):
        ac = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Publish feedback before goal has been accepted
            goal_uuid = UUID(uuid=uuid.uuid4().bytes)
            self.mock_action_server.publish_feedback(goal_uuid)
            self.timed_spin(1.0)
            self.assertEqual(self.feedback, None)

            # Send a goal and then publish feedback
            future = ac.send_goal_async(
                Fibonacci.Goal(),
                feedback_callback=self.feedback_callback,
                goal_uuid=goal_uuid)
            rclpy.spin_until_future_complete(self.node, future, self.executor)

            # Check the feedback was not received
            self.assertEqual(self.feedback, None)
        finally:
            ac.destroy()

    def test_send_goal_async_with_feedback_for_another_goal(self):
        ac = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Send a goal and then publish feedback
            first_goal_uuid = UUID(uuid=uuid.uuid4().bytes)
            future = ac.send_goal_async(
                Fibonacci.Goal(),
                feedback_callback=self.feedback_callback,
                goal_uuid=first_goal_uuid)
            rclpy.spin_until_future_complete(self.node, future, self.executor)

            # Send another goal, but without a feedback callback
            second_goal_uuid = UUID(uuid=uuid.uuid4().bytes)
            future = ac.send_goal_async(
                Fibonacci.Goal(),
                goal_uuid=second_goal_uuid)
            rclpy.spin_until_future_complete(self.node, future, self.executor)

            # Publish feedback for the second goal
            self.mock_action_server.publish_feedback(second_goal_uuid)
            self.timed_spin(1.0)
            self.assertEqual(self.feedback, None)
            # Publish feedback for the first goal (with callback)
            self.mock_action_server.publish_feedback(first_goal_uuid)
            self.timed_spin(1.0)
            self.assertNotEqual(self.feedback, None)
        finally:
            ac.destroy()

    def test_send_goal_async_with_feedback_for_not_a_goal(self):
        ac = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Send a goal and then publish feedback
            goal_uuid = UUID(uuid=uuid.uuid4().bytes)
            future = ac.send_goal_async(
                Fibonacci.Goal(),
                feedback_callback=self.feedback_callback,
                goal_uuid=goal_uuid)
            rclpy.spin_until_future_complete(self.node, future, self.executor)

            # Publish feedback for a non-existent goal ID
            self.mock_action_server.publish_feedback(UUID(uuid=uuid.uuid4().bytes))
            self.timed_spin(1.0)
            self.assertEqual(self.feedback, None)
        finally:
            ac.destroy()

    def test_send_goal_async_no_server(self):
        ac = ActionClient(self.node, Fibonacci, 'not_fibonacci')
        try:
            goal = Fibonacci.Goal()
            future = ac.send_goal_async(goal)
            self.assertFalse(rclpy.spin_once(self.node, executor=self.executor, timeout_sec=2.0))
            self.assertFalse(future.done())
        finally:
            ac.destroy()


if __name__ == '__main__':
    unittest.main()
