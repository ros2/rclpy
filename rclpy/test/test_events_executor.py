# Copyright 2024-2025 Brad Martin
# Copyright 2024 Merlin Labs, Inc.
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
import os
import typing
import unittest

import action_msgs.msg
import rclpy.action
import rclpy.clock_type
import rclpy.duration
import rclpy.event_handler
import rclpy.executors
import rclpy.experimental
import rclpy.node
import rclpy.qos
import rclpy.time
import rclpy.timer
import rosgraph_msgs.msg
import test_msgs.action
import test_msgs.msg
import test_msgs.srv


def _get_pub_sub_qos(transient_local: bool) -> rclpy.qos.QoSProfile:
    if not transient_local:
        return rclpy.qos.QoSProfile(history=rclpy.qos.HistoryPolicy.KEEP_ALL)
    # For test purposes we deliberately want a TRANSIENT_LOCAL QoS with KEEP_ALL
    # history.
    return rclpy.qos.QoSProfile(
        history=rclpy.qos.HistoryPolicy.KEEP_ALL,
        durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
    )


class SubTestNode(rclpy.node.Node):
    """Node to test subscriptions and subscription-related events."""

    def __init__(self, *, transient_local: bool = False) -> None:
        super().__init__('test_sub_node')
        self._new_pub_future: typing.Optional[rclpy.Future] = None
        self._received_future: typing.Optional[rclpy.Future] = None
        self._sub = self.create_subscription(
            test_msgs.msg.BasicTypes,
            # This node seems to get stale discovery data and then complain about QoS
            # changes if we reuse the same topic name.
            'test_topic' + ('_transient_local' if transient_local else ''),
            self._handle_sub,
            _get_pub_sub_qos(transient_local),
            event_callbacks=rclpy.event_handler.SubscriptionEventCallbacks(
                matched=self._handle_matched_sub
            ),
        )

    def drop_subscription(self) -> None:
        self.destroy_subscription(self._sub)

    def expect_pub_info(
        self,
    ) -> rclpy.Future:
        self._new_pub_future = rclpy.Future()
        return self._new_pub_future

    def expect_message(self) -> rclpy.Future:
        self._received_future = rclpy.Future()
        return self._received_future

    def _handle_sub(self, msg: test_msgs.msg.BasicTypes) -> None:
        if self._received_future is not None:
            future = self._received_future
            self._received_future = None
            future.set_result(msg)

    def _handle_matched_sub(self, info: rclpy.event_handler.QoSSubscriptionMatchedInfo) -> None:
        """Handle a new publisher being matched to our subscription."""
        if self._new_pub_future is not None:
            self._new_pub_future.set_result(info)
            self._new_pub_future = None


class PubTestNode(rclpy.node.Node):
    """Node to test publications and publication-related events."""

    def __init__(self, *, transient_local: bool = False) -> None:
        super().__init__('test_pub_node')
        self._new_sub_future: typing.Optional[rclpy.Future] = None
        self._pub = self.create_publisher(
            test_msgs.msg.BasicTypes,
            'test_topic' + ('_transient_local' if transient_local else ''),
            _get_pub_sub_qos(transient_local),
            event_callbacks=rclpy.event_handler.PublisherEventCallbacks(
                matched=self._handle_matched_pub
            ),
        )

    def expect_sub_info(self) -> rclpy.Future:
        self._new_sub_future = rclpy.Future()
        return self._new_sub_future

    def publish(self, value: float) -> None:
        self._pub.publish(test_msgs.msg.BasicTypes(float32_value=value))

    def _handle_matched_pub(self, info: rclpy.event_handler.QoSPublisherMatchedInfo) -> None:
        """Handle a new subscriber being matched to our publication."""
        if self._new_sub_future is not None:
            self._new_sub_future.set_result(info)
            self._new_sub_future = None


class ServiceServerTestNode(rclpy.node.Node):
    """Node to test service server-side operation."""

    def __init__(self) -> None:
        super().__init__('test_service_server_node')
        self._got_request_future: typing.Optional[rclpy.Future] = None
        self._pending_response: typing.Optional[test_msgs.srv.BasicTypes.Response] = None
        self.create_service(test_msgs.srv.BasicTypes, 'test_service', self._handle_request)

    def expect_request(self, success: bool, error_msg: str) -> rclpy.Future:
        """
        Expect an incoming request.

        The arguments are used to compose the response.
        """
        self._got_request_future = rclpy.Future()
        self._pending_response = test_msgs.srv.BasicTypes.Response(
            bool_value=success, string_value=error_msg
        )
        return self._got_request_future

    def _handle_request(
        self,
        req: test_msgs.srv.BasicTypes.Request,
        res: test_msgs.srv.BasicTypes.Response,
    ) -> test_msgs.srv.BasicTypes.Response:
        if self._got_request_future is not None:
            self._got_request_future.set_result(req)
            self._got_request_future = None
        if self._pending_response is not None:
            res = self._pending_response
            self._pending_response = None
        return res


class ServiceClientTestNode(rclpy.node.Node):
    """Node to test service client-side operation."""

    def __init__(self) -> None:
        super().__init__('test_service_client_node')
        self._client: rclpy.client.Client = self.create_client(
            test_msgs.srv.BasicTypes, 'test_service'
        )

    def issue_request(self, value: float) -> rclpy.Future:
        req = test_msgs.srv.BasicTypes.Request(float32_value=value)
        return self._client.call_async(req)


class TimerTestNode(rclpy.node.Node):
    """Node to test timer operation."""

    def __init__(
        self,
        index: int = 0,
        parameter_overrides: typing.Optional[list[rclpy.Parameter]] = None,
    ) -> None:
        super().__init__(f'test_timer{index}', parameter_overrides=parameter_overrides)
        self._timer_events = 0
        self._tick_future: typing.Optional[rclpy.Future] = None
        self._timer = self.create_timer(0.1, self._handle_timer)

    @property
    def timer_events(self) -> int:
        return self._timer_events

    def expect_tick(self) -> rclpy.Future:
        """Get future for an anticipated timer tick."""
        self._tick_future = rclpy.Future()
        return self._tick_future

    def _handle_timer(self) -> None:
        self._timer_events += 1
        if self._tick_future is not None:
            self._tick_future.set_result('foo')
            self._tick_future = None


class ClockPublisherNode(rclpy.node.Node):
    """Node to publish rostime clock updates."""

    def __init__(self) -> None:
        super().__init__('clock_node')
        self._now = rclpy.time.Time(clock_type=rclpy.clock_type.ClockType.ROS_TIME)
        self._pub = self.create_publisher(
            rosgraph_msgs.msg.Clock,
            '/clock',
            rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT),
        )

    def advance_time(self, millisec: int) -> None:
        self._now += rclpy.duration.Duration(nanoseconds=millisec * 1000000)
        self._pub.publish(rosgraph_msgs.msg.Clock(clock=self._now.to_msg()))

    @property
    def now(self) -> rclpy.time.Time:
        return self._now


class ActionServerTestNode(rclpy.node.Node):
    """Node to test action server-side operation."""

    def __init__(self) -> None:
        super().__init__(
            'test_action_server_node',
            parameter_overrides=[rclpy.Parameter('use_sim_time', value=True)],
        )
        self._got_goal_future: typing.Optional[rclpy.Future] = (
            None
        )
        self._srv = rclpy.action.ActionServer(
            self,
            test_msgs.action.Fibonacci,
            'test_action',
            self._handle_execute,
            handle_accepted_callback=self._handle_accepted,
            result_timeout=10,
        )
        self._goal_handle: typing.Optional[rclpy.action.server.ServerGoalHandle] = None
        self._sequence: list[int] = []

    def expect_goal(self) -> rclpy.Future:
        self._goal_handle = None
        self._got_goal_future = rclpy.Future()
        return self._got_goal_future

    def _handle_accepted(self, goal_handle: rclpy.action.server.ServerGoalHandle) -> None:
        self._goal_handle = goal_handle
        self._sequence = [0, 1]
        if self._got_goal_future is not None:
            self._got_goal_future.set_result(goal_handle.request)
            self._got_goal_future = None
        # Wait to finish until instructed by test

    def advance_feedback(self) -> typing.Optional[list[int]]:
        """
        Add an entry to the result in progress and sends a feedback message.

        Returns the current sequence in progress if incomplete, or None if the sequence
        is complete and it's time to complete the operation instead.

        """
        assert self._goal_handle is not None
        n = self._goal_handle.request.order + 1
        if len(self._sequence) < n:
            self._sequence.append(self._sequence[-2] + self._sequence[-1])
        if len(self._sequence) >= n:
            return None

        # FYI normally feedbacks would be sent from the execute handler, but we've tied
        # it to its own public method for testing
        fb = test_msgs.action.Fibonacci.Feedback()
        fb.sequence = self._sequence
        self._goal_handle.publish_feedback(fb)
        return self._sequence

    def execute(self) -> rclpy.action.server.ServerGoalHandle:
        """
        Completes the action in progress.

        Returns the handle to the goal executed.

        """
        handle = self._goal_handle
        self._goal_handle = None
        assert handle is not None
        handle.execute()
        return handle

    def _handle_execute(
        self, goal_handle: rclpy.action.server.ServerGoalHandle
    ) -> test_msgs.action.Fibonacci.Result:
        goal_handle.succeed()
        result = test_msgs.action.Fibonacci.Result()
        result.sequence = self._sequence
        return result


class ActionClientTestNode(rclpy.node.Node):
    """Node to test action client-side operation."""

    def __init__(self) -> None:
        super().__init__('test_action_client_node')
        self._client = rclpy.action.ActionClient(self, test_msgs.action.Fibonacci, 'test_action')
        self._feedback_future: typing.Optional[rclpy.Future] = None
        self._result_future: typing.Optional[rclpy.Future] = (
            None
        )

    def send_goal(self, order: int) -> rclpy.Future:
        """
        Send a new goal.

        The future will contain the goal handle when the goal submission response has
        been received.

        """
        self._client.wait_for_server()
        goal_ack_future = self._client.send_goal_async(
            test_msgs.action.Fibonacci.Goal(order=order),
            feedback_callback=self._handle_feedback,
        )
        goal_ack_future.add_done_callback(self._handle_goal_ack)
        return goal_ack_future

    def _handle_goal_ack(self, future: rclpy.Future) -> None:
        handle = future.result()
        assert handle is not None
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._handle_result_response)

    def expect_feedback(self) -> rclpy.Future:
        self._feedback_future = rclpy.Future()
        return self._feedback_future

    def _handle_feedback(
        self,
        # If this is a private 'Impl' detail, why is rclpy handing this out??
        fb_msg: test_msgs.action.Fibonacci.Impl.FeedbackMessage,
    ) -> None:
        if self._feedback_future is not None:
            self._feedback_future.set_result(fb_msg.feedback)
            self._feedback_future = None

    def expect_result(self) -> rclpy.Future:
        self._result_future = rclpy.Future()
        return self._result_future

    def _handle_result_response(self, future: rclpy.Future) -> None:
        response: typing.Optional[test_msgs.action.Fibonacci_GetResult_Response] = future.result()
        assert response is not None
        assert self._result_future is not None
        result: test_msgs.action.Fibonacci.Result = response.result
        self._result_future.set_result(result)
        self._result_future = None


# These two python types are both actually rmw_matched_status_t
rmw_matched_status_t = typing.Union[
    rclpy.event_handler.QoSSubscriptionMatchedInfo, rclpy.event_handler.QoSPublisherMatchedInfo
]


class TestEventsExecutor(unittest.TestCase):

    def setUp(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        # Prevent nodes under test from discovering other random stuff to talk to
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
        rclpy.init()

        self.executor = rclpy.experimental.EventsExecutor()

    def tearDown(self) -> None:
        rclpy.shutdown()

    def _expect_future_done(self, future: rclpy.Future) -> None:
        # Use a moderately long timeout with the expectation that we shouldn't often
        # need the whole duration.
        self.executor.spin_until_future_complete(future, 1.0)
        self.assertTrue(future.done())

    def _expect_future_not_done(self, future: rclpy.Future) -> None:
        # Use a short timeout to give the future some time to complete if we are going
        # to fail, but not very long because we'll be waiting the full duration every
        # time during successful tests.  It's ok if the timeout is a bit short and the
        # failure isn't 100% deterministic.
        self.executor.spin_until_future_complete(future, 0.2)
        self.assertFalse(future.done())

    def _spin_for(self, sec: float) -> None:
        """Spins the executor for the given number of realtime seconds."""
        # Note that this roundabout approach of waiting on a future that will never
        # finish with a timeout seems to be the only way with the rclpy.Executor API to
        # spin for a fixed time.
        self.executor.spin_until_future_complete(rclpy.Future(), sec)

    def _check_match_event_future(
        self,
        future: rclpy.Future,
        total_count: int,
        current_count: int,
    ) -> None:
        # NOTE: fastdds appears to be buggy and reports a change in total_count with
        # total_count_change staying zero.  cyclonedds works as expected.  Rather than
        # have this test be sensitive to which RMW is selected, let's just avoid testing
        # the change fields altogether.
        self._expect_future_done(future)
        info: typing.Optional[rmw_matched_status_t] = future.result()
        assert info is not None
        self.assertEqual(info.total_count, total_count)
        self.assertEqual(info.current_count, current_count)

    def _check_message_future(
        self, future: rclpy.Future, value: float
    ) -> None:
        self._expect_future_done(future)
        msg: typing.Optional[test_msgs.msg.BasicTypes] = future.result()
        assert msg is not None
        self.assertAlmostEqual(msg.float32_value, value, places=5)

    def _check_service_request_future(
        self, future: rclpy.Future, value: float
    ) -> None:
        self._expect_future_done(future)
        req: typing.Optional[test_msgs.srv.BasicTypes.Request] = future.result()
        assert req is not None
        self.assertAlmostEqual(req.float32_value, value, places=5)

    def _check_service_response_future(
        self,
        future: rclpy.Future,
        success: bool,
        error_msg: str,
    ) -> None:
        self._expect_future_done(future)
        res: typing.Optional[test_msgs.srv.BasicTypes.Response] = future.result()
        assert res is not None
        self.assertEqual(res.bool_value, success)
        self.assertEqual(res.string_value, error_msg)

    def test_pub_sub(self) -> None:
        sub_node = SubTestNode()
        new_pub_future = sub_node.expect_pub_info()
        received_future = sub_node.expect_message()
        self.executor.add_node(sub_node)

        # With subscriber node alone, should be no publisher or messages
        self._expect_future_not_done(new_pub_future)
        self.assertFalse(received_future.done())  # Already waited a bit

        pub_node = PubTestNode()
        new_sub_future = pub_node.expect_sub_info()
        self.executor.add_node(pub_node)

        # Publisher and subscriber should find each other but no messages should be
        # exchanged yet
        self._check_match_event_future(new_pub_future, 1, 1)
        new_pub_future = sub_node.expect_pub_info()
        self._check_match_event_future(new_sub_future, 1, 1)
        new_sub_future = pub_node.expect_sub_info()
        self._expect_future_not_done(received_future)

        # Send messages and make sure they're received.
        for i in range(300):
            pub_node.publish(0.1 * i)
            self._check_message_future(received_future, 0.1 * i)
            received_future = sub_node.expect_message()

        # Destroy the subscription, make sure the publisher is notified
        sub_node.drop_subscription()
        self._check_match_event_future(new_sub_future, 1, 0)
        new_sub_future = pub_node.expect_sub_info()

        # Publish another message to ensure all subscriber callbacks got cleaned up
        pub_node.publish(4.7)
        self._expect_future_not_done(new_pub_future)
        self.assertFalse(received_future.done())  # Already waited a bit

        # Delete the subscribing node entirely.  There should be no additional match activity and
        # still no subscriber callbacks.
        self.executor.remove_node(sub_node)
        sub_node.destroy_node()
        self._expect_future_not_done(new_sub_future)
        self.assertFalse(new_pub_future.done())  # Already waited a bit
        self.assertFalse(received_future.done())  # Already waited a bit

    def test_pub_sub_multi_message(self) -> None:
        # Creates a transient local publisher and queues multiple messages on it.  Then
        # creates a subscriber and makes sure all sent messages get delivered when it
        # comes up.
        pub_node = PubTestNode(transient_local=True)
        self.executor.add_node(pub_node)
        for i in range(5):
            pub_node.publish(0.1 * i)

        sub_node = SubTestNode(transient_local=True)
        received_future = sub_node.expect_message()
        received_messages: list[test_msgs.msg.BasicTypes] = []

        def handle_message(future: rclpy.Future) -> None:
            nonlocal received_future
            msg = future.result()
            assert msg is not None
            received_messages.append(msg)
            received_future = sub_node.expect_message()
            received_future.add_done_callback(handle_message)

        received_future.add_done_callback(handle_message)
        self._expect_future_not_done(received_future)
        self.executor.add_node(sub_node)
        while len(received_messages) < 5:
            self._expect_future_done(received_future)
        self.assertEqual(len(received_messages), 5)
        for i in range(5):
            self.assertAlmostEqual(received_messages[i].float32_value, 0.1 * i, places=5)
        self._expect_future_not_done(received_future)

        pub_node.publish(0.5)
        self._check_message_future(received_future, 0.5)

    def test_service(self) -> None:
        server_node = ServiceServerTestNode()
        got_request_future = server_node.expect_request(True, 'test response 0')
        self.executor.add_node(server_node)
        self._expect_future_not_done(got_request_future)

        client_node = ServiceClientTestNode()
        self.executor.add_node(client_node)
        self._expect_future_not_done(got_request_future)
        for i in range(300):
            got_response_future = client_node.issue_request(7.1)
            self._check_service_request_future(got_request_future, 7.1)
            got_request_future = server_node.expect_request(True, f'test response {i + 1}')
            self._check_service_response_future(got_response_future, True, f'test response {i}')

        # Destroy server node and retry issuing a request
        self.executor.remove_node(server_node)
        server_node.destroy_node()
        self._expect_future_not_done(got_request_future)
        got_response_future = client_node.issue_request(5.0)
        self._expect_future_not_done(got_request_future)
        self.assertFalse(got_response_future.done())  # Already waited a bit

    def test_timers(self) -> None:
        realtime_node = TimerTestNode(index=0)
        rostime_node = TimerTestNode(
            index=1, parameter_overrides=[rclpy.Parameter('use_sim_time', value=True)]
        )
        clock_node = ClockPublisherNode()
        for node in [realtime_node, rostime_node, clock_node]:
            self.executor.add_node(node)

        # Wait a bit, and make sure the realtime timer ticks, and the rostime one does
        # not.  Since this is based on wall time, be very flexible on tolerances here.
        self._spin_for(1.0)
        realtime_ticks = realtime_node.timer_events
        self.assertGreater(realtime_ticks, 1)
        self.assertLess(realtime_ticks, 50)
        self.assertEqual(rostime_node.timer_events, 0)

        # Manually tick the rostime timer by less than a full interval.
        rostime_tick_future = rostime_node.expect_tick()
        for _ in range(99):
            clock_node.advance_time(1)
        self._expect_future_not_done(rostime_tick_future)
        clock_node.advance_time(1)
        self._expect_future_done(rostime_tick_future)
        # Now tick by a bunch of full intervals.
        for _ in range(300):
            rostime_tick_future = rostime_node.expect_tick()
            clock_node.advance_time(100)
            self._expect_future_done(rostime_tick_future)

        # Ensure the realtime timer ticked much less than the rostime one.
        self.assertLess(realtime_node.timer_events, rostime_node.timer_events)

        # Create two timers with the same interval, both set to cancel the other from the callback.
        # Only one of the callbacks should be delivered, though we can't necessarily predict which
        # one.
        def handler():
            nonlocal count, timer1, timer2
            count += 1
            timer1.cancel()
            timer2.cancel()

        count = 0
        timer1 = rostime_node.create_timer(0.01, handler)
        timer2 = rostime_node.create_timer(0.01, handler)
        self._spin_for(0.0)
        self.assertEqual(count, 0)
        clock_node.advance_time(10)
        self._spin_for(0.0)
        self.assertEqual(count, 1)
        clock_node.advance_time(10)
        self._spin_for(0.0)
        self.assertEqual(count, 1)

    def test_action(self) -> None:
        clock_node = ClockPublisherNode()
        self.executor.add_node(clock_node)

        server_node = ActionServerTestNode()
        got_goal_future = server_node.expect_goal()
        self.executor.add_node(server_node)
        clock_node.advance_time(0)
        self._expect_future_not_done(got_goal_future)

        client_node = ActionClientTestNode()
        self.executor.add_node(client_node)
        self._expect_future_not_done(got_goal_future)
        for i in range(300):
            order = (i % 40) + 1  # Don't want sequence to get too big
            goal_acknowledged_future = client_node.send_goal(order)

            self._expect_future_done(got_goal_future)
            self._expect_future_done(goal_acknowledged_future)
            req: typing.Optional[test_msgs.action.Fibonacci.Goal] = got_goal_future.result()
            assert req is not None
            self.assertEqual(req.order, order)
            result_future = client_node.expect_result()

            while True:
                got_feedback_future = client_node.expect_feedback()
                seq = server_node.advance_feedback()
                if seq is None:
                    break
                self._expect_future_done(got_feedback_future)
                feedback = got_feedback_future.result()
                assert feedback is not None
                self.assertEqual(len(feedback.sequence), len(seq))

            last_handle = server_node.execute()
            self._expect_future_done(result_future)
            self.assertFalse(got_feedback_future.done())

            res: typing.Optional[test_msgs.action.Fibonacci.Result] = result_future.result()
            assert res is not None
            self.assertEqual(len(res.sequence), order + 1)

            got_goal_future = server_node.expect_goal()

        # Test completed goal expiration by time
        self.assertEqual(last_handle.status, action_msgs.msg.GoalStatus.STATUS_SUCCEEDED)
        clock_node.advance_time(9999)
        self._spin_for(0.2)
        self.assertEqual(last_handle.status, action_msgs.msg.GoalStatus.STATUS_SUCCEEDED)
        clock_node.advance_time(2)
        self._spin_for(0.2)
        self.assertEqual(last_handle.status, action_msgs.msg.GoalStatus.STATUS_UNKNOWN)

        # Destroy server node and retry issuing a goal
        self.executor.remove_node(server_node)
        server_node.destroy_node()
        self._expect_future_not_done(got_goal_future)
        goal_acknowledged_future = client_node.send_goal(5)
        self._expect_future_not_done(got_goal_future)
        self.assertFalse(goal_acknowledged_future.done())  # Already waited a bit


if __name__ == '__main__':
    unittest.main()
