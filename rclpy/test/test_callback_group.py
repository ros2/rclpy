# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from test_msgs.msg import BasicTypes, Empty


class TestCallbackGroup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('TestCallbackGroup', namespace='/rclpy', context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_reentrant_group(self):
        self.assertIsNotNone(self.node.handle)
        group = ReentrantCallbackGroup()
        t1 = self.node.create_timer(1.0, lambda: None, callback_group=group)
        t2 = self.node.create_timer(1.0, lambda: None, callback_group=group)

        self.assertTrue(group.can_execute(t1))
        self.assertTrue(group.can_execute(t2))
        self.assertTrue(group.beginning_execution(t1))
        self.assertTrue(group.beginning_execution(t2))

    def test_reentrant_group_not_blocking(self):
        self.assertIsNotNone(self.node.handle)
        # Create multithreaded executor needed for parallel callback handling
        executor = MultiThreadedExecutor(num_threads=2, context=self.context)
        executor.add_node(self.node)
        try:
            # Setup flags for different scopes,
            # which indicate that the short callback has been called
            got_short_callback = False
            received_short_callback_in_long_callback = False

            # Setup two future objects that control the executor
            future_up = Future()
            future_down = Future()

            # This callback is used to check if a callback can be received while another
            # long running callback is being executed
            def short_callback(msg):
                nonlocal got_short_callback
                # Set flag so signal that the callback has been received
                got_short_callback = True

            # This callback is as a long running callback
            # It will be checking that the short callback can
            # run in parallel to this long running one
            def long_callback(msg):
                nonlocal received_short_callback_in_long_callback
                nonlocal future_up
                nonlocal future_down
                # The following future is used to delay the publishing of
                # the message that triggers the short callback.
                # This is done to ensure the long running callback is being executed
                # while the short callback is called
                future_up.set_result(None)
                # Wait for a maximum of 5 seconds
                # The short callback needs to be called in this time window
                for i in range(50):
                    time.sleep(0.1)
                    # Check if the short callback was called
                    if got_short_callback:
                        # Set a flag to signal that the short callback
                        # was executed during the long one
                        received_short_callback_in_long_callback = True
                        # Skip the rest of the waiting
                        break
                # Stop the executor from running any longer as there is nothing left to do
                future_down.set_result(None)

            # Create ReentrantCallbackGroup which is needed in combination with the
            # MultiThreadedExecutor to run callbacks not mutually exclusive
            group = ReentrantCallbackGroup()
            # Create subscriptions to trigger the callbacks
            self.node.create_subscription(
                Empty,
                'trigger_long',
                long_callback,
                1,
                callback_group=group)
            self.node.create_subscription(
                Empty,
                'trigger_short',
                short_callback,
                1,
                callback_group=group)
            # Create publishers to trigger both callbacks
            pub_trigger_long = self.node.create_publisher(Empty, 'trigger_long', 1)
            pub_trigger_short = self.node.create_publisher(Empty, 'trigger_short', 1)
            # Start the long running callback
            pub_trigger_long.publish(Empty())
            # Spin until we are sure that the long running callback is running
            executor.spin_until_future_complete(future_up)
            # Publish the short callback
            pub_trigger_short.publish(Empty())
            # Wait until the long running callback ends
            # (due to the signal from the short one or a timeout)
            executor.spin_until_future_complete(future_down)
            # Check if we were able to receive the short callback during the long running one
            self.assertTrue(received_short_callback_in_long_callback)
        finally:
            executor.shutdown()

    def test_mutually_exclusive_group(self):
        self.assertIsNotNone(self.node.handle)
        group = MutuallyExclusiveCallbackGroup()
        t1 = self.node.create_timer(1.0, lambda: None, callback_group=group)
        t2 = self.node.create_timer(1.0, lambda: None, callback_group=group)

        self.assertTrue(group.can_execute(t1))
        self.assertTrue(group.can_execute(t2))

        self.assertTrue(group.beginning_execution(t1))
        self.assertFalse(group.can_execute(t2))
        self.assertFalse(group.beginning_execution(t2))

        group.ending_execution(t1)
        self.assertTrue(group.can_execute(t2))
        self.assertTrue(group.beginning_execution(t2))

    def test_create_timer_with_group(self):
        tmr1 = self.node.create_timer(1.0, lambda: None)
        group = ReentrantCallbackGroup()
        tmr2 = self.node.create_timer(1.0, lambda: None, callback_group=group)

        self.assertFalse(group.has_entity(tmr1))
        self.assertTrue(group.has_entity(tmr2))

    def test_create_subscription_with_group(self):
        sub1 = self.node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg), 1)
        group = ReentrantCallbackGroup()
        sub2 = self.node.create_subscription(
            BasicTypes, 'chatter', lambda msg: print(msg), 1, callback_group=group)

        self.assertFalse(group.has_entity(sub1))
        self.assertTrue(group.has_entity(sub2))

    def test_create_client_with_group(self):
        cli1 = self.node.create_client(GetParameters, 'get/parameters')
        group = ReentrantCallbackGroup()
        cli2 = self.node.create_client(GetParameters, 'get/parameters', callback_group=group)

        self.assertFalse(group.has_entity(cli1))
        self.assertTrue(group.has_entity(cli2))

    def test_create_service_with_group(self):
        srv1 = self.node.create_service(GetParameters, 'get/parameters', lambda req: None)
        group = ReentrantCallbackGroup()
        srv2 = self.node.create_service(
            GetParameters, 'get/parameters', lambda req: None, callback_group=group)

        self.assertFalse(group.has_entity(srv1))
        self.assertTrue(group.has_entity(srv2))


if __name__ == '__main__':
    unittest.main()
