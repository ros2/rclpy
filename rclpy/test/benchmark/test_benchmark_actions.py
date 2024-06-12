# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_services_default, ReliabilityPolicy
from test_msgs.action import Fibonacci


ONE_HUNDRED = 100


def test_one_hundred_goals(benchmark):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('benchmark_actions', context=context)
        qos = qos_profile_services_default
        qos.reliability = ReliabilityPolicy.RELIABLE

        action_client = ActionClient(
            node,
            Fibonacci,
            'benchmark',
            goal_service_qos_profile=qos,
            result_service_qos_profile=qos)

        def execute_cb(goal_handle):
            goal_handle.succeed()
            return Fibonacci.Result()

        action_server = ActionServer(
            node,
            Fibonacci,
            'benchmark',
            execute_cb,
            goal_service_qos_profile=qos,
            result_service_qos_profile=qos)

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        # Wait for client/server to discover each other
        assert action_client.wait_for_server(timeout_sec=5.0)

        def bm():
            for _ in range(ONE_HUNDRED):
                goal_fut = action_client.send_goal_async(Fibonacci.Goal())
                executor.spin_until_future_complete(goal_fut)
                client_goal_handle = goal_fut.result()
                assert client_goal_handle.accepted
                result_fut = client_goal_handle.get_result_async()
                executor.spin_until_future_complete(result_fut)
                assert GoalStatus.STATUS_SUCCEEDED == result_fut.result().status

        benchmark(bm)

        executor.shutdown()
        action_client.destroy()
        action_server.destroy()
        node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
