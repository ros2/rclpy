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

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from test_msgs.msg import Empty as EmptyMsg


ONE_THOUSAND = 1000


def test_one_thousand_messages(benchmark):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('benchmark_pub_sub', context=context)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        pub = node.create_publisher(EmptyMsg, 'empty', qos)
        num_calls = 0

        def cb(_):
            nonlocal num_calls
            num_calls += 1
            # Send next message
            pub.publish(EmptyMsg())

        sub = node.create_subscription(EmptyMsg, 'empty', cb, qos)

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        # Wait for pub/sub to discover each other
        while not pub.get_subscription_count():
            executor.spin_once(timeout_sec=0.01)

        def bm():
            nonlocal num_calls
            # Reset for each benchmark run
            num_calls = 0
            # Prime the loop with a message
            pub.publish(EmptyMsg())
            while num_calls < ONE_THOUSAND:
                executor.spin_once(timeout_sec=0)

        benchmark(bm)

        executor.shutdown()
        node.destroy_publisher(pub)
        node.destroy_subscription(sub)
        node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
