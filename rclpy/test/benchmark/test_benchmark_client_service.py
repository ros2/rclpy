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
from rclpy.qos import qos_profile_services_default, ReliabilityPolicy
from test_msgs.srv import Empty as EmptySrv


ONE_THOUSAND = 1000


def test_one_thousand_service_calls(benchmark):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('benchmark_client_service', context=context)
        qos = qos_profile_services_default
        qos.reliability = ReliabilityPolicy.RELIABLE
        client = node.create_client(EmptySrv, 'empty', qos_profile=qos)

        def cb(_, response):
            return response

        service = node.create_service(EmptySrv, 'empty', cb, qos_profile=qos)

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        # Wait for client/service to discover each other
        assert client.wait_for_service(timeout_sec=5.0)

        def bm():
            for _ in range(ONE_THOUSAND):
                fut = client.call_async(EmptySrv.Request())
                executor.spin_until_future_complete(fut)
                assert fut.result()  # Raises if there was an error

        benchmark(bm)

        executor.shutdown()
        node.destroy_client(client)
        node.destroy_service(service)
        node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
