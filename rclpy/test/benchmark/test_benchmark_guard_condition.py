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


ONE_THOUSAND = 1000


def test_one_thousand_guard_callbacks(benchmark):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('benchmark_many_gc_calls', context=context)
        num_calls = 0

        def cb():
            nonlocal num_calls
            num_calls += 1

        gc = node.create_guard_condition(cb)

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        def bm():
            nonlocal num_calls
            # Reset for each benchmark run
            num_calls = 0
            while num_calls < ONE_THOUSAND:
                gc.trigger()
                executor.spin_once(timeout_sec=0)
            return num_calls == ONE_THOUSAND

        assert benchmark(bm)

        executor.shutdown()
        node.destroy_guard_condition(gc)
        node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def test_one_thousand_guard_coroutine_callbacks(benchmark):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('benchmark_many_gc_calls', context=context)
        num_calls = 0

        async def coro():
            nonlocal num_calls
            num_calls += 1

        gc = node.create_guard_condition(coro)

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        def bm():
            nonlocal num_calls
            # Reset for each benchmark run
            num_calls = 0
            while num_calls < ONE_THOUSAND:
                gc.trigger()
                executor.spin_once(timeout_sec=0)
            return num_calls == ONE_THOUSAND

        assert benchmark(bm)

        executor.shutdown()
        node.destroy_guard_condition(gc)
        node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
