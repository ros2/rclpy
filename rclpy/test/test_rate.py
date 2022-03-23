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

import threading
import time

import pytest
import rclpy
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import SingleThreadedExecutor

# Hz
FREQ = 10.0
PERIOD = 1.0 / FREQ
PASS_MAX_AVERAGE_JITTER = PERIOD * 0.1
PASS_MAX_SINGLE_JITTER = PERIOD * 0.25


class RateRunner:

    def __init__(self, rate):
        self.avg_period = None
        self.max_jitter = None
        self.min_period = None
        self.max_period = None
        self.done = False

        self._num_measurements = 10
        self._thread = threading.Thread(target=self._run, args=(rate,), daemon=True)
        self._thread.start()

    def __str__(self):
        return 'avg period: {} max jitter: {} min: {} max: {} '.format(
            self.avg_period, self.max_jitter, self.min_period, self.max_period)

    def _run(self, rate):
        try:
            measurements = []
            # First sleep time depends on how long thread took to start, so ignore it
            rate.sleep()
            last_wake_time = time.monotonic()

            for i in range(self._num_measurements):
                rate.sleep()
                now = time.monotonic()
                measurements.append(now - last_wake_time)
                last_wake_time = now

            self.avg_period = sum(measurements) / len(measurements)
            self.max_jitter = max([abs(m - self.avg_period) for m in measurements])  # noqa: C407
            self.min_period = min(measurements)
            self.max_period = max(measurements)
        finally:
            self.done = True


class TestRate:

    def setup_method(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('test_rate', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def teardown_method(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_rate_valid_period(self):
        rate = self.node.create_rate(FREQ)

        runner = RateRunner(rate)
        while not runner.done:
            self.executor.spin_once()

        assert runner.max_jitter <= PASS_MAX_SINGLE_JITTER, str(runner)
        assert abs(runner.avg_period - PERIOD) <= PASS_MAX_AVERAGE_JITTER, str(runner)

    def test_rate_invalid_period(self):
        with pytest.raises(TypeError):
            self.node.create_rate(None)

        with pytest.raises(ValueError):
            self.node.create_rate(0.0)

        with pytest.raises(ValueError):
            self.node.create_rate(-1.0)

    def test_destroy(self):
        rate = self.node.create_rate(FREQ)
        assert self.node.destroy_rate(rate)

    def test_destroy_wakes_rate(self):
        rate = self.node.create_rate(0.0000001)

        self._thread = threading.Thread(target=rate.sleep, daemon=True)
        self._thread.start()
        assert self.node.destroy_rate(rate)
        self._thread.join()


def sleep_check_exception(rate):
    try:
        rate.sleep()
    except ROSInterruptException:
        # rate.sleep() can raise ROSInterruptException if the context is
        # shutdown while it is sleeping.  Just ignore it here.
        pass


def test_shutdown_wakes_rate():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_rate_shutdown', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)

    rate = node.create_rate(0.0000001)

    _thread = threading.Thread(target=sleep_check_exception, args=(rate,), daemon=True)
    _thread.start()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown(context=context)
    _thread.join()
