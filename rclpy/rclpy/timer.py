# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from rclpy.handle import Handle
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.utilities import get_default_context


class Timer:

    def __init__(self, callback, callback_group, timer_period_ns, clock, *, context=None):
        self._context = get_default_context() if context is None else context
        self._clock = clock
        with self._clock.handle as clock_capsule:
            self.__handle = Handle(_rclpy.rclpy_create_timer(
                clock_capsule, self._context.handle, timer_period_ns))
        self.__handle.requires(self._clock.handle)
        self.timer_period_ns = timer_period_ns
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False

    @property
    def handle(self):
        return self.__handle

    def destroy(self):
        self.handle.destroy()

    @property
    def clock(self):
        return self._clock

    @property
    def timer_period_ns(self):
        with self.handle as capsule:
            val = _rclpy.rclpy_get_timer_period(capsule)
        self._timer_period_ns = val
        return val

    @timer_period_ns.setter
    def timer_period_ns(self, value):
        val = int(value)
        with self.handle as capsule:
            _rclpy.rclpy_change_timer_period(capsule, val)
        self._timer_period_ns = val

    def is_ready(self):
        with self.handle as capsule:
            return _rclpy.rclpy_is_timer_ready(capsule)

    def is_canceled(self):
        with self.handle as capsule:
            return _rclpy.rclpy_is_timer_canceled(capsule)

    def cancel(self):
        with self.handle as capsule:
            _rclpy.rclpy_cancel_timer(capsule)

    def reset(self):
        with self.handle as capsule:
            _rclpy.rclpy_reset_timer(capsule)

    def time_since_last_call(self):
        with self.handle as capsule:
            return _rclpy.rclpy_time_since_last_call(capsule)

    def time_until_next_call(self):
        with self.handle as capsule:
            return _rclpy.rclpy_time_until_next_call(capsule)
