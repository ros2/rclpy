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

from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.utilities import get_default_context


# TODO(mikaelarguedas) create a Timer or ROSTimer once we can specify custom time sources
class WallTimer:

    def __init__(self, callback, callback_group, timer_period_ns, *, context=None):
        self._context = get_default_context() if context is None else context
        # TODO(sloretz) Allow passing clocks in via timer constructor
        self._clock = Clock(clock_type=ClockType.STEADY_TIME)
        [self.timer_handle, self.timer_pointer] = _rclpy.rclpy_create_timer(
            self._clock._clock_handle, self._context.handle, timer_period_ns)
        self.timer_period_ns = timer_period_ns
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False

    @property
    def clock(self):
        return self._clock

    @property
    def timer_period_ns(self):
        val = _rclpy.rclpy_get_timer_period(self.timer_handle)
        self._timer_period_ns = val
        return val

    @timer_period_ns.setter
    def timer_period_ns(self, value):
        val = int(value)
        _rclpy.rclpy_change_timer_period(self.timer_handle, val)
        self._timer_period_ns = val

    def is_ready(self):
        return _rclpy.rclpy_is_timer_ready(self.timer_handle)

    def is_canceled(self):
        return _rclpy.rclpy_is_timer_canceled(self.timer_handle)

    def cancel(self):
        _rclpy.rclpy_cancel_timer(self.timer_handle)

    def reset(self):
        _rclpy.rclpy_reset_timer(self.timer_handle)

    def time_since_last_call(self):
        return _rclpy.rclpy_time_since_last_call(self.timer_handle)

    def time_until_next_call(self):
        return _rclpy.rclpy_time_until_next_call(self.timer_handle)
