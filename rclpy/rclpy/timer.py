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

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class Timer(object):

    def __init__(self, timer_handle, timer_pointer, callback, timer_period_ns):
        [self.timer_handle, self.timer_pointer] = _rclpy.rclpy_create_timer(timer_period_ns)
        self.timer_period_ns = timer_period_ns
        self.callback = callback

    @property
    def timer_period_ns(self):
        val = _rclpy.rclpy_get_timer_period(self.timer_handle)
        self._timer_period_ns = val
        return val

    @timer_period_ns.setter
    def timer_period_ns(self, value):
        print('changing timer.timer_period_ns')
        val = int(value)
        _rclpy.rclpy_change_timer_period(self.timer_handle, val)
        self._timer_period_ns = val
        print('done changing timer.timer_period_ns')

    def time_since_last_call(self):
        return _rclpy.rclpy_time_since_last_call(self.timer_handle)

    def time_until_next_call(self):
        return _rclpy.rclpy_time_until_next_call(self.timer_handle)
