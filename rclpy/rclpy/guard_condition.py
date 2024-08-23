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

from typing import Callable, Optional

from rclpy.callback_groups import CallbackGroup
from rclpy.context import Context
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.utilities import get_default_context


class GuardCondition:

    def __init__(self, callback: Optional[Callable],
                 callback_group: Optional[CallbackGroup],
                 context: Optional[Context] = None) -> None:
        """
        Create a GuardCondition.

        .. warning:: Users should not create a guard condition with this constructor, instead they
           should call :meth:`.Node.create_guard_condition`.
        """
        self._context = get_default_context() if context is None else context

        if self._context.handle is None:
            raise RuntimeError('Context must be initialized a GuardCondition can be created')

        with self._context.handle:
            self.__gc = _rclpy.GuardCondition(self._context.handle)
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False
        # True when the executor sees this has been triggered but has not yet been handled
        self._executor_triggered = False

    def trigger(self) -> None:
        with self.__gc:
            self.__gc.trigger_guard_condition()

    @property
    def handle(self) -> _rclpy.GuardCondition:
        return self.__gc

    def destroy(self) -> None:
        """
        Destroy a container for a ROS guard condition.

        .. warning:: Users should not destroy a guard condition with this method, instead
           they should call :meth:`.Node.destroy_guard_condition`.
        """
        self.handle.destroy_when_not_in_use()
