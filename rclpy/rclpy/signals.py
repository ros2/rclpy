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
from typing import Optional, TYPE_CHECKING

from rclpy.exceptions import InvalidHandle
from rclpy.guard_condition import GuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

if TYPE_CHECKING:
    from typing import TypeAlias

    from rclpy.context import Context

# re-export SignalHandlerOptions enum
SignalHandlerOptions: 'TypeAlias' = _rclpy.SignalHandlerOptions


def install_signal_handlers(options: SignalHandlerOptions = SignalHandlerOptions.ALL) -> None:
    """
    Install rclpy signal handlers.

    :param options: Indicate if to install sigint, sigterm, both or no signal handler.
    """
    _rclpy.install_signal_handlers(options)


def get_current_signal_handlers_options() -> SignalHandlerOptions:
    """
    Get current signal handler options.

    :return: rclpy.signals.SignalHandlerOptions instance.
    """
    return _rclpy.get_current_signal_handlers_options()


def uninstall_signal_handlers() -> None:
    """Uninstall the rclpy signal handlers."""
    _rclpy.uninstall_signal_handlers()


class SignalHandlerGuardCondition(GuardCondition):

    def __init__(self, context: 'Optional[Context]' = None) -> None:
        super().__init__(callback=None, callback_group=None, context=context)
        with self.handle:
            _rclpy.register_sigint_guard_condition(self.handle)

    def __del__(self) -> None:
        try:
            self.destroy()
        except InvalidHandle:
            # already destroyed
            pass
        except ValueError:
            # Guard condition was not registered
            pass

    def destroy(self) -> None:
        with self.handle:
            _rclpy.unregister_sigint_guard_condition(self.handle)
        super().destroy()
