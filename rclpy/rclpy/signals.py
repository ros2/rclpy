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

from enum import Enum
import signal
import threading

from rclpy import context
from rclpy.exceptions import InvalidHandle
from rclpy.guard_condition import GuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class SignalHandlerOptions(Enum):
    """Enum to indicate which signal handlers to install."""

    # WARN: If this class changes, check also `signal_handler.cpp`

    NO = 0
    """No signal handler should be installed."""

    SIGINT = 1
    """Install only a sigint handler."""

    SIGTERM = 2
    """Install only a sigterm handler."""

    ALL = 3
    """Install both a sigint and a sigterm handler."""


g_lock = threading.Lock()
g_old_sigterm_handler = None


def install_signal_handlers(options: SignalHandlerOptions = SignalHandlerOptions.ALL):
    """
    Install rclpy signal handlers.

    :param options: Indicate if to install sigint, sigterm, both or no signal handler.
    """
    global g_lock
    global g_old_sigterm_handler
    with g_lock:
        if options in (SignalHandlerOptions.ALL, SignalHandlerOptions.SIGTERM):
            def _signal_handler(signum, frame):
                context._shutdown_all_contexts()
            g_old_sigterm_handler = signal.signal(signal.SIGTERM, _signal_handler)
        return _rclpy.install_signal_handlers(options.value)


def get_current_signal_handlers_options():
    """
    Get current signal handler options.

    :return: rclpy.signals.SignalHandlerOptions instance.
    """
    return SignalHandlerOptions(_rclpy.get_current_signal_handlers_options())


def uninstall_signal_handlers():
    """Uninstall the rclpy signal handlers."""
    global g_lock
    global g_old_sigterm_handler
    with g_lock:
        if g_old_sigterm_handler is not None:
            signal.signal(signal.SIGTERM, g_old_sigterm_handler)
        return _rclpy.uninstall_signal_handlers()


class SignalHandlerGuardCondition(GuardCondition):

    def __init__(self, context=None):
        super().__init__(callback=None, callback_group=None, context=context)
        with self.handle:
            _rclpy.register_sigint_guard_condition(self.handle)

    def __del__(self):
        try:
            self.destroy()
        except InvalidHandle:
            # already destroyed
            pass
        except ValueError:
            # Guard condition was not registered
            pass

    def destroy(self):
        with self.handle:
            _rclpy.unregister_sigint_guard_condition(self.handle)
        super().destroy()
