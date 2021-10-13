# Copyright 2021 Open Source Robotics Foundation, Inc.
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
from typing import List

from rclpy import context
from rclpy.exceptions import InvalidHandle
from rclpy.guard_condition import GuardCondition


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

    def has_sigint(self):
        """Return True if the instance is SIGINT or ALL."""
        return self is SignalHandlerOptions.SIGINT or self is SignalHandlerOptions.ALL

    def has_sigterm(self):
        """Return True if the instance is SIGTERM or ALL."""
        return self is SignalHandlerOptions.SIGTERM or self is SignalHandlerOptions.ALL


g_interrupt_guard_conditions: List['SignalHandlerGuardCondition'] = []
"""List of all interrupt guard conditions."""

g_current_signal_handlers_options: SignalHandlerOptions = SignalHandlerOptions.NO
"""Currently installed signal handlers."""

g_lock: threading.RLock = threading.RLock()
"""Lock protecting global state of this module."""

g_old_sigint_handler = signal.SIG_DFL
g_old_sigterm_handler = signal.SIG_DFL


def signal_handler(signum, frame):
    """Handle sigint and sigterm signals."""
    global g_interrupt_guard_conditions
    global g_lock
    if signum == signal.SIGTERM:
        context._shutdown_contexts_on_signal()
    with g_lock:
        for g in g_interrupt_guard_conditions:
            g.trigger()
    if signum == signal.SIGINT:
        raise KeyboardInterrupt()


def install_signal_handlers(options: SignalHandlerOptions = SignalHandlerOptions.ALL):
    """
    Install rclpy signal handlers.

    No signal chaining is automatically applied.
    If you need to install your own signal handler, pass SignalHandleOptions.NO to rclpy.init()
    and install your own signal handler that calls rclpy.signals.signal_handler().

    :param options: Indicate if to install sigint, sigterm, both or no signal handler.
    :return: tuple pair, first elements is the old sigint handler and second element the old
        sigterm handler. If the sigint or sigterm handler weren't installed,
        None is returned in that element.
    """
    global g_lock
    global g_current_signal_handlers_options
    global g_old_sigint_handler
    global g_old_sigterm_handler
    old_sigint_handler = None
    old_sigterm_handler = None
    with g_lock:
        if options.has_sigint() and not g_current_signal_handlers_options.has_sigint():
            old_sigint_handler = signal.signal(signal.SIGINT, signal_handler)
        if options.has_sigterm() and not g_current_signal_handlers_options.has_sigterm():
            old_sigterm_handler = signal.signal(signal.SIGTERM, signal_handler)
        g_current_signal_handlers_options = options
        if old_sigint_handler is not None:
            g_old_sigint_handler = old_sigint_handler
        if old_sigterm_handler is not None:
            g_old_sigterm_handler = old_sigterm_handler
    return (old_sigint_handler, old_sigterm_handler)


def get_current_signal_handlers_options():
    """
    Get current signal handler options.

    :return: rclpy.signals.SignalHandlerOptions instance.
    """
    global g_current_signal_handlers_options
    return g_current_signal_handlers_options


def uninstall_signal_handlers():
    """Uninstall the rclpy signal handlers, and reinstall the old ones."""
    global g_lock
    global g_current_signal_handlers_options
    global g_old_sigterm_handler
    global g_old_sigterm_handler
    with g_lock:
        if g_current_signal_handlers_options.has_sigint():
            print(f'old sigint handler {g_old_sigint_handler}')
            signal.signal(signal.SIGINT, g_old_sigint_handler)
        if g_current_signal_handlers_options.has_sigterm():
            signal.signal(signal.SIGTERM, g_old_sigterm_handler)


class SignalHandlerGuardCondition(GuardCondition):

    def __init__(self, context=None):
        global g_lock
        global g_interrupt_guard_conditions
        super().__init__(callback=None, callback_group=None, context=context)
        with g_lock:
            g_interrupt_guard_conditions.append(self)

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
        global g_lock
        global g_interrupt_guard_conditions
        with g_lock:
            g_interrupt_guard_conditions.remove(self)
        super().destroy()
