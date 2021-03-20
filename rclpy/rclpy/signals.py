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

import signal
import threading
import warnings
import weakref

from rclpy.guard_condition import GuardCondition


class SignalHandlerGuardCondition(GuardCondition):

    __instances = weakref.WeakSet()
    __old_handler = None
    __registered = False
    __lock = threading.Lock()

    @classmethod
    def __register_guard_condition(cls, instance):
        with cls.__lock:
            if not cls.__registered:
                cls.__old_handler = signal.signal(signal.SIGINT, cls.__sigint_handler)
            cls.__registered = True

            cls.__instances.add(instance)

    @classmethod
    def __maybe_uninstall_signal_handler(cls):
        with cls.__lock:
            if cls.__registered and not len(cls.__instances):
                current_handler = signal.signal(signal.SIGINT, cls.__old_handler)
                if current_handler != cls.__sigint_handler:
                    # Another signal handler was registered and chained to ours - put it back
                    hopefully_old = signal.signal(signal.SIGINT, current_handler)
                    if hopefully_old != cls.__old_handler:
                        # Someone registered yet another signal handler at the same time
                        warnings.warn(
                            f'Signal handler {hopefully_old} was mistakenly unregistered due to a'
                            + f' race condition when unregistering {cls.__sigint_handler}')
                else:
                    cls.__old_handler = None
                    cls.__registered = False

    @classmethod
    def __sigint_handler(cls, signum, frame):
        with cls.__lock:
            for gc in cls.__instances:
                gc.trigger()
            if cls.__old_handler:
                cls.__old_handler(signum, frame)
        cls.__maybe_uninstall_signal_handler()

    def __init__(self, context=None):
        super().__init__(callback=None, callback_group=None, context=context)

        self.__register_guard_condition(self)
