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

import asyncio
import queue
import threading


class Task:
    """
    A task that can be executed.

    TODO Subclass asycio.Task? Need make it thread safe
    """

    def __init__(self, handler):
        self._handler = handler
        self._coroutine = None
        self._done = False
        self._lock = threading.Lock()
        self._queue = queue.Queue()

    def __call__(self):
        """
        Run or resume a task.

        This attempts to execute a callback. If the callback is a coroutine it will attempt to
        await it.
        """
        with self._lock:
            # Execute as much as we can (including done callbacks) until a coroutine yields
            while not self._done:
                if self._coroutine is None:
                    # A non-coroutine callback executes here
                    result = self._handler()
                    if asyncio.iscoroutine(result):
                        self._coroutine = result
                    self._handler = None

                if self._coroutine is not None:
                    try:
                        # A coroutine gets executed here
                        self._coroutine.send(None)
                    except StopIteration:
                        self._coroutine.close()
                        self._coroutine = None
                    else:
                        # The coroutine yielded
                        break

                try:
                    # Get the next callback, and make executing it part of this task
                    self._handler = self._queue.get_nowait()
                except queue.Empty:
                    self._done = True

    def done(self):
        return self._done

    def add_done_callback(self, fn):
        self._queue.put(fn)
