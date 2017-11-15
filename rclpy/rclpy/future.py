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


class Future:
    """Access the result of a :class:`.Task`."""

    def __init__(self, task):
        self._task = task

    def __await__(self):
        # Yield if the task is not finished
        while not self._task.done():
            yield

    def result(self):
        """
        Get the result of the task.

        :returns: the result of the task this future tracks, or None if the task is not done.
        """
        if self._task.done():
            return self._task.result()


class Task:
    """A task that can be executed."""

    def __init__(self, handler):
        self._handler = handler
        self._coroutine = None
        self._done = False
        self._lock = threading.Lock()
        self._queue = queue.Queue()
        self._result = None

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
                    self._result = self._handler()
                    if asyncio.iscoroutine(self._result):
                        self._coroutine = self._result
                        self._result = None
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

    def result(self):
        return self._result

    def add_done_callback(self, fn):
        self._queue.put(fn)
