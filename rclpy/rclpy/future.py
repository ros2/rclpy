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
        return self._task.result()

    def result(self):
        """
        Get the result of the task.

        :returns: the result of the task this future tracks, or None if the task is not done.
        """
        if self._task.done():
            return self._task.result()

    def add_done_callback(self, func):
        """Add a callback to be executed when a task is done."""
        self._task.add_done_callback(func)

    def cancel(self):
        """Cancel the running task."""
        self._task.cancel()

    def is_cancelled(self):
        """Return true if the task is cancelled."""
        return self._task.is_cancelled()

    def done(self):
        """Return True if the task is done or cancelled."""
        return self._task.done()


class Present(Future):
    """Be a Future whose result is known now."""

    def __init__(self, result):
        task = Task(lambda: result)
        task()
        super().__init__(task)


class Task:
    """A task that can be executed."""

    def __init__(self, handler):
        # handler is the next thing to run. It may or may not be a coroutine
        self._handler = handler
        # set to a coroutine returned by a handler
        self._coroutine = None
        self._done = False
        self._cancelled = False
        self._lock = threading.Lock()
        self._queue = queue.Queue()
        self._has_result = False
        self._result = None

    def __call__(self):
        """
        Run or resume a task.

        This attempts to execute a handler. If the handler is a coroutine it will attempt to
        await it. If there are done callbacks it will repeat that process until all callbacks have
        been executed.

        The return value of the handler passed to the constructor is stored as the task result.
        """
        with self._lock:
            # Execute as much as it can (including done callbacks) until a coroutine yields
            while not self._done:
                result = None
                if self._handler is not None:
                    # A non-coroutine callback executes here
                    result = self._handler()
                    if asyncio.iscoroutine(result):
                        self._coroutine = result
                    self._handler = None

                if self._coroutine is not None:
                    try:
                        # A coroutine gets executed here
                        result = self._coroutine.send(None)
                    except StopIteration:
                        self._coroutine.close()
                        self._coroutine = None
                    else:
                        # The coroutine yielded
                        break

                if self._coroutine is None and not self._has_result:
                    print('Has result', result)
                    self._has_result = True
                    self._result = result

                try:
                    # Get the next callback, and make executing it part of this task
                    self._handler = self._queue.get_nowait()
                except queue.Empty:
                    self._done = True

    def cancel(self):
        self._cancelled = True
        self._done = True

    def is_cancelled(self):
        return self._cancelled

    def done(self):
        return self._done

    def result(self):
        return self._result

    def add_done_callback(self, fn):
        with self._lock:
            if self._done:
                fn()
            else:
                self._queue.put(fn)
