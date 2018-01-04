# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import inspect
import threading
import weakref


def _fake_weakref():
    """Return None when called to simulate a weak reference that has been garbage collected."""
    return None


class Future:
    """Represent the outcome of a task in the future."""

    def __init__(self, *, executor=None):
        # true if the task is done or cancelled
        self._done = False
        # true if the task is cancelled
        self._cancelled = False
        # the final return value of the handler
        self._result = None
        # An exception raised by the handler when called
        self._exception = None
        # callbacks to be scheduled after this task completes
        self._callbacks = []
        # Lock for threadsafety
        self._lock = threading.Lock()
        # An executor to use when scheduling done callbacks
        self._executor = _fake_weakref
        if executor is not None:
            self._executor = weakref.ref(executor)

    def __await__(self):
        # Yield if the task is not finished
        while not self._done:
            yield
        return self._result

    def cancel(self):
        """Request cancellation of the running task if it is not done already."""
        if not self._done:
            self._cancelled = True

    def cancelled(self):
        """
        Indicate if the task has been cancelled.

        :return: True if the task was cancelled
        :rtype: bool
        """
        return self._cancelled

    def done(self):
        """
        Indicate if the task has finished executing.

        :return: True if the task is finished or raised while it was executing
        :rtype: bool
        """
        return self._done

    def result(self):
        """
        Get the result of a done task.

        :return: The result set by the task
        """
        return self._result

    def exception(self):
        """
        Get an exception raised by a done task.

        :return: The exception raised by the task
        """
        return self._exception

    def set_result(self, result):
        """
        Set the result returned by a task.

        :param result: The output of a long running task.
        """
        self._result = result
        self._done = True
        self._cancelled = False
        self._schedule_done_callbacks()

    def set_exception(self, exception):
        """
        Set the exception raised by the task.

        :param result: The output of a long running task.
        """
        self._exception = exception
        self._done = True
        self._cancelled = False
        self._schedule_done_callbacks()

    def _schedule_done_callbacks(self):
        """Schedule done callbacks on the executor if possible."""
        executor = self._executor()
        if executor is not None:
            for callback in self._callbacks:
                executor.create_task(callback, self)

    def add_done_callback(self, callback):
        """
        Add a callback to be executed when the task is done.

        :param callback: a callback to be run when the task completes.
        """
        if self._done:
            executor = self._executor()
            if executor is not None:
                executor.create_task(callback, self)
        else:
            with self._lock:
                self._callbacks.append(callback)


class Task(Future):
    """
    Execute a function or coroutine.

    This executes either a normal function or a coroutine to completion. On completion it creates
    tasks for any 'done' callbacks.

    This class should only be instantiated by :class:`rclpy.executors.Executor`.
    """

    def __init__(self, handler, *args, executor=None):
        super().__init__(executor=executor)
        # _handler is either a normal function or a coroutine
        self._handler = handler
        # Arguments passed into the function
        self._args = args
        if inspect.iscoroutinefunction(handler):
            self._handler = handler(*args)
            self._args = None
        # True while the task is being executed
        self._executing = False

    def __call__(self):
        """
        Run or resume a task.

        This attempts to execute a handler. If the handler is a coroutine it will attempt to
        await it. If there are done callbacks it will schedule them with the executor.

        The return value of the handler is stored as the task result.
        """
        if self._done or self._executing:
            return
        with self._lock:
            if self._done:
                return
            self._executing = True

            if inspect.iscoroutine(self._handler):
                # Execute a coroutine
                try:
                    self._handler.send(None)
                except StopIteration as e:
                    # The coroutine finished; store the result
                    self._handler.close()
                    self.set_result(e.value)
                    self._complete_task()
                except Exception as e:
                    self.set_exception(e)
                    self._complete_task()
            else:
                # Execute a normal function
                try:
                    self.set_result(self._handler(*self._args))
                except Exception as e:
                    self.set_exception(e)
                self._complete_task()

            self._executing = False

    def _complete_task(self):
        """Store result and schedule done callbacks."""
        self._handler = None
        self._args = None

    def executing(self):
        """
        Check if the task is currently being executed.

        :return: True if the task is currently executing.
        :rtype: bool
        """
        return self._executing
