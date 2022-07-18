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

import asyncio
import inspect
import sys
import threading
import warnings
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
        self._exception_fetched = False
        # callbacks to be scheduled after this task completes
        self._callbacks = []
        # Lock for threadsafety
        self._lock = threading.Lock()
        # An executor to use when scheduling done callbacks
        self._executor = None
        self._set_executor(executor)

    def __del__(self):
        if self._exception is not None and not self._exception_fetched:
            print(
                'The following exception was never retrieved: ' + str(self._exception),
                file=sys.stderr)

    def __await__(self):
        if not self._done:
            yield self
        return self.result()

    __iter__ = __await__

    def cancel(self):
        """Request cancellation of the running task if it is not done already."""
        with self._lock:
            if not self._done:
                self._cancelled = True
        self._schedule_or_invoke_done_callbacks()

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

        :raises: Exception if one was set during the task.

        :return: The result set by the task, or None if no result was set.
        """
        if self._exception:
            raise self.exception()
        return self._result

    def exception(self):
        """
        Get an exception raised by a done task.

        :return: The exception raised by the task
        """
        self._exception_fetched = True
        return self._exception

    def set_result(self, result):
        """
        Set the result returned by a task.

        :param result: The output of a long running task.
        """
        with self._lock:
            self._result = result
            self._done = True
            self._cancelled = False
        self._schedule_or_invoke_done_callbacks()

    def set_exception(self, exception):
        """
        Set the exception raised by the task.

        :param result: The output of a long running task.
        """
        with self._lock:
            self._exception = exception
            self._exception_fetched = False
            self._done = True
            self._cancelled = False
        self._schedule_or_invoke_done_callbacks()

    def _schedule_or_invoke_done_callbacks(self):
        """
        Schedule done callbacks on the executor if possible, else run them directly.

        This function assumes self._lock is not held.
        """
        with self._lock:
            executor = self._executor()
            callbacks = self._callbacks
            self._callbacks = []

        if executor is not None:
            # Have the executor take care of the callbacks
            for callback in callbacks:
                executor.call_soon(callback, self)
        else:
            # No executor, call right away
            for callback in callbacks:
                try:
                    callback(self)
                except Exception as e:
                    # Don't let exceptions be raised because there may be more callbacks to call
                    warnings.warn('Unhandled exception in done callback: {}'.format(e))

    def _set_executor(self, executor):
        """Set the executor this future is associated with."""
        with self._lock:
            if executor is None:
                self._executor = _fake_weakref
            else:
                self._executor = weakref.ref(executor)

    def add_done_callback(self, callback):
        """
        Add a callback to be executed when the task is done.

        Callbacks should not raise exceptions.

        The callback may be called immediately by this method if the future is already done.
        If this happens and the callback raises, the exception will be raised by this method.

        :param callback: a callback taking the future as an agrument to be run when completed
        """
        invoke = False
        with self._lock:
            if self._done:
                executor = self._executor()
                if executor is not None:
                    executor.call_soon(callback, self)
                else:
                    invoke = True
            else:
                self._callbacks.append(callback)

        # Invoke when not holding self._lock
        if invoke:
            callback(self)


class Task(Future):
    """
    Execute a function or coroutine.

    This executes either a normal function or a coroutine to completion. On completion it creates
    tasks for any 'done' callbacks.

    This class should only be instantiated by :class:`rclpy.executors.Executor`.
    """

    def __init__(self, handler, args=None, kwargs=None, executor=None):
        super().__init__(executor=executor)
        if executor is None:
            raise RuntimeError('Task requires an executor')
        # _handler is either a normal function or a coroutine
        self._handler = handler
        # Arguments passed into the function
        if args is None:
            args = []
        self._args = args
        if kwargs is None:
            kwargs = {}
        self._kwargs = kwargs
        if inspect.iscoroutinefunction(handler):
            self._handler = handler(*args, **kwargs)
            self._args = None
            self._kwargs = None

    def __call__(self, future=None):
        """
        Run or resume a task.

        This attempts to execute a handler. If the handler is a coroutine it will attempt to
        await it. If there are done callbacks it will schedule them with the executor.

        The return value of the handler is stored as the task result.

        This function must not be called in parallel with itself.

        :param future: do not use
        """
        if self._done:
            return
        if inspect.iscoroutine(self._handler):
            # Execute a coroutine
            try:
                result = self._handler.send(None)
                if isinstance(result, Future):
                    # Wait for an rclpy future to complete
                    result.add_done_callback(self)
                elif asyncio.isfuture(result):
                    # Get the event loop of this thread (raises RuntimeError if there isn't one)
                    event_loop = asyncio.get_running_loop()
                    # Make sure we're in the same thread as the future's event loop.
                    # TODO(sloretz) is asyncio.Future.get_loop() thread-safe?
                    if result.get_loop() is not event_loop:
                        raise RuntimeError('Cannot await asyncio future from a different thread')
                    # Resume this task when the asyncio future completes
                    result.add_done_callback(lambda _: self._executor().call_soon(self))
                elif result is None:
                    # Wait for one iteration if a bare yield is used
                    self._executor().call_soon(self)
                else:
                    # What is this intermediate value?
                    # Could be a different async library's coroutine
                    # Could be a generator yielded a value
                    raise RuntimeError(f'Coroutine yielded unexpected value: {result}')
            except StopIteration as e:
                # Coroutine or generator returning a result
                self._handler.close()
                self.set_result(e.value)
                self._complete_task()
            except Exception as e:
                # Coroutine or generator raising an exception
                self._handler.close()
                self.set_exception(e)
                self._complete_task()
        else:
            # Execute a normal function
            try:
                self.set_result(self._handler(*self._args, **self._kwargs))
            except Exception as e:
                self.set_exception(e)
            self._complete_task()

    def _complete_task(self):
        """Cleanup after task finished."""
        self._handler = None
        self._args = None
        self._kwargs = None
