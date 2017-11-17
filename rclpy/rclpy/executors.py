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

from concurrent.futures import ThreadPoolExecutor
import multiprocessing
from threading import Condition
from threading import Lock

from rclpy.future import Task
from rclpy.guard_condition import GuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.impl.implementation_singleton import rclpy_wait_set_implementation as _rclpy_wait_set
from rclpy.timer import WallTimer
from rclpy.utilities import ok
from rclpy.utilities import timeout_sec_to_nsec


class Executor:
    """
    A base class for an executor.

    An executor controls the threading model used to process callbacks. Callbacks are units of work
    like subscription callbacks, timer callbacks, service calls, and received client responses. An
    executor controls which threads callbacks get executed in.

    A custom executor must define :func:`Executor.spin_once`. If the executor has any cleanup then
    it should also define :func:`Executor.shutdown`.
    """

    def __init__(self):
        super().__init__()
        self._nodes = set()
        self._nodes_lock = Lock()
        # This is triggered when wait_for_ready_callbacks should rebuild the wait list
        self._guard_condition = GuardCondition(None, None)
        # Triggered by signal handler for sigint
        self._sigint_gc, _ = _rclpy.rclpy_get_sigint_guard_condition()
        # True if shutdown has been called
        self._is_shutdown = False
        # 3-tuple (Task() instance, entity, node)
        self._tasks = []
        self._work_condition = Condition()
        self._wait_set = _rclpy_wait_set.WaitSet()

    def shutdown(self, timeout_sec=None):
        """
        Stop executing callbacks and wait for their completion.

        Return true if all outstanding callbacks finished executing.

        :param timeout_sec: Seconds to wait. Block forever if None or negative. Don't wait if 0
        :type timeout_sec: float or None
        :rtype: bool
        """
        self._is_shutdown = True

        if timeout_sec is None or timeout_sec >= 0:
            tasks = self._tasks

            def no_work_running():
                nonlocal tasks
                return not [t for t, e, n in tasks if not t.done()]

            with self._work_condition:
                self._work_condition.wait_for(no_work_running, timeout_sec)

        with self._nodes_lock:
            self._nodes = set()
        _rclpy.rclpy_destroy_entity(self._guard_condition.guard_handle)
        _rclpy.rclpy_destroy_entity(self._sigint_gc)

        self._guard_condition = None
        self._sigint_gc = None
        self._tasks = None
        return True

    def __del__(self):
        if self._guard_condition is not None:
            _rclpy.rclpy_destroy_entity(self._guard_condition.guard_handle)
            _rclpy.rclpy_destroy_entity(self._sigint_gc)

    def add_node(self, node):
        """
        Add a node whose callbacks should be managed by this executor.

        Return true if the node was added.

        :rtype: bool
        """
        with self._nodes_lock:
            self._nodes.add(node)
            # Rebuild the wait set so it includes this new node
            self._guard_condition.trigger()

    def get_nodes(self):
        """
        Return nodes which have been added to this executor.

        :rtype: list
        """
        with self._nodes_lock:
            return list(self._nodes)

    def spin(self):
        """Execute callbacks until shutdown."""
        while ok():
            self.spin_once()

    def spin_once(self, timeout_sec=None):
        """
        Wait for and execute a single callback.

        A custom executor should use :func:`Executor.wait_for_ready_callbacks` to get work.

        :param timeout_sec: Seconds to wait. Block forever if None or negative. Don't wait if 0
        :type timeout_sec: float or None
        :rtype: None
        """
        raise NotImplementedError()

    def _make_task(self, entity):
        """
        Make a task that performs work on an entity.

        :param entity: An entity to wait on
        :rtype: rclpy.future.Task
        """
        gc = self._guard_condition
        is_shutdown = self._is_shutdown
        # Mark this so it doesn't get added back to the wait list
        entity._executor_handle.notify_ready()

        def execute():
            nonlocal entity
            nonlocal gc
            nonlocal is_shutdown
            if is_shutdown or not entity.callback_group.beginning_execution(entity):
                # Didn't get the callback, or the executor has been ordered to stop
                entity._executor_handle.cancel_ready()
                gc.trigger()
                return

            # Signal that this has been 'taken' and can be added back to the wait list
            arg = entity._executor_handle.take_from_wait_list()
            gc.trigger()
            # Return result of callback (Task will do the right thing if this is a coroutine)
            return entity._executor_handle.execute_callback(arg)

        def postExecute():
            nonlocal entity
            nonlocal gc
            # Notify callback group this entity is no longer executing
            entity.callback_group.ending_execution(entity)
            # Notify the wait set can be rebuilt
            gc.trigger()
            # Notify anyone waiting for shutdown
            with self._work_condition:
                self._work_condition.notify_all()

        task = Task(execute)
        task.add_done_callback(postExecute)
        return task

    def _process_new_callbacks(self, nodes, wait_set):
        """
        Create Tasks for brand new work.

        :param nodes: nodes to yield work for
        :type nodes: list
        :param wait_set: wait set that has already been waited on
        :type wait_set: _rclpy_wait_set.WaitSet
        """
        # Process ready entities one node at a time
        for node in nodes:
            for tmr in node.timers:
                if wait_set.is_ready(tmr) and tmr.callback_group.can_execute(tmr):
                    # TODO(Sloretz) Which rcl cancelled timer bug does this workaround?
                    if not _rclpy.rclpy_is_timer_ready(tmr.timer_handle):
                        continue
                    task = self._make_task(tmr)
                    self._tasks.append((task, tmr, node))

            for sub in node.subscriptions:
                if wait_set.is_ready(sub) and sub.callback_group.can_execute(sub):
                    task = self._make_task(sub)
                    self._tasks.append((task, sub, node))

            for gc in node.guards:
                if wait_set.is_ready(gc) and gc.callback_group.can_execute(gc):
                    task = self._make_task(gc)
                    self._tasks.append((task, gc, node))

            for cli in node.clients:
                if wait_set.is_ready(cli) and cli.callback_group.can_execute(cli):
                    task = self._make_task(cli)
                    self._tasks.append((task, cli, node))

            for srv in node.services:
                if wait_set.is_ready(srv) and srv.callback_group.can_execute(srv):
                    task = self._make_task(srv)
                    self._tasks.append((task, srv, node))

    def can_execute(self, entity):
        """
        Determine if a callback for an entity can be executed.

        :param entity: Subscription, Timer, Guard condition, etc
        :returns: True if the entity callback can be executed
        :rtype: bool
        """
        return not entity._executor_handle.ready() and entity.callback_group.can_execute(entity)

    def wait_for_ready_callbacks(self, timeout_sec=None, nodes=None):
        """
        Yield callbacks that are ready to be performed.

        Executors must reuse the returned generator until StopIteration is raised. Failure to do
        so could result in coroutines never being resumed.

        :param timeout_sec: Seconds to wait. Block forever if None or negative. Don't wait if 0
        :type timeout_sec: float or None
        :param nodes: A list of nodes to wait on. Wait on all nodes if None.
        :type nodes: list or None
        :rtype: Generator[(callable, entity, :class:`rclpy.node.Node`)]
        """
        timeout_timer = None
        timeout_nsec = timeout_sec_to_nsec(timeout_sec)
        if timeout_nsec > 0:
            timeout_timer = WallTimer(None, None, timeout_nsec)

        if nodes is None:
            nodes = self.get_nodes()

        yielded_work = False
        while not yielded_work and not self._is_shutdown:
            # Gather entities that can be waited on
            self._wait_set.clear()
            for node in nodes:
                self._wait_set.add_subscriptions(filter(self.can_execute, node.subscriptions))
                self._wait_set.add_timers(filter(self.can_execute, node.timers))
                self._wait_set.add_clients(filter(self.can_execute, node.clients))
                self._wait_set.add_services(filter(self.can_execute, node.services))
                self._wait_set.add_guard_conditions(filter(self.can_execute, node.guards))

            self._wait_set.add_guard_conditions((self._sigint_gc, self._guard_condition))

            if timeout_timer is not None:
                self._wait_set.add_timers((timeout_timer,))

            # Wait for something to become ready
            self._wait_set.wait(timeout_nsec)

            # Check sigint guard condition
            if self._wait_set.is_ready(self._sigint_gc):
                raise KeyboardInterrupt()

            # Turn new work into tasks to be executed
            self._process_new_callbacks(nodes, self._wait_set)

            # Clean up old tasks
            self._tasks = [(task, e, n) for task, e, n in self._tasks if not task.done()]

            # Yield work to the executor (reversed so newer work is executed first)
            yielded_work = yield from reversed(self._tasks)

            # Check timeout timer
            if (
                timeout_nsec == 0 or
                (timeout_timer is not None and self._wait_set.is_ready(timeout_timer))
            ):
                break


class SingleThreadedExecutor(Executor):
    """Runs callbacks in the thread which calls :func:`SingleThreadedExecutor.spin`."""

    def __init__(self):
        super().__init__()
        self._callback_iter = None

    def spin_once(self, timeout_sec=None):
        # Reuse the same callback iterator to avoid unecessary calls to rcl_wait
        if self._callback_iter is None:
            self._callback_iter = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        try:
            handler, entity, node = next(self._callback_iter)
        except StopIteration:
            self._callback_iter = None
        else:
            handler()


class MultiThreadedExecutor(Executor):
    """Runs callbacks in a pool of threads."""

    def __init__(self, num_threads=None):
        """
        Initialize the executor.

        :param num_threads: number of worker threads in the pool. If None the number of threads
                      will use multiprocessing.cpu_count(). If that's not implemented the number
                      of threads defaults to 1.
        :type num_threads: int
        """
        super().__init__()
        self._callback_iter = None
        if num_threads is None:
            try:
                num_threads = multiprocessing.cpu_count()
            except NotImplementedError:
                num_threads = 1
        self._executor = ThreadPoolExecutor(num_threads)

    def spin_once(self, timeout_sec=None):
        # Reuse the same callback iterator to avoid unecessary calls to rcl_wait
        if self._callback_iter is None:
            self._callback_iter = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        try:
            handler, entity, node = next(self._callback_iter)
        except StopIteration:
            self._callback_iter = None
        else:
            self._executor.submit(handler)
