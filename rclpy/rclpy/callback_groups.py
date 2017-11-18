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

from threading import Lock
import weakref


class CallbackGroup:
    """Control when callbacks are allowed to be executed."""

    def __init__(self):
        super().__init__()
        self.entities = set()

    def add_entity(self, entity):
        """
        Add an entity to the callback group.

        :param entity: a subscription, timer, client, or service instance
        :rtype: None
        """
        self.entities.add(weakref.ref(entity))

    def has_entity(self, entity):
        """
        Determine if an entity has been added to this group.

        :param entity: a subscription, timer, client, or service instance
        :rtype: bool
        """
        return weakref.ref(entity) in self.entities

    def can_execute(self, entity):
        """
        Return true if an entity can be executed.

        The executor may call this on a task that has already started execution. In this case the
        callback group should return True from this method.

        :param entity: a subscription, timer, client, or service instance
        :rtype: bool
        """
        raise NotImplementedError()

    def beginning_execution(self, entity):
        """
        Get permission from the callback from the group to begin executing an entity.

        Return true if the callback can be executed, false otherwise. If this returns True then
        :func:`CallbackGroup.ending_execution` must be called after the callback has been executed.

        :param entity: a subscription, timer, client, or service instance
        :rtype: bool
        """
        raise NotImplementedError()

    def ending_execution(self, entity):
        """
        Notify group that a callback has finished executing.

        :param entity: a subscription, timer, client, or service instance
        :rtype: None
        """
        raise NotImplementedError()


class ReentrantCallbackGroup(CallbackGroup):
    """Allow callbacks to be executed in parallel without restriction."""

    def can_execute(self, entity):
        return True

    def beginning_execution(self, entity):
        return True

    def ending_execution(self, entity):
        pass


class MutuallyExclusiveCallbackGroup(CallbackGroup):
    """Allow only one callback to be executing at a time."""

    def __init__(self):
        super().__init__()
        self._active_entity = None
        self._lock = Lock()

    def can_execute(self, entity):
        with self._lock:
            assert weakref.ref(entity) in self.entities
            return self._active_entity is None or weakref.ref(entity) == self._active_entity

    def beginning_execution(self, entity):
        with self._lock:
            weak_entity = weakref.ref(entity)
            assert weak_entity in self.entities
            if self._active_entity is None:
                self._active_entity = weak_entity
                return True
        return False

    def ending_execution(self, entity):
        with self._lock:
            weak_entity = weakref.ref(entity)
            assert self._active_entity == weak_entity
            self._active_entity = None
