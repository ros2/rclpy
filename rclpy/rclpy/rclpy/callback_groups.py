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
    """
    The base class for a callback group.

    A callback group controls when callbacks are allowed to be executed.

    This class should not be instantiated.
    Instead, classes should extend it and implement :meth:`can_execute`,
    :meth:`beginning_execution`, and :meth:`ending_execution`.
    """

    def __init__(self) -> None:
        super().__init__()
        self.entities: set = set()

    def add_entity(self, entity) -> None:
        """
        Add an entity to the callback group.

        :param entity: a subscription, timer, client, service, or waitable instance.
        """
        self.entities.add(weakref.ref(entity))

    def has_entity(self, entity) -> bool:
        """
        Determine if an entity has been added to this group.

        :param entity: a subscription, timer, client, service, or waitable instance.
        """
        return weakref.ref(entity) in self.entities

    def can_execute(self, entity) -> bool:
        """
        Determine if an entity can be executed.

        :param entity: a subscription, timer, client, service, or waitable instance.
        :return: ``True`` if the entity can be executed, ``False`` otherwise.
        """
        raise NotImplementedError()

    def beginning_execution(self, entity) -> bool:
        """
        Get permission for the callback from the group to begin executing an entity.

        If this returns ``True`` then :meth:`CallbackGroup.ending_execution` must be called after
        the callback has been executed.

        :param entity: a subscription, timer, client, service, or waitable instance.
        :return: ``True`` if the callback can be executed, ``False`` otherwise.
        """
        raise NotImplementedError()

    def ending_execution(self, entity) -> None:
        """
        Notify group that a callback has finished executing.

        :param entity: a subscription, timer, client, service, or waitable instance.
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
            return self._active_entity is None

    def beginning_execution(self, entity):
        with self._lock:
            assert weakref.ref(entity) in self.entities
            if self._active_entity is None:
                self._active_entity = entity
                return True
        return False

    def ending_execution(self, entity):
        with self._lock:
            assert self._active_entity == entity
            self._active_entity = None
