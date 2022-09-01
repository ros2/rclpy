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


class NumberOfEntities:

    __slots__ = [
        'num_subscriptions',
        'num_guard_conditions',
        'num_timers',
        'num_clients',
        'num_services',
        'num_events']

    def __init__(
        self, num_subs=0, num_gcs=0, num_timers=0,
        num_clients=0, num_services=0, num_events=0
    ):
        self.num_subscriptions = num_subs
        self.num_guard_conditions = num_gcs
        self.num_timers = num_timers
        self.num_clients = num_clients
        self.num_services = num_services
        self.num_events = num_events

    def __add__(self, other):
        result = self.__class__()
        for attr in result.__slots__:
            left = getattr(self, attr)
            right = getattr(other, attr)
            setattr(result, attr, left + right)
        return result

    def __repr__(self):
        return '<{0}({1}, {2}, {3}, {4}, {5}, {6})>'.format(
            self.__class__.__name__, self.num_subscriptions,
            self.num_guard_conditions, self.num_timers, self.num_clients,
            self.num_services, self.num_events)


class Waitable:
    """
    Add something to a wait set and execute it.

    This class wraps a collection of entities which can be added to a wait set.
    """

    def __init__(self, callback_group):
        # A callback group to control when this entity can execute (used by Executor)
        self.callback_group = callback_group
        self.callback_group.add_entity(self)
        # Flag set by executor when a handler has been created but not executed (used by Executor)
        self._executor_event = False
        # List of Futures that have callbacks needing execution
        self._futures = []

    def __enter__(self):
        """Implement to mark entities as in-use to prevent destruction while waiting on them."""
        pass

    def __exit__(self, t, v, tb):
        """Implement to mark entities as not-in-use to allow destruction after waiting on them."""
        pass

    def add_future(self, future):
        self._futures.append(future)

    def remove_future(self, future):
        self._futures.remove(future)

    def is_ready(self, wait_set):
        """Return True if entities are ready in the wait set."""
        raise NotImplementedError('Must be implemented by subclass')

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        raise NotImplementedError('Must be implemented by subclass')

    async def execute(self, taken_data):
        """Execute work after data has been taken from a ready wait set."""
        raise NotImplementedError('Must be implemented by subclass')

    def get_num_entities(self):
        """Return number of each type of entity used."""
        raise NotImplementedError('Must be implemented by subclass')

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        raise NotImplementedError('Must be implemented by subclass')
