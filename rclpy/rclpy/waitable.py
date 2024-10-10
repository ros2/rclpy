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

from types import TracebackType
from typing import Any, Generic, List, Optional, Type, TYPE_CHECKING, TypeVar


from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

T = TypeVar('T')

if TYPE_CHECKING:
    from typing_extensions import Self

    from rclpy.callback_groups import CallbackGroup
    from rclpy.task import Future


class NumberOfEntities:

    __slots__ = [
        'num_subscriptions',
        'num_guard_conditions',
        'num_timers',
        'num_clients',
        'num_services',
        'num_events']

    def __init__(
        self, num_subs: int = 0, num_gcs: int = 0, num_timers: int = 0,
        num_clients: int = 0, num_services: int = 0, num_events: int = 0
    ):
        self.num_subscriptions = num_subs
        self.num_guard_conditions = num_gcs
        self.num_timers = num_timers
        self.num_clients = num_clients
        self.num_services = num_services
        self.num_events = num_events

    def __add__(self, other: 'NumberOfEntities') -> 'NumberOfEntities':
        result = self.__class__()
        result.num_subscriptions = self.num_subscriptions + other.num_subscriptions
        result.num_guard_conditions = self.num_guard_conditions + other.num_guard_conditions
        result.num_timers = self.num_timers + other.num_timers
        result.num_clients = self.num_clients + other.num_clients
        result.num_services = self.num_services + other.num_services
        result.num_events = self.num_events + other.num_events
        return result

    def __iadd__(self, other: 'NumberOfEntities') -> 'NumberOfEntities':
        self.num_subscriptions += other.num_subscriptions
        self.num_guard_conditions += other.num_guard_conditions
        self.num_timers += other.num_timers
        self.num_clients += other.num_clients
        self.num_services += other.num_services
        self.num_events += other.num_events
        return self

    def __repr__(self) -> str:
        return '<{0}({1}, {2}, {3}, {4}, {5}, {6})>'.format(
            self.__class__.__name__, self.num_subscriptions,
            self.num_guard_conditions, self.num_timers, self.num_clients,
            self.num_services, self.num_events)


class Waitable(Generic[T]):
    """
    Add something to a wait set and execute it.

    This class wraps a collection of entities which can be added to a wait set.
    """

    def __init__(self, callback_group: 'CallbackGroup'):
        # A callback group to control when this entity can execute (used by Executor)
        self.callback_group = callback_group
        self.callback_group.add_entity(self)
        # Flag set by executor when a handler has been created but not executed (used by Executor)
        self._executor_event = False
        # List of Futures that have callbacks needing execution
        self._futures: List[Future[Any]] = []

    def __enter__(self) -> 'Self':
        """Implement to mark entities as in-use to prevent destruction while waiting on them."""
        raise NotImplementedError('Must be implemented by subclass')

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> None:
        """Implement to mark entities as not-in-use to allow destruction after waiting on them."""
        raise NotImplementedError('Must be implemented by subclass')

    def add_future(self, future: 'Future[Any]') -> None:
        self._futures.append(future)

    def remove_future(self, future: 'Future[Any]') -> None:
        self._futures.remove(future)

    def is_ready(self, wait_set: _rclpy.WaitSet) -> bool:
        """Return True if entities are ready in the wait set."""
        raise NotImplementedError('Must be implemented by subclass')

    def take_data(self) -> T:
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        raise NotImplementedError('Must be implemented by subclass')

    async def execute(self, taken_data: T) -> None:
        """Execute work after data has been taken from a ready wait set."""
        raise NotImplementedError('Must be implemented by subclass')

    def get_num_entities(self) -> NumberOfEntities:
        """Return number of each type of entity used."""
        raise NotImplementedError('Must be implemented by subclass')

    def add_to_wait_set(self, wait_set: _rclpy.WaitSet) -> None:
        """Add entities to wait set."""
        raise NotImplementedError('Must be implemented by subclass')
