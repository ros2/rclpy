from typing import Literal, Protocol

from rclpy.client import ClientHandle
from rclpy.context import ContextHandle
from rclpy.destroyable import DestroyableType
from rclpy.event_handler import EventHandle
from rclpy.guard_condition import GuardConditionHandle
from rclpy.service import ServiceHandle
from rclpy.subscription import SubscriptionHandle
from rclpy.timer import TimerHandle


IsReadyValues = Literal['subscription', 'client', 'service', 'timer', 'guard_condition', 'event']
GetReadyEntityValues = Literal['subscription', 'client', 'service', 'timer', 'guard_condition']


class WaitSetHandle(DestroyableType, Protocol):

    def __init__(self, number_of_subscriptions: int, number_of_guard_conditions: int,
                 number_of_timers: int, number_of_clients: int, number_of_services: int,
                 number_of_events: int, context: ContextHandle) -> None: ...
    """Construct a WaitSet."""

    @property
    def pointer(self) -> int: ...
    """Get the address of the entity as an integer"""

    def clear_entities(self) -> None: ...
    """Clear all the pointers in the wait set"""

    def add_service(self, service: ServiceHandle) -> int: ...
    """Add a service to the wait set structure"""

    def add_subscription(self, subscription: SubscriptionHandle) -> int: ...
    """Add a subcription to the wait set structure"""

    def add_client(self, client: ClientHandle) -> int: ...
    """Add a client to the wait set structure"""

    def add_guard_condition(self, guard_condition: GuardConditionHandle) -> int: ...
    """Add a guard condition to the wait set structure"""

    def add_timer(self, timer: TimerHandle) -> int: ...
    """Add a timer to the wait set structure"""

    def add_event(self, event: EventHandle) -> int: ...
    """Add an event to the wait set structure"""

    def is_ready(self, entity_type: IsReadyValues, index: int) -> bool: ...
    """Check if an entity in the wait set is ready by its index"""

    def get_ready_entities(self, entity_type: GetReadyEntityValues) -> list[int]: ...
    """Get list of entities ready by entity type"""

    def wait(self, timeout: int) -> None: ...
    """Wait until timeout is reached or event happened"""
