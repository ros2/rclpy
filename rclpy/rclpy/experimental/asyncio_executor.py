# Copyright 2025 Nadav Elkabets
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
from functools import partial
import traceback
import warnings
from typing import Callable
from typing import Coroutine
from typing import Dict
from typing import List
from typing import Optional
from typing import Set
from typing import TypeVar
from typing import Union

from rclpy.client import Client
from rclpy.context import Context
from rclpy.executors import BaseExecutor
from rclpy.node import Node
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.utilities import get_default_context

EntityT = TypeVar('EntityT', bound=Union[Subscription, Service, Client])


class AsyncioExecutor(BaseExecutor):
    def __init__(
        self, loop: Optional[asyncio.AbstractEventLoop] = None,
        *,
        context: Optional[Context] = None
    ) -> None:
        self._owns_loop = False
        self._loop = loop or self._get_loop()
        self._context = context or get_default_context()
        self._context.on_shutdown(self._sync_shutdown)
        self._nodes: Set['Node'] = set()
        self._subscription_to_node: Dict[Subscription, 'Node'] = {}
        self._client_to_node: Dict[Client, 'Node'] = {}
        self._node_to_tasks: Dict['Node', Set[asyncio.Task]] = {}
        self._shutdown_task: Optional[asyncio.Task] = None

    def get_nodes(self) -> List['Node']:
        """Return nodes that have been added to this executor."""
        return list(self._nodes)

    @property
    def context(self) -> Context:
        """Get the context associated with the executor."""
        return self._context

    @property
    def loop(self) -> asyncio.AbstractEventLoop:
        """Get the event loop associated with the executor."""
        return self._loop

    def create_future(self) -> asyncio.Future:
        """Create an asyncio.Future attached to this executor's event loop."""
        return self._loop.create_future()

    async def __aenter__(self) -> 'AsyncioExecutor':
        return self

    async def __aexit__(
        self,
        _exc_type: Optional[type[BaseException]],
        _exc_val: Optional[BaseException],
        _exc_tb: Optional[object],
    ) -> None:
        await self.shutdown()

    def spin(self) -> None:
        """Block and process callbacks until shutdown."""
        self._loop.run_forever()


    def _clear_entities(self) -> List[asyncio.Task]:
        self._nodes.clear()
        self._update_entities_from_nodes()

        all_tasks = []
        for tasks in self._node_to_tasks.values():
            for task in tasks:
                task.cancel()
                all_tasks.append(task)
        self._node_to_tasks.clear()
        return all_tasks

    async def shutdown(self) -> None:
        """Clear all nodes and cancel pending tasks."""
        all_tasks = self._clear_entities()
        if all_tasks:
            await asyncio.gather(*all_tasks, return_exceptions=True)

    def __del__(self) -> None:
        if self._owns_loop and not self._loop.is_closed():
            self._loop.close()

    async def _gather_and_stop(self, tasks: List[asyncio.Task]) -> None:
        await asyncio.gather(*tasks, return_exceptions=True)
        if self._owns_loop:
            self._loop.stop()

    def _sync_shutdown(self) -> None:
        """Synchronous shutdown called by context on_shutdown."""
        all_tasks = self._clear_entities()

        if not all_tasks:
            return

        if self._loop.is_closed():
            warnings.warn(
                f'Event loop is closed but {len(all_tasks)} tasks are still pending. '
                'Call "await executor.shutdown()" before closing the event loop.',
                RuntimeWarning,
                stacklevel=2
            )
            return

        if self._loop.is_running():
            self._shutdown_task = self._loop.create_task(self._gather_and_stop(all_tasks))
        else:
            self._loop.run_until_complete(asyncio.gather(*all_tasks, return_exceptions=True))

    def _get_loop(self) -> asyncio.AbstractEventLoop:
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            self._owns_loop = True
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            return loop

    def wake(self) -> None:
        self._update_entities_from_nodes()

    def add_node(self, node: Node) -> bool:
        if node in self._nodes:
            return False

        self._nodes.add(node)
        self._node_to_tasks[node] = set()
        node.executor = self
        self._update_entities_from_nodes()
        return True

    async def remove_node(self, node: Node) -> None:
        if node not in self._nodes:
            return

        self._nodes.remove(node)
        self._update_entities_from_nodes()

        node_tasks = self._node_to_tasks.pop(node)
        for task in node_tasks:
            task.cancel()
        if node_tasks:
            await asyncio.gather(*node_tasks, return_exceptions=True)

    def _update_entities_from_nodes(self) -> None:
        new_subscriptions: Dict[Subscription, Node] = {}
        new_clients: Dict[Client, Node] = {}
        for node in self._nodes:
            new_subscriptions.update({sub: node for sub in node.subscriptions})
            new_clients.update({cli: node for cli in node.clients})

        self._update_entity_set(
            self._subscription_to_node,
            new_subscriptions,
            self._handle_added_subscription,
            self._handle_removed_subscription
        )
        self._update_entity_set(
            self._client_to_node,
            new_clients,
            self._handle_added_client,
            self._handle_removed_client
        )
    
    def _handle_added_subscription(self, sub: Subscription, node: Node):
        sub.handle.set_on_new_message_callback(
            partial(
                self._loop.call_soon_threadsafe,
                self._handle_ready_entity,
                self._take_subscription,
                sub,
                node,
            )
        )

    def _handle_removed_subscription(self, sub: Subscription):
        sub.handle.clear_on_new_message_callback()

    def _handle_added_client(self, client: Client, node: Node):
        # Warn about pre-existing requests that were created before node was added to executor.
        # Those requests returned rclpy.Future which cannot be awaited in asyncio.
        if client._pending_requests:
            warnings.warn(
                f'Client "{client.srv_name}" has {len(client._pending_requests)} pending '
                'request(s) created before being added to AsyncioExecutor. These requests '
                'returned rclpy.Future which cannot be awaited in asyncio. Add nodes to '
                'AsyncioExecutor before calling call_async().',
                RuntimeWarning,
                stacklevel=4
            )

        client.handle.set_on_new_response_callback(
            partial(
                self._loop.call_soon_threadsafe,
                self._handle_ready_entity,
                self._take_client,
                client,
                node,
            )
        )

    def _handle_removed_client(self, client: Client):
        client.handle.clear_on_new_response_callback()

    def _update_entity_set(
        self,
        current_entity_to_node: Dict[EntityT, Node],
        new_entity_to_node: Dict[EntityT, Node],
        on_added_entity: Callable[[EntityT, Node], None],
        on_removed_entity: Callable[[EntityT], None],
    ) -> bool:
        current_entities = set(current_entity_to_node.keys())
        new_entities = set(new_entity_to_node.keys())

        added_entities = new_entities - current_entities
        for entity in added_entities:
            node = new_entity_to_node[entity]
            current_entity_to_node[entity] = node
            entity.handle.__enter__()
            on_added_entity(entity, node)

        removed_entities = current_entities - new_entities
        for entity in removed_entities:
            on_removed_entity(entity)
            entity.handle.__exit__(None, None, None)
            del current_entity_to_node[entity]

        return bool(added_entities or removed_entities)
    
    def _handle_ready_entity(
        self,
        take_entity_callback: Callable[[EntityT], Optional[Coroutine]],
        entity: EntityT,
        node: Node,
        number_of_events: int,
    ) -> None:
        if node not in self._nodes:
            return

        tasks = self._node_to_tasks[node]
        for _ in range(number_of_events):
            callback = take_entity_callback(entity)
            if not callback:
                return

            task = self._loop.create_task(callback)
            task.add_done_callback(partial(self._done_callback, node))
            tasks.add(task)

    def _done_callback(
        self,
        node: Node,
        task: asyncio.Task,
    ) -> None:
        if task.cancelled():
            return

        self._node_to_tasks[node].remove(task)

        exc = task.exception()
        if exc:
            node.get_logger().error(''.join(traceback.format_exception(exc)))
