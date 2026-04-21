# Copyright 2026 Open Source Robotics Foundation, Inc.
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
from typing import Callable, Dict, Optional, Type

from rclpy.client import BaseClient
from rclpy.context import Context
from rclpy.qos import QoSProfile
from rclpy.type_support import Srv, SrvRequestT, SrvResponseT


class AsyncClient(BaseClient[SrvRequestT, SrvResponseT]):
    """
    A client of a ROS service.

    .. admonition:: Experimental

       This API is experimental.
    """

    def __init__(
        self,
        context: Context,
        client_impl: object,
        srv_type: Type[Srv[SrvRequestT, SrvResponseT]],
        srv_name: str,
        qos_profile: QoSProfile,
        on_destroy: Callable[['AsyncClient'], None],
        tg: Optional[asyncio.TaskGroup] = None,
    ) -> None:
        super().__init__(context, client_impl, srv_type, srv_name, qos_profile,
                         on_destroy=on_destroy)
        self._pending_requests: Dict[int, asyncio.Future] = {}
        self._task: Optional[asyncio.Task] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._read_event = asyncio.Event()
        if tg is not None:
            self._task = tg.create_task(self._run())

    def _on_new_response(self, _num_waiting: int) -> None:
        assert self._loop is not None
        self._loop.call_soon_threadsafe(self._read_event.set)

    async def wait_for_service(self, *, check_interval: float = 0.1) -> None:
        """
        Wait for a service server to become ready.

        To apply a timeout, wrap the call with ``async with asyncio.timeout()``.

        :param check_interval: Seconds between checks. Defaults to 0.1.
        """
        while not self.service_is_ready():
            await asyncio.sleep(check_interval)

    def _destroy(self) -> None:
        if self._task is not None:
            self._task.cancel()
        self.handle.clear_on_new_response_callback()
        for future in self._pending_requests.values():
            future.cancel()
        super()._destroy()

    async def call(self, request: SrvRequestT) -> SrvResponseT:
        """Send a service request and await the response."""
        if self._destroyed:
            raise RuntimeError('Calling a destroyed client is forbidden')
        loop = asyncio.get_running_loop()
        future: asyncio.Future[SrvResponseT] = loop.create_future()
        sequence_number = self.handle.send_request(request)
        self._pending_requests[sequence_number] = future
        try:
            return await future
        finally:
            self._pending_requests.pop(sequence_number)

    async def _responses(self):
        """Async generator yielding (header, response) from DDS."""
        self.handle.set_on_new_response_callback(self._on_new_response)
        while not self._destroyed:
            header_and_response = self.handle.take_response(
                self.srv_type.Response)
            if header_and_response != (None, None):
                yield header_and_response
            else:
                self._read_event.clear()
                await self._read_event.wait()

    async def _run(self) -> None:
        """DDS bridge response loop for clients."""
        self._loop = asyncio.get_running_loop()
        try:
            async for header, response in self._responses():
                future = self._pending_requests.get(
                    header.request_id.sequence_number)
                if future is not None:
                    future.set_result(response)
        finally:
            self._task = None
            self.destroy()
