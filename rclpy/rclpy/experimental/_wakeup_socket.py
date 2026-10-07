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
import socket


class _WakeProtocol(asyncio.Protocol):

    def __init__(self, event: asyncio.Event) -> None:
        self._event = event

    def data_received(self, _data: bytes) -> None:
        self._event.set()


class WakeupSocket:

    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        transport: asyncio.BaseTransport,
        rsock: socket.socket,
        wsock: socket.socket
    ) -> None:
        self._loop = loop
        self._transport = transport
        self._rsock = rsock
        self._wsock = wsock

    @classmethod
    async def create(cls, event: asyncio.Event) -> 'WakeupSocket':
        loop = asyncio.get_running_loop()
        rsock, wsock = socket.socketpair()
        try:
            wsock.setblocking(False)
            transport, _ = await loop.create_connection(
                lambda: _WakeProtocol(event), sock=rsock)
        except BaseException:
            rsock.close()
            wsock.close()
            raise
        return cls(loop, transport, rsock, wsock)

    def fileno(self) -> int:
        return self._wsock.fileno()

    def close(self) -> None:
        self._wsock.close()
        self._transport.close()
