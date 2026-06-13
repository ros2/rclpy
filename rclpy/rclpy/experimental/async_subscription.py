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
from collections.abc import AsyncGenerator
from collections.abc import Coroutine
from typing import Any, Callable, Optional, Type
from typing import cast

from rclpy.executors import await_or_execute
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.subscription import BaseSubscription, SubscriptionCallbackUnion
from rclpy.subscription import MessageAndInfo
from rclpy.subscription import SubscriptionCallback
from rclpy.subscription import SubscriptionCallbackWithMessageInfo
from rclpy.type_support import MsgT


class AsyncSubscription(BaseSubscription[MsgT]):
    """
    A subscription to a ROS topic.

    .. admonition:: Experimental

       This API is experimental.
    """

    def __init__(
        self,
        subscription_impl: '_rclpy.Subscription[MsgT]',
        msg_type: Type[MsgT],
        topic: str,
        callback: SubscriptionCallbackUnion[MsgT],
        qos_profile: QoSProfile,
        on_destroy: Callable[['AsyncSubscription[MsgT]'], None],
        raw: bool = False,
        concurrent: bool = False,
        tg: Optional[asyncio.TaskGroup] = None,
    ) -> None:
        super().__init__(subscription_impl, msg_type, topic, callback, qos_profile, raw,
                         on_destroy=on_destroy)
        self._concurrent = concurrent
        self._task: Optional[asyncio.Task[None]] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._read_event = asyncio.Event()
        if tg is not None:
            self._task = tg.create_task(self._run())

    def _on_new_message(self, _num_waiting: int) -> None:
        assert self._loop is not None
        self._loop.call_soon_threadsafe(self._read_event.set)

    def _destroy(self) -> None:
        if self._task is not None:
            self._task.cancel()
        self.handle.clear_on_new_message_callback()
        super()._destroy()

    async def _messages(self) -> AsyncGenerator[MessageAndInfo[MsgT], None]:
        """Async generator yielding (msg, msg_info) from DDS."""
        self.handle.set_on_new_message_callback(self._on_new_message)
        while not self._destroyed:
            msg_and_info = self.handle.take_message(
                self.msg_type, self.raw)
            if msg_and_info is not None:
                yield msg_and_info
            else:
                self._read_event.clear()
                await self._read_event.wait()

    def _make_callback(self, msg_and_info: MessageAndInfo[MsgT]) -> Coroutine[Any, Any, None]:
        """Create a callback coroutine from a (msg, msg_info) tuple."""
        if self._callback_type is BaseSubscription.CallbackType.MessageOnly:
            callback_msg = cast(SubscriptionCallback[MsgT], self.callback)
            return await_or_execute(callback_msg, msg_and_info[0])
        else:
            callback_msg_and_info = cast(SubscriptionCallbackWithMessageInfo[MsgT], self.callback)
            return await_or_execute(callback_msg_and_info, *msg_and_info)

    async def _run(self) -> None:
        """DDS bridge read loop for subscriptions."""
        self._loop = asyncio.get_running_loop()
        try:
            if self._concurrent:
                async with asyncio.TaskGroup() as tg:
                    async for msg_and_info in self._messages():
                        tg.create_task(self._make_callback(msg_and_info))
            else:
                async for msg_and_info in self._messages():
                    await self._make_callback(msg_and_info)
        finally:
            self._task = None
            self.destroy()
