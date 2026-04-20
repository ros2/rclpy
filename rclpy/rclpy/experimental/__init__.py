# Copyright 2024-2025 Brad Martin
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

import sys

from .events_executor import EventsExecutor

__all__ = [
    'EventsExecutor'
]

if sys.version_info >= (3, 12):
    from .async_client import AsyncClient
    from .async_clock import AsyncClock
    from .async_node import AsyncNode
    from .async_publisher import AsyncPublisher
    from .async_service import AsyncService
    from .async_subscription import AsyncSubscription
    from .async_timer import AsyncTimer

    __all__ += [
        'AsyncClient',
        'AsyncClock',
        'AsyncNode',
        'AsyncPublisher',
        'AsyncService',
        'AsyncSubscription',
        'AsyncTimer'
    ]
