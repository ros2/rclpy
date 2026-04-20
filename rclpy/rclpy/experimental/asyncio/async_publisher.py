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

from typing import Callable, Type

from rclpy.publisher import BasePublisher
from rclpy.qos import QoSProfile
from rclpy.type_support import MsgT


class AsyncPublisher(BasePublisher[MsgT]):
    """
    A publisher on a ROS topic.

    .. admonition:: Experimental

       This API is experimental.
    """

    def __init__(
        self,
        publisher_impl: object,
        msg_type: Type[MsgT],
        topic: str,
        qos_profile: QoSProfile,
        on_destroy: Callable[['AsyncPublisher'], None],
    ) -> None:
        super().__init__(publisher_impl, msg_type, topic, qos_profile,
                         on_destroy=on_destroy)
