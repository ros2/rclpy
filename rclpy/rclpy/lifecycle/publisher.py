# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from typing import Union

from rclpy.publisher import MsgType
from rclpy.publisher import Publisher

from .managed_entity import SimpleManagedEntity


class LifecyclePublisher(SimpleManagedEntity, Publisher):
    """Managed publisher entity."""

    def __init__(self, *args, **kwargs):
        SimpleManagedEntity.__init__(self)
        Publisher.__init__(self, *args, **kwargs)

    @SimpleManagedEntity.when_enabled
    def publish(self, msg: Union[MsgType, bytes]) -> None:
        """
        Publish a message if the lifecycle publisher is enabled.

        See rclpy.publisher.Publisher.publish() for more details.
        """
        Publisher.publish(self, msg)
