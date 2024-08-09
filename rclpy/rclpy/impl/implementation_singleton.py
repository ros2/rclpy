# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

"""
Provide singleton access to the rclpy C modules.

For example, you might use it like this:

.. code::

    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

    _rclpy.rclpy_init()
    while _rclpy.rclpy_ok():
        # ...
"""

from rpyutils import import_c_library

if TYPE_CHECKING:
    from rclpy.client import ClientHandle
    from rclpy.context import ContextHandle
    from rclpy.event_handler import EventHandle
    from rclpy.guard_condition import GuardConditionHandle
    from rclpy.service import ServiceHandle
    from rclpy.subscription import SubscriptionHandle
    from rclpy.timer import TimerHandle
    from rclpy.wait_set import WaitSetHandle

    class rclpyHandle(Protocol):

        def rclpy_remove_ros_args(self, pycli_args: Sequence[str]) -> List[str]: ...

        def rclpy_get_rmw_implementation_identifier(self) -> str: ...

        Client: Type[ClientHandle]
        Context: Type[ContextHandle]
        EventHandle: Type[EventHandle]
        GuardCondition: Type[GuardConditionHandle]
        Service: Type[ServiceHandle]
        Subscription: Type[SubscriptionHandle]
        Timer: Type[TimerHandle]
        WaitSet: Type[WaitSetHandle]


package = 'rclpy'

rclpy_implementation = import_c_library('._rclpy_pybind11', package)
