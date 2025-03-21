# Copyright 2024-2025 Brad Martin
# Copyright 2024 Merlin Labs, Inc.
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
import faulthandler
import typing

import rclpy.executors
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


# Try to look like we inherit from the rclpy Executor for type checking purposes without
# getting any of the code from the base class.
def EventsExecutor(*, context: typing.Optional[rclpy.Context] = None) -> rclpy.executors.Executor:
    if context is None:
        context = rclpy.get_default_context()

    # For debugging purposes, if anything goes wrong in C++ make sure we also get a
    # Python backtrace dumped with the crash.
    faulthandler.enable()

    ex = typing.cast(rclpy.executors.Executor, _rclpy.EventsExecutor(context))

    # rclpy.Executor does this too.  Note, the context itself is smart enough to check
    # for bound methods, and check whether the instances they're bound to still exist at
    # callback time, so we don't have to worry about tearing down this stale callback at
    # destruction time.
    # TODO(bmartin427) This should really be done inside of the EventsExecutor
    # implementation itself, but I'm unable to figure out a pybind11 incantation that
    # allows me to pass this bound method call from C++.
    context.on_shutdown(ex.wake)

    return ex
