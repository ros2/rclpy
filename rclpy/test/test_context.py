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

from rclpy.context import Context


def test_on_shutdown_method():
    context = Context()
    context.init()
    assert context.ok()

    callback_called = False

    class SomeClass:

        def on_shutdown(self):
            nonlocal callback_called
            callback_called = True

    instance = SomeClass()
    context.on_shutdown(instance.on_shutdown)

    context.shutdown()
    assert not context.ok()

    assert callback_called


def test_on_shutdown_function():
    context = Context()
    context.init()
    assert context.ok()

    callback_called = False

    def on_shutdown():
        nonlocal callback_called
        callback_called = True

    context.on_shutdown(on_shutdown)

    context.shutdown()
    assert not context.ok()

    assert callback_called


def test_context_manager():
    context = Context()

    assert not context.ok(), 'the context should not be ok() before init() is called'

    with context as the_context:
        # Make sure the correct instance is returned
        assert the_context is context

        assert context.ok(), 'the context should now be initialized'

    assert not context.ok(), 'the context should now be shut down'

    # Make sure it does not raise (smoke test)
    context.try_shutdown()
