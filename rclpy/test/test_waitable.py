# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import unittest

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import check_for_type_support
from rclpy.qos import qos_profile_default
from rclpy.waitable import NumberOfEntities
from rclpy.waitable import Waitable

from test_msgs.msg import Empty as EmptyMsg


check_for_type_support(EmptyMsg)


class DummyWaitable(Waitable):

    def __init__(self, node):
        super().__init__(ReentrantCallbackGroup())
        # Set up a number of guard conditions, and make sure they're waited on

        # List of pycapsule for guard conditions
        self.guard_condition = _rclpy.rclpy_create_guard_condition()[0]
        self.guard_condition_index = None
        self.guard_is_ready = False
        self.subscription = _rclpy.rclpy_create_subscription(
            node.handle, EmptyMsg, 'test_topic', qos_profile_default.get_c_qos_profile())
        self.subscription_condition_index = None
        self.subscription_is_ready = False
        self.node = node

    def is_ready(self, wait_set):
        """Return True if entities are ready in the wait set."""
        for guard_idx, ws_idx in self.indexes.items():
            # TODO(sloretz) create this API for the purpose of testing this
            if _rclpy.rclpy_wait_set_is_ready('guard_condition', ws_idx):
                self.ready_guards.append(self.guards[guard_idx])
        return self.ready_guards

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        if self.subscription_is_ready:
            return _rclpy.rclpy_take(self.subscription, EmptyMsg)
        return None

    async def execute(self, taken_data):
        """Execute work after data has been taken from a ready wait set."""
        if taken_data is None:
            # Guard condition
            # TODO(sloretz) set flag for guard condition
            pass
        else:
            # Subscription
            # TODO(sloretz) set flag for taken subscription
            pass

    def get_num_entities(self):
        """Return number of each type of entity used."""
        return NumberOfEntities(1, 1, 0, 0, 0)

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        self.guard_condition_index = _rclpy.rclpy_wait_set_add_entity(
            'guard_condition', wait_set, self.guard_condition)
        self.subscription_index = _rclpy.rclpy_wait_set_add_entity(
            'subscription', wait_set, self.subscription)


class TestWaitable(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('TestWaitable', namespace='/rclpy/test')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_waitable(self):

        waitable = DummyWaitable(self.node)
        self.node.add_waitable(waitable)

        # TODO(sloretz) spin and make sure it calls waitable.execute
