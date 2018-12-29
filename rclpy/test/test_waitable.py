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

import threading
import time
import unittest

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import check_for_type_support
from rclpy.qos import qos_profile_default
from rclpy.task import Future
from rclpy.waitable import NumberOfEntities
from rclpy.waitable import Waitable

from test_msgs.msg import Empty as EmptyMsg
from test_msgs.srv import Empty as EmptySrv


check_for_type_support(EmptyMsg)
check_for_type_support(EmptySrv)


class ClientWaitable(Waitable):

    def __init__(self, node):
        super().__init__(ReentrantCallbackGroup())

        self.client = _rclpy.rclpy_create_client(
            node.handle, EmptySrv, 'test_client', qos_profile_default.get_c_qos_profile())[0]
        self.client_index = None
        self.client_is_ready = False

        self.node = node
        self.future = None

    def is_ready(self, wait_set):
        """Return True if entities are ready in the wait set."""
        if _rclpy.rclpy_wait_set_is_ready('client', wait_set, self.client_index):
            self.client_is_ready = True
        return self.client_is_ready

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        if self.client_is_ready:
            self.client_is_ready = False
            return _rclpy.rclpy_take_response(self.client, EmptySrv.Response)
        return None

    async def execute(self, taken_data):
        """Execute work after data has been taken from a ready wait set."""
        test_data = {}
        if isinstance(taken_data[1], EmptySrv.Response):
            test_data['client'] = taken_data[1]
        self.future.set_result(test_data)

    def get_num_entities(self):
        """Return number of each type of entity used."""
        return NumberOfEntities(0, 0, 0, 1, 0)

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        self.client_index = _rclpy.rclpy_wait_set_add_entity('client', wait_set, self.client)


class ServerWaitable(Waitable):

    def __init__(self, node):
        super().__init__(ReentrantCallbackGroup())

        self.server = _rclpy.rclpy_create_service(
            node.handle, EmptySrv, 'test_server', qos_profile_default.get_c_qos_profile())[0]
        self.server_index = None
        self.server_is_ready = False

        self.node = node
        self.future = None

    def is_ready(self, wait_set):
        """Return True if entities are ready in the wait set."""
        if _rclpy.rclpy_wait_set_is_ready('service', wait_set, self.server_index):
            self.server_is_ready = True
        return self.server_is_ready

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        if self.server_is_ready:
            self.server_is_ready = False
            return _rclpy.rclpy_take_request(self.server, EmptySrv.Request)
        return None

    async def execute(self, taken_data):
        """Execute work after data has been taken from a ready wait set."""
        test_data = {}
        if isinstance(taken_data[0], EmptySrv.Request):
            test_data['server'] = taken_data[0]
        self.future.set_result(test_data)

    def get_num_entities(self):
        """Return number of each type of entity used."""
        return NumberOfEntities(0, 0, 0, 0, 1)

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        self.server_index = _rclpy.rclpy_wait_set_add_entity('service', wait_set, self.server)


class TimerWaitable(Waitable):

    def __init__(self, node):
        super().__init__(ReentrantCallbackGroup())

        self._clock = Clock(clock_type=ClockType.STEADY_TIME)
        period_nanoseconds = 10000
        self.timer = _rclpy.rclpy_create_timer(
            self._clock._clock_handle, node.context.handle, period_nanoseconds
        )[0]
        self.timer_index = None
        self.timer_is_ready = False

        self.node = node
        self.future = None

    def is_ready(self, wait_set):
        """Return True if entities are ready in the wait set."""
        if _rclpy.rclpy_wait_set_is_ready('timer', wait_set, self.timer_index):
            self.timer_is_ready = True
        return self.timer_is_ready

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        if self.timer_is_ready:
            self.timer_is_ready = False
            _rclpy.rclpy_call_timer(self.timer)
            return 'timer'
        return None

    async def execute(self, taken_data):
        """Execute work after data has been taken from a ready wait set."""
        test_data = {}
        if taken_data is 'timer':
            test_data['timer'] = taken_data
        self.future.set_result(test_data)

    def get_num_entities(self):
        """Return number of each type of entity used."""
        return NumberOfEntities(0, 0, 1, 0, 0)

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        self.timer_index = _rclpy.rclpy_wait_set_add_entity('timer', wait_set, self.timer)


class SubscriptionWaitable(Waitable):

    def __init__(self, node):
        super().__init__(ReentrantCallbackGroup())

        self.guard_condition = _rclpy.rclpy_create_guard_condition(node.context.handle)[0]
        self.guard_condition_index = None
        self.guard_is_ready = False

        self.subscription = _rclpy.rclpy_create_subscription(
            node.handle, EmptyMsg, 'test_topic', qos_profile_default.get_c_qos_profile())[0]
        self.subscription_index = None
        self.subscription_is_ready = False

        self.node = node
        self.future = None

    def is_ready(self, wait_set):
        """Return True if entities are ready in the wait set."""
        if _rclpy.rclpy_wait_set_is_ready('subscription', wait_set, self.subscription_index):
            self.subscription_is_ready = True
        return self.subscription_is_ready

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        if self.subscription_is_ready:
            self.subscription_is_ready = False
            return _rclpy.rclpy_take(self.subscription, EmptyMsg)
        return None

    async def execute(self, taken_data):
        """Execute work after data has been taken from a ready wait set."""
        test_data = {}
        if isinstance(taken_data, EmptyMsg):
            test_data['subscription'] = taken_data
        self.future.set_result(test_data)

    def get_num_entities(self):
        """Return number of each type of entity used."""
        return NumberOfEntities(1, 0, 0, 0, 0)

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        self.subscription_index = _rclpy.rclpy_wait_set_add_entity(
            'subscription', wait_set, self.subscription)


class GuardConditionWaitable(Waitable):

    def __init__(self, node):
        super().__init__(ReentrantCallbackGroup())

        self.guard_condition = _rclpy.rclpy_create_guard_condition(node.context.handle)[0]
        self.guard_condition_index = None
        self.guard_is_ready = False

        self.node = node
        self.future = None

    def is_ready(self, wait_set):
        """Return True if entities are ready in the wait set."""
        if _rclpy.rclpy_wait_set_is_ready('guard_condition', wait_set, self.guard_condition_index):
            self.guard_is_ready = True
        return self.guard_is_ready

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        if self.guard_is_ready:
            self.guard_is_ready = False
            return 'guard_condition'
        return None

    async def execute(self, taken_data):
        """Execute work after data has been taken from a ready wait set."""
        test_data = {}
        if taken_data is 'guard_condition':
            test_data['guard_condition'] = True
        self.future.set_result(test_data)

    def get_num_entities(self):
        """Return number of each type of entity used."""
        return NumberOfEntities(0, 1, 0, 0, 0)

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        self.guard_condition_index = _rclpy.rclpy_wait_set_add_entity(
            'guard_condition', wait_set, self.guard_condition)


class TestWaitable(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('TestWaitable', namespace='/rclpy/test', context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def start_spin_thread(self, waitable):
        waitable.future = Future(executor=self.executor)
        self.thr = threading.Thread(
            target=self.executor.spin_until_future_complete, args=(waitable.future,), daemon=True)
        self.thr.start()
        return self.thr

    def setUp(self):
        pass

    def tearDown(self):
        self.node.remove_waitable(self.waitable)

    def test_waitable_with_client(self):
        self.waitable = ClientWaitable(self.node)
        self.node.add_waitable(self.waitable)

        server = self.node.create_service(EmptySrv, 'test_client', lambda req, resp: resp)

        while not _rclpy.rclpy_service_server_is_available(self.node.handle, self.waitable.client):
            time.sleep(0.1)

        thr = self.start_spin_thread(self.waitable)
        _rclpy.rclpy_send_request(self.waitable.client, EmptySrv.Request())
        thr.join()

        assert self.waitable.future.done()
        assert isinstance(self.waitable.future.result()['client'], EmptySrv.Response)
        self.node.destroy_service(server)

    def test_waitable_with_server(self):
        self.waitable = ServerWaitable(self.node)
        self.node.add_waitable(self.waitable)
        client = self.node.create_client(EmptySrv, 'test_server')

        thr = self.start_spin_thread(self.waitable)
        client.call_async(EmptySrv.Request())
        thr.join()

        assert self.waitable.future.done()
        assert isinstance(self.waitable.future.result()['server'], EmptySrv.Request)
        self.node.destroy_client(client)

    def test_waitable_with_timer(self):
        self.waitable = TimerWaitable(self.node)
        self.node.add_waitable(self.waitable)

        thr = self.start_spin_thread(self.waitable)
        thr.join()

        assert self.waitable.future.done()
        assert self.waitable.future.result()['timer']

    def test_waitable_with_subscription(self):
        self.waitable = SubscriptionWaitable(self.node)
        self.node.add_waitable(self.waitable)
        pub = self.node.create_publisher(EmptyMsg, 'test_topic')

        thr = self.start_spin_thread(self.waitable)
        pub.publish(EmptyMsg())
        thr.join()

        assert self.waitable.future.done()
        assert isinstance(self.waitable.future.result()['subscription'], EmptyMsg)
        self.node.destroy_publisher(pub)

    def test_waitable_with_guard_condition(self):
        self.waitable = GuardConditionWaitable(self.node)
        self.node.add_waitable(self.waitable)

        thr = self.start_spin_thread(self.waitable)
        _rclpy.rclpy_trigger_guard_condition(self.waitable.guard_condition)
        thr.join()

        assert self.waitable.future.done()
        assert self.waitable.future.result()['guard_condition']


class TestNumberOfEntities(unittest.TestCase):

    def test_add(self):
        n1 = NumberOfEntities(1, 2, 3, 4, 5)
        n2 = NumberOfEntities(10, 20, 30, 40, 50)
        n = n1 + n2
        assert n.num_subscriptions == 11
        assert n.num_guard_conditions == 22
        assert n.num_timers == 33
        assert n.num_clients == 44
        assert n.num_services == 55

    def test_add_assign(self):
        n1 = NumberOfEntities(1, 2, 3, 4, 5)
        n2 = NumberOfEntities(10, 20, 30, 40, 50)
        n1 += n2
        assert n1.num_subscriptions == 11
        assert n1.num_guard_conditions == 22
        assert n1.num_timers == 33
        assert n1.num_clients == 44
        assert n1.num_services == 55
