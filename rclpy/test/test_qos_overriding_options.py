# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import pytest

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos_overriding_options import _declare_qos_parameters
from rclpy.qos_overriding_options import _get_qos_policy_parameter
from rclpy.qos_overriding_options import InvalidQosOverridesError
from rclpy.qos_overriding_options import QosCallbackResult
from rclpy.qos_overriding_options import QoSOverridingOptions
from rclpy.qos_overriding_options import QoSPolicyKind


@pytest.fixture(autouse=True)
def init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_get_qos_policy_parameter():
    qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        lifespan=Duration(nanoseconds=1e3),
        deadline=Duration(nanoseconds=1e6),
        liveliness=QoSLivelinessPolicy.SYSTEM_DEFAULT,
        liveliness_lease_duration=Duration(nanoseconds=1e9)
        )
    value = _get_qos_policy_parameter(qos, QoSPolicyKind.HISTORY)
    assert value == 'keep_last'
    value = _get_qos_policy_parameter(qos, QoSPolicyKind.DEPTH)
    assert value == qos.depth
    value = _get_qos_policy_parameter(qos, QoSPolicyKind.RELIABILITY)
    assert value == 'reliable'
    value = _get_qos_policy_parameter(qos, QoSPolicyKind.DURABILITY)
    assert value == 'volatile'
    value = _get_qos_policy_parameter(qos, QoSPolicyKind.LIFESPAN)
    assert value == qos.lifespan.nanoseconds
    value = _get_qos_policy_parameter(qos, QoSPolicyKind.DEADLINE)
    assert value == qos.deadline.nanoseconds
    value = _get_qos_policy_parameter(qos, QoSPolicyKind.LIVELINESS)
    assert value == 'system_default'
    value = _get_qos_policy_parameter(qos, QoSPolicyKind.LIVELINESS_LEASE_DURATION)
    assert value == qos.liveliness_lease_duration.nanoseconds


def test_declare_qos_parameters():
    node = Node('my_node')
    _declare_qos_parameters(
        Publisher, node, '/my_topic', QoSProfile(depth=10),
        QoSOverridingOptions.with_default_policies()
    )
    qos_overrides = node.get_parameters_by_prefix('qos_overrides')
    assert len(qos_overrides) == 3
    expected_params = (
        ('/my_topic.publisher.depth', 10),
        ('/my_topic.publisher.history', 'keep_last'),
        ('/my_topic.publisher.reliability', 'reliable'),
    )
    for actual, expected in zip(
        sorted(qos_overrides.items(), key=lambda x: x[0]), expected_params
    ):
        assert actual[0] == expected[0]  # same param name
        assert actual[1].value == expected[1]  # same param value


def test_declare_qos_parameters_with_overrides():
    node = Node('my_node', parameter_overrides=[
        Parameter('qos_overrides./my_topic.publisher.depth', value=100),
        Parameter('qos_overrides./my_topic.publisher.reliability', value='best_effort'),
    ])
    for i in range(2):  # try twice, the second time the parameters will be get and not declared
        _declare_qos_parameters(
            Publisher, node, '/my_topic', QoSProfile(depth=10),
            QoSOverridingOptions.with_default_policies()
        )
        qos_overrides = node.get_parameters_by_prefix('qos_overrides')
        assert len(qos_overrides) == 3
        expected_params = (
            ('/my_topic.publisher.depth', 100),
            ('/my_topic.publisher.history', 'keep_last'),
            ('/my_topic.publisher.reliability', 'best_effort'),
        )
        for actual, expected in zip(
            sorted(qos_overrides.items(), key=lambda x: x[0]), expected_params
        ):
            assert actual[0] == expected[0]  # same param name
            assert actual[1].value == expected[1]  # same param value


def test_declare_qos_parameters_with_happy_callback():
    def qos_validation_callback(qos):
        result = QosCallbackResult()
        result.successful = True
        return result

    node = Node('my_node')
    _declare_qos_parameters(
        Publisher, node, '/my_topic', QoSProfile(depth=10),
        QoSOverridingOptions.with_default_policies(callback=qos_validation_callback)
    )
    qos_overrides = node.get_parameters_by_prefix('qos_overrides')
    assert len(qos_overrides) == 3
    expected_params = (
        ('/my_topic.publisher.depth', 10),
        ('/my_topic.publisher.history', 'keep_last'),
        ('/my_topic.publisher.reliability', 'reliable'),
    )
    for actual, expected in zip(
        sorted(qos_overrides.items(), key=lambda x: x[0]), expected_params
    ):
        assert actual[0] == expected[0]  # same param name
        assert actual[1].value == expected[1]  # same param value


def test_declare_qos_parameters_with_unhappy_callback():
    def qos_validation_callback(qos):
        result = QosCallbackResult()
        result.successful = False
        result.reason = 'my_custom_error_message'
        return result

    node = Node('my_node')

    with pytest.raises(InvalidQosOverridesError) as err:
        _declare_qos_parameters(
            Publisher, node, '/my_topic', QoSProfile(depth=10),
            QoSOverridingOptions.with_default_policies(callback=qos_validation_callback)
        )
    assert 'my_custom_error_message' in str(err.value)


def test_declare_qos_parameters_with_id():
    node = Node('my_node')
    _declare_qos_parameters(
        Publisher, node, '/my_topic', QoSProfile(depth=10),
        QoSOverridingOptions.with_default_policies(entity_id='i_have_an_id')
    )
    qos_overrides = node.get_parameters_by_prefix('qos_overrides')
    assert len(qos_overrides) == 3
    expected_params = (
        ('/my_topic.publisher_i_have_an_id.depth', 10),
        ('/my_topic.publisher_i_have_an_id.history', 'keep_last'),
        ('/my_topic.publisher_i_have_an_id.reliability', 'reliable'),
    )
    for actual, expected in zip(
        sorted(qos_overrides.items(), key=lambda x: x[0]), expected_params
    ):
        assert actual[0] == expected[0]  # same param name
        assert actual[1].value == expected[1]  # same param value
