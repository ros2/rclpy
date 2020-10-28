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
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.qos import QoSPolicyKind, QoSProfile
from rclpy.qos_overriding_options import _declare_qos_parameteres
from rclpy.qos_overriding_options import InvalidQosOverridesError
from rclpy.qos_overriding_options import QoSOverridingOptions


def test_declare_qos_parameters():
    rclpy.init()
    node = Node("my_node")
    _declare_qos_parameteres(
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
    for actual, expected in zip (
        sorted(qos_overrides.items(), key=lambda x: x[0]), expected_params
    ):
        assert actual[0] == expected[0]  # same param name
        assert actual[1].value == expected[1]  # same param value

    rclpy.shutdown()


def test_declare_qos_parameters_with_overrides():
    rclpy.init()
    node = Node("my_node", parameter_overrides=[
        Parameter('qos_overrides./my_topic.publisher.depth', value=100),
        Parameter('qos_overrides./my_topic.publisher.reliability', value='best_effort'),
    ])
    _declare_qos_parameteres(
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
    for actual, expected in zip (
        sorted(qos_overrides.items(), key=lambda x: x[0]), expected_params
    ):
        assert actual[0] == expected[0]  # same param name
        assert actual[1].value == expected[1]  # same param value

    rclpy.shutdown()


def test_declare_qos_parameters_with_happy_callback():
    rclpy.init()
    node = Node("my_node")
    _declare_qos_parameteres(
        Publisher, node, '/my_topic', QoSProfile(depth=10),
        QoSOverridingOptions.with_default_policies(callback=lambda x: True)
    )
    qos_overrides = node.get_parameters_by_prefix('qos_overrides')
    assert len(qos_overrides) == 3
    expected_params = (
        ('/my_topic.publisher.depth', 10),
        ('/my_topic.publisher.history', 'keep_last'),
        ('/my_topic.publisher.reliability', 'reliable'),
    )
    rclpy.shutdown()


def test_declare_qos_parameters_with_unhappy_callback():
    rclpy.init()
    node = Node("my_node")

    with pytest.raises(InvalidQosOverridesError):
        _declare_qos_parameteres(
            Publisher, node, '/my_topic', QoSProfile(depth=10),
            QoSOverridingOptions.with_default_policies(callback=lambda x: False)
        )
    rclpy.shutdown()


def test_declare_qos_parameters_with_id():
    rclpy.init()
    node = Node("my_node")
    _declare_qos_parameteres(
        Publisher, node, '/my_topic', QoSProfile(depth=10),
        QoSOverridingOptions.with_default_policies(id='i_have_an_id')
    )
    qos_overrides = node.get_parameters_by_prefix('qos_overrides')
    assert len(qos_overrides) == 3
    expected_params = (
        ('/my_topic.publisher_i_have_an_id.depth', 10),
        ('/my_topic.publisher_i_have_an_id.history', 'keep_last'),
        ('/my_topic.publisher_i_have_an_id.reliability', 'reliable'),
    )
    for actual, expected in zip (
        sorted(qos_overrides.items(), key=lambda x: x[0]), expected_params
    ):
        assert actual[0] == expected[0]  # same param name
        assert actual[1].value == expected[1]  # same param value

    rclpy.shutdown()
