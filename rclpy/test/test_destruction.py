# Copyright 2016 Open Source Robotics Foundation, Inc.
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
from rclpy.handle import InvalidHandle
from test_msgs.msg import BasicTypes
from test_msgs.srv import BasicTypes as BasicTypesSrv


def test_destroy_node():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('test_node1', context=context)
        node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def test_destroy_node_twice():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('test_node2', context=context)
        node.destroy_node()
        with pytest.raises(InvalidHandle):
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def test_destroy_timers():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('test_node3', context=context)
        try:
            timer1 = node.create_timer(0.1, None)
            timer2 = node.create_timer(1, None)
            timer2  # noqa

            assert 2 == len(node.timers)
            assert node.destroy_timer(timer1)

            assert 1 == len(node.timers)
        finally:
            node.destroy_node()
        assert 0 == len(node.timers)
    finally:
        rclpy.shutdown(context=context)


def test_destroy_entities():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('test_node4', context=context)
        try:
            timer = node.create_timer(0.1, None)
            timer  # noqa
            assert 1 == len(node.timers)
            pub1 = node.create_publisher(BasicTypes, 'pub1_topic')
            assert 2 == len(node.publishers)
            pub2 = node.create_publisher(BasicTypes, 'pub2_topic')
            pub2  # noqa
            assert 3 == len(node.publishers)
            sub1 = node.create_subscription(
                BasicTypes, 'sub1_topic', lambda msg: ...)
            assert 1 == len(node.subscriptions)
            sub2 = node.create_subscription(
                BasicTypes, 'sub2_topic', lambda msg: ...)
            sub2  # noqa
            assert 2 == len(node.subscriptions)

            assert node.destroy_publisher(pub1)
            assert 2 == len(node.publishers)

            assert node.destroy_subscription(sub1)
            assert 1 == len(node.subscriptions)
        finally:
            node.destroy_node()
        assert 0 == len(node.timers)
        assert 0 == len(node.publishers)
        assert 0 == len(node.subscriptions)
    finally:
        rclpy.shutdown(context=context)


def test_destroy_subscription_asap():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    try:
        node = rclpy.create_node('test_destroy_subscription_asap', context=context)
        try:
            sub = node.create_subscription(BasicTypes, 'sub_topic', lambda msg: ...)

            # handle valid
            with sub.handle:
                pass

            with sub.handle:
                node.destroy_subscription(sub)
                # handle valid because it's still being used
                with sub.handle:
                    pass

            with pytest.raises(InvalidHandle):
                # handle invalid because it was destroyed when no one was using it
                with sub.handle:
                    pass
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def test_destroy_node_asap():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    try:
        node = rclpy.create_node('test_destroy_subscription_asap', context=context)
        with node.handle:
            node.destroy_node()
            # handle valid because it's still being used
            with node.handle:
                pass

        with pytest.raises(InvalidHandle):
            # handle invalid because it was destroyed when no one was using it
            with node.handle:
                pass
    finally:
        rclpy.shutdown(context=context)


def test_destroy_publisher_asap():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    try:
        node = rclpy.create_node('test_destroy_publisher_asap', context=context)
        try:
            pub = node.create_publisher(BasicTypes, 'pub_topic')

            # handle valid
            with pub.handle:
                pass

            with pub.handle:
                node.destroy_publisher(pub)
                # handle valid because it's still being used
                with pub.handle:
                    pass

            with pytest.raises(InvalidHandle):
                # handle invalid because it was destroyed when no one was using it
                with pub.handle:
                    pass
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def test_destroy_client_asap():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    try:
        node = rclpy.create_node('test_destroy_client_asap', context=context)
        try:
            client = node.create_client(BasicTypesSrv, 'cli_service')

            # handle valid
            with client.handle:
                pass

            with client.handle:
                node.destroy_client(client)
                # handle valid because it's still being used
                with client.handle:
                    pass

            with pytest.raises(InvalidHandle):
                # handle invalid because it was destroyed when no one was using it
                with client.handle:
                    pass
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def test_destroy_service_asap():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    try:
        node = rclpy.create_node('test_destroy_service_asap', context=context)
        try:
            service = node.create_service(BasicTypesSrv, 'srv_service', lambda req, res: ...)

            # handle valid
            with service.handle:
                pass

            with service.handle:
                node.destroy_service(service)
                # handle valid because it's still being used
                with service.handle:
                    pass

            with pytest.raises(InvalidHandle):
                # handle invalid because it was destroyed when no one was using it
                with service.handle:
                    pass
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def test_destroy_timer_asap():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    try:
        node = rclpy.create_node('test_destroy_timer_asap', context=context)
        try:
            timer = node.create_timer(1.0, lambda: ...)

            # handle valid
            with timer.handle:
                pass

            with timer.handle:
                node.destroy_timer(timer)
                # handle valid because it's still being used
                with timer.handle:
                    pass

            with pytest.raises(InvalidHandle):
                # handle invalid because it was destroyed when no one was using it
                with timer.handle:
                    pass
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
