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

import threading
import time

import pytest
import rclpy
from rclpy.exceptions import InvalidHandle
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
        node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def test_destroy_node_while_spinning():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        node = rclpy.create_node('test_node1', context=context)

        def spin():
            with pytest.raises(rclpy.executors.ExternalShutdownException):
                rclpy.spin(node, executor)

        thread = threading.Thread(
                target=spin,
                daemon=True)
        thread.start()
        try:
            # Let the spin thread get going
            time.sleep(0.5)
            node.destroy_node()

            # Make sure the spin thread has time to react to the
            # destruction of the node before stopping the test
            time.sleep(0.5)
            context.shutdown()
        finally:
            thread.join()
    finally:
        rclpy.try_shutdown(context=context)


def test_destroy_timers():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node('test_node3', context=context)
        try:
            timer1 = node.create_timer(0.1, None)
            timer2 = node.create_timer(1, None)
            timer2  # noqa

            assert 2 == len(tuple(node.timers))
            assert node.destroy_timer(timer1)

            assert 1 == len(tuple(node.timers))
        finally:
            node.destroy_node()
        assert 0 == len(tuple(node.timers))
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
            assert 1 == len(tuple(node.timers))
            pub1 = node.create_publisher(BasicTypes, 'pub1_topic', 1)
            assert 2 == len(tuple(node.publishers))
            pub2 = node.create_publisher(BasicTypes, 'pub2_topic', 1)
            pub2  # noqa
            assert 3 == len(tuple(node.publishers))
            sub1 = node.create_subscription(
                BasicTypes, 'sub1_topic', lambda msg: ..., 1)
            assert 1 == len(tuple(node.subscriptions))
            sub2 = node.create_subscription(
                BasicTypes, 'sub2_topic', lambda msg: ..., 1)
            sub2  # noqa
            assert 2 == len(tuple(node.subscriptions))

            assert node.destroy_publisher(pub1)
            assert 2 == len(tuple(node.publishers))

            assert node.destroy_subscription(sub1)
            assert 1 == len(tuple(node.subscriptions))
        finally:
            node.destroy_node()
        assert 0 == len(tuple(node.timers))
        assert 0 == len(tuple(node.publishers))
        assert 0 == len(tuple(node.subscriptions))
    finally:
        rclpy.shutdown(context=context)


def test_destroy_subscription_asap():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    try:
        node = rclpy.create_node('test_destroy_subscription_asap', context=context)
        try:
            sub = node.create_subscription(BasicTypes, 'sub_topic', lambda msg: ..., 1)

            # handle valid
            with sub.handle:
                pass

            node.destroy_subscription(sub)

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

        node.destroy_node()

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
            pub = node.create_publisher(BasicTypes, 'pub_topic', 1)

            # handle valid
            with pub.handle:
                pass

            node.destroy_publisher(pub)

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

            node.destroy_client(client)

            with pytest.raises(InvalidHandle):
                # handle invalid because it was destroyed
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

            node.destroy_service(service)

            with pytest.raises(InvalidHandle):
                # handle invalid because it was destroyed
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

            node.destroy_timer(timer)

            with pytest.raises(InvalidHandle):
                # handle invalid because it was destroyed
                with timer.handle:
                    pass
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
