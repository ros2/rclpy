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

import time
import unittest

import rclpy

from test_msgs.msg import BasicTypes

TEST_NODE_NAMESPACE = 'test_node_ns'

TEST_TOPIC = 'my_topic'
TEST_TOPIC_FROM = 'topic_from'
TEST_TOPIC_TO = 'topic_to'
TEST_FQN_TOPIC_FROM = '/original/my_ns/my_topic'
TEST_FQN_TOPIC_TO = '/remapped/another_ns/new_topic'


class TestPublisher(unittest.TestCase):

    @classmethod
    def setUp(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node(
            'node',
            context=cls.context,
            cli_args=[
                '--ros-args', '-r', '{}:={}'.format(TEST_TOPIC_FROM, TEST_TOPIC_TO),
                '--ros-args', '-r', '{}:={}'.format(TEST_FQN_TOPIC_FROM, TEST_FQN_TOPIC_TO)
            ],
        )
        cls.node_with_ns = rclpy.create_node(
            'node_withns',
            context=cls.context,
            namespace=TEST_NODE_NAMESPACE,
        )

    @classmethod
    def tearDown(cls):
        cls.node.destroy_node()
        cls.node_with_ns.destroy_node()
        rclpy.shutdown(context=cls.context)

    @classmethod
    def do_test_topic_name(cls, test_topics, node):
        """
        Test the topic names of publishers created by the given node.

        The node will create publishers with topic in test_topics, and then test if
        the publisher's topic_name property is equal to the expected value.

        Args:
        ----
            test_topics: A list of binary tuple in the form (topic, expected topic), the
            former will be passed to node.create_publisher and the latter will be compared
            with publisher.topic_name
            node: The node used to create the publisher. The node's namespace will have
            an effect on the publisher's topic_name.

        """
        for topic, target_topic in test_topics:
            publisher = node.create_publisher(BasicTypes, topic, 0)
            assert publisher.topic_name == target_topic
            publisher.destroy()

    @classmethod
    def do_test_wait_for_all_acked(cls, test_topic, node, pub_qos, sub_qos):
        pub = node.create_publisher(BasicTypes, test_topic, pub_qos)
        sub = node.create_subscription(BasicTypes, test_topic, lambda msg: print(msg), sub_qos)
        time.sleep(1)
        assert pub.get_subscription_count() == 1
        basic_types_msg = BasicTypes()
        try:
            pub.publish(basic_types_msg)
        except Exception:
            raise AssertionError('Publish data failed')

        try:
            assert pub.wait_for_all_acked() is True
        except Exception:
            raise AssertionError('wait_for_all_acked raise exception')

        pub.destroy()
        sub.destroy()

    def test_topic_name(self):
        test_topics = [
            (TEST_TOPIC, '/' + TEST_TOPIC),
            ('/' + TEST_TOPIC, '/' + TEST_TOPIC),
            ('/my_ns/' + TEST_TOPIC, '/my_ns/' + TEST_TOPIC),
            ('my_ns/' + TEST_TOPIC, '/my_ns/' + TEST_TOPIC),
        ]
        TestPublisher.do_test_topic_name(test_topics, self.node)

        # topics in a node which has a namespace
        test_topics = [
            (TEST_TOPIC, '/' + TEST_NODE_NAMESPACE + '/' + TEST_TOPIC),
            ('/' + TEST_TOPIC, '/' + TEST_TOPIC),
            ('/my_ns/' + TEST_TOPIC, '/my_ns/' + TEST_TOPIC),
            ('my_ns/' + TEST_TOPIC, '/' + TEST_NODE_NAMESPACE + '/my_ns/' + TEST_TOPIC),
        ]
        TestPublisher.do_test_topic_name(test_topics, self.node_with_ns)

    def test_topic_name_remapping(self):
        test_topics = [
            (TEST_TOPIC_FROM, '/' + TEST_TOPIC_TO),
            ('/' + TEST_TOPIC_FROM, '/' + TEST_TOPIC_TO),
            ('/my_ns/' + TEST_TOPIC_FROM, '/my_ns/' + TEST_TOPIC_FROM),
            ('my_ns/' + TEST_TOPIC_FROM, '/my_ns/' + TEST_TOPIC_FROM),
            (TEST_FQN_TOPIC_FROM, TEST_FQN_TOPIC_TO),
        ]
        TestPublisher.do_test_topic_name(test_topics, self.node)

    def test_wait_for_all_acked(self):
        pub_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE)
        sub_qos = pub_qos

        # Publisher and subscription are all reliable
        TestPublisher.do_test_wait_for_all_acked(TEST_TOPIC, self.node, pub_qos, sub_qos)

        # Publisher and subscription are all best effort
        pub_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        sub_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        TestPublisher.do_test_wait_for_all_acked(TEST_TOPIC, self.node, pub_qos, sub_qos)

        # Publisher is reliable and subscription is best effort
        pub_qos.reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE
        TestPublisher.do_test_wait_for_all_acked(TEST_TOPIC, self.node, pub_qos, sub_qos)

        # Publisher is best effort and subscription is reliable
        # Not work.


if __name__ == '__main__':
    unittest.main()
