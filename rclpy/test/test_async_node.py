# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import asyncio

import pytest

from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetLoggerLevels
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters

import rclpy
from rclpy.experimental import AsyncNode
from rclpy.qos import HistoryPolicy
from rclpy.qos import qos_profile_parameter_events
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from test_msgs.msg import BasicTypes
from test_msgs.msg import Strings
from test_msgs.srv import BasicTypes as BasicTypesSrv

from type_description_interfaces.srv import GetTypeDescription

TEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


@pytest.fixture(autouse=True)
def rclpy_context():
    """Initialize and shut down rclpy for each test."""
    with rclpy.init():
        yield


@pytest.mark.asyncio
async def test_lifecycle():
    """Node creates and destroys cleanly via async context manager."""
    async with AsyncNode('test_lifecycle_node'):
        pass


@pytest.mark.asyncio
async def test_direct_entity_destroy():
    """Destroying a subscription stops message delivery."""
    received = asyncio.Event()

    async def callback(msg):
        received.set()

    async with AsyncNode('test_direct_destroy_node') as node:
        pub = node.create_publisher(Strings, '/test_direct_destroy_topic', TEST_QOS)
        sub = node.create_subscription(
            Strings, '/test_direct_destroy_topic', callback, TEST_QOS)

        pub.publish(Strings(string_value='before'))
        async with asyncio.timeout(5):
            await received.wait()
        received.clear()

        sub.destroy()

        pub.publish(Strings(string_value='after'))
        with pytest.raises(TimeoutError):
            async with asyncio.timeout(0.5):
                await received.wait()


@pytest.mark.asyncio
async def test_create_before_aenter():
    """Entities created before entering async context dispatch after entry."""
    received = asyncio.Event()

    async def callback(msg):
        received.set()

    node = AsyncNode('test_create_before_aenter_node')
    pub = node.create_publisher(Strings, '/test_pre_aenter_topic', TEST_QOS)
    node.create_subscription(
        Strings, '/test_pre_aenter_topic', callback, TEST_QOS)

    async with node:
        pub.publish(Strings(string_value='hello'))
        async with asyncio.timeout(5):
            await received.wait()


@pytest.mark.asyncio
async def test_multiple_entities_two_nodes():
    """Two nodes each with a publisher and subscription communicate cross-node."""
    received_a = asyncio.Event()
    received_b = asyncio.Event()

    async def callback_a(msg):
        received_a.set()

    async def callback_b(msg):
        received_b.set()

    async with (
        AsyncNode('test_multi_node_a') as node_a,
        AsyncNode('test_multi_node_b') as node_b,
    ):
        pub_a = node_a.create_publisher(Strings, '/test_multi_a', TEST_QOS)
        node_b.create_subscription(Strings, '/test_multi_a', callback_a, TEST_QOS)

        pub_b = node_b.create_publisher(Strings, '/test_multi_b', TEST_QOS)
        node_a.create_subscription(Strings, '/test_multi_b', callback_b, TEST_QOS)

        async with asyncio.timeout(5):
            while (pub_a.get_subscription_count() < 1
                   or pub_b.get_subscription_count() < 1):
                await asyncio.sleep(0.05)

        pub_a.publish(Strings(string_value='a'))
        pub_b.publish(Strings(string_value='b'))

        async with asyncio.timeout(5):
            await received_a.wait()
            await received_b.wait()


@pytest.mark.asyncio
async def test_enable_logger_service():
    """Logger service is reachable over DDS when enabled."""
    async with (
        AsyncNode(
            'test_logger_svc_node', enable_logger_service=True
        ) as srv_node,
        AsyncNode('test_logger_client_node') as client_node,
    ):
        client = client_node.create_client(
            GetLoggerLevels,
            '/test_logger_svc_node/get_logger_levels')

        async with asyncio.timeout(5):
            await client.wait_for_service()

        logger_name = srv_node.get_logger().name
        req = GetLoggerLevels.Request(names=[logger_name])
        async with asyncio.timeout(5):
            resp = await client.call(req)
        assert len(resp.levels) == 1
        assert resp.levels[0].name == logger_name


@pytest.mark.asyncio
async def test_parameter_service_over_dds():
    """Get and set parameters over DDS between two AsyncNodes."""
    async with (
        AsyncNode(
            'test_param_srv_node',
            allow_undeclared_parameters=True,
        ) as srv_node,
        AsyncNode('test_param_client_node') as client_node,
    ):
        srv_node.declare_parameter('my_param', 42)

        get_client = client_node.create_client(
            GetParameters, '/test_param_srv_node/get_parameters')
        set_client = client_node.create_client(
            SetParameters, '/test_param_srv_node/set_parameters')

        async with asyncio.timeout(5):
            await get_client.wait_for_service()
            await set_client.wait_for_service()

        get_req = GetParameters.Request(names=['my_param'])
        async with asyncio.timeout(5):
            get_resp = await get_client.call(get_req)
        assert len(get_resp.values) == 1
        assert get_resp.values[0].integer_value == 42
        assert get_resp.values[0].type == ParameterType.PARAMETER_INTEGER

        set_req = SetParameters.Request(parameters=[
            ParameterMsg(
                name='my_param',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=99,
                ),
            ),
        ])
        async with asyncio.timeout(5):
            set_resp = await set_client.call(set_req)
        assert len(set_resp.results) == 1
        assert set_resp.results[0].successful is True

        async with asyncio.timeout(5):
            get_resp2 = await get_client.call(get_req)
        assert get_resp2.values[0].integer_value == 99


@pytest.mark.asyncio
async def test_run_basic():
    """run() blocks until destroy_node() is called."""
    node = AsyncNode('test_run_node')
    loop = asyncio.get_running_loop()
    loop.call_later(0.1, node.destroy_node)
    async with asyncio.timeout(5):
        await node.run()


@pytest.mark.asyncio
async def test_run_with_callback_shutdown():
    """run() returns when a callback calls destroy_node()."""
    received = asyncio.Event()

    node = AsyncNode('test_run_cb_shutdown_node')

    async def callback(msg):
        received.set()
        node.destroy_node()

    pub = node.create_publisher(Strings, '/test_run_cb_topic', TEST_QOS)
    node.create_subscription(Strings, '/test_run_cb_topic', callback, TEST_QOS)

    loop = asyncio.get_running_loop()
    loop.call_later(0.1, pub.publish, Strings(string_value='stop'))
    async with asyncio.timeout(5):
        await node.run()

    assert received.is_set()


@pytest.mark.asyncio
async def test_run_raises_if_already_running():
    """run() raises if node is already under async with."""
    async with AsyncNode('test_run_conflict_node') as node:
        with pytest.raises(RuntimeError):
            await node.run()


@pytest.mark.asyncio
async def test_type_description_service():
    """Verify type description service responds to requests on AsyncNode."""
    async with (
        AsyncNode('test_type_desc_srv_node') as srv_node,
        AsyncNode('test_type_desc_client_node') as client_node,
    ):
        topic = '/test_type_desc_basic_types'
        srv_node.create_publisher(BasicTypes, topic, 10)

        client = client_node.create_client(
            GetTypeDescription,
            '/test_type_desc_srv_node/get_type_description')

        async with asyncio.timeout(5):
            await client.wait_for_service()

            pub_infos = srv_node.get_publishers_info_by_topic(topic)
            assert len(pub_infos)
            type_hash = str(pub_infos[0].topic_type_hash)

            request = GetTypeDescription.Request(
                type_name='test_msgs/msg/BasicTypes',
                type_hash=type_hash,
                include_type_sources=True)
            response = await client.call(request)

        assert response.successful
        assert (response.type_description.type_description.type_name
                == 'test_msgs/msg/BasicTypes')
        assert len(response.type_sources)


@pytest.mark.asyncio
async def test_graph_discovery_methods():
    """Graph discovery methods are accessible on AsyncNode via BaseNode."""
    async with AsyncNode('test_graph_node', namespace='/test_ns') as node:
        topics = node.get_topic_names_and_types()
        assert isinstance(topics, list)

        services = node.get_service_names_and_types()
        assert isinstance(services, list)

        actions = node.get_action_names_and_types()
        assert isinstance(actions, list)

        names = node.get_node_names()
        assert 'test_graph_node' in names

        fq_names = node.get_fully_qualified_node_names()
        assert '/test_ns/test_graph_node' in fq_names

        names_ns = node.get_node_names_and_namespaces()
        assert any(n == 'test_graph_node' and ns == '/test_ns'
                   for n, ns in names_ns)

        names_ns_enc = node.get_node_names_and_namespaces_with_enclaves()
        assert isinstance(names_ns_enc, list)

        fq_name = node.get_fully_qualified_name()
        assert fq_name == '/test_ns/test_graph_node'


@pytest.mark.asyncio
async def test_count_methods():
    """Count methods work on AsyncNode."""
    async with AsyncNode('test_count_node') as node:
        topic = '/test_count_topic'
        node.create_publisher(Strings, topic, TEST_QOS)
        async def _noop(msg): pass
        node.create_subscription(Strings, topic, _noop, TEST_QOS)
        assert node.count_publishers(topic) == 1
        assert node.count_subscribers(topic) == 1


@pytest.mark.asyncio
async def test_endpoint_info_methods():
    """Endpoint info methods work on AsyncNode."""
    async with AsyncNode('test_endpoint_node') as node:
        node.create_publisher(BasicTypes, '/test_endpoint_topic', TEST_QOS)

        pub_info = node.get_publishers_info_by_topic('/test_endpoint_topic')
        assert len(pub_info) == 1
        assert pub_info[0].node_name == 'test_endpoint_node'

        async def _noop(msg): pass
        node.create_subscription(
            BasicTypes, '/test_endpoint_topic', _noop, TEST_QOS)
        sub_info = node.get_subscriptions_info_by_topic('/test_endpoint_topic')
        assert len(sub_info) == 1
        assert sub_info[0].node_name == 'test_endpoint_node'


@pytest.mark.asyncio
async def test_remote_node_introspection():
    """Remote node introspection methods work on AsyncNode."""
    async with AsyncNode('test_remote_node', namespace='/test_ns') as node:
        pubs = node.get_publisher_names_and_types_by_node(
            'test_remote_node', '/test_ns')
        assert isinstance(pubs, list)

        subs = node.get_subscriber_names_and_types_by_node(
            'test_remote_node', '/test_ns')
        assert isinstance(subs, list)

        svcs = node.get_service_names_and_types_by_node(
            'test_remote_node', '/test_ns')
        assert isinstance(svcs, list)

        clients = node.get_client_names_and_types_by_node(
            'test_remote_node', '/test_ns')
        assert isinstance(clients, list)


@pytest.mark.asyncio
async def test_wait_for_node_async():
    """Async wait_for_node finds an existing node."""
    async with (
        AsyncNode('test_wfn_target', namespace='/test_ns') as _,
        AsyncNode('test_wfn_watcher', namespace='/test_ns') as watcher,
    ):
        async with asyncio.timeout(3.0):
            await watcher.wait_for_node('/test_ns/test_wfn_target')


@pytest.mark.asyncio
async def test_wait_for_node_async_timeout():
    """Async wait_for_node raises TimeoutError for nonexistent node."""
    async with AsyncNode('test_wfn_timeout_node') as node:
        with pytest.raises(TimeoutError):
            async with asyncio.timeout(0.3):
                await node.wait_for_node('nonexistent_node')


@pytest.mark.asyncio
async def test_create_publisher_on_destroyed_node():
    """create_publisher raises RuntimeError on a destroyed node."""
    node = AsyncNode('test_destroyed_pub_node')
    node.destroy_node()
    with pytest.raises(RuntimeError):
        node.create_publisher(Strings, '/topic', TEST_QOS)


@pytest.mark.asyncio
async def test_create_subscription_on_destroyed_node():
    """create_subscription raises RuntimeError on a destroyed node."""
    node = AsyncNode('test_destroyed_sub_node')
    node.destroy_node()

    async def callback(msg):
        pass

    with pytest.raises(RuntimeError):
        node.create_subscription(Strings, '/topic', callback, TEST_QOS)


@pytest.mark.asyncio
async def test_create_client_on_destroyed_node():
    """create_client raises RuntimeError on a destroyed node."""
    node = AsyncNode('test_destroyed_client_node')
    node.destroy_node()
    with pytest.raises(RuntimeError):
        node.create_client(BasicTypesSrv, '/service')


@pytest.mark.asyncio
async def test_create_timer_on_destroyed_node():
    """create_timer raises RuntimeError on a destroyed node."""
    node = AsyncNode('test_destroyed_timer_node')
    node.destroy_node()

    async def callback():
        pass

    with pytest.raises(RuntimeError):
        node.create_timer(1.0, callback)


@pytest.mark.asyncio
async def test_destroy_node_idempotent():
    """Calling destroy_node() twice does not raise."""
    node = AsyncNode('test_double_destroy_node')
    node.destroy_node()
    node.destroy_node()


@pytest.mark.asyncio
async def test_entity_destroy_idempotent():
    """Calling destroy() twice on an entity does not raise."""
    async with AsyncNode('test_entity_double_destroy_node') as node:
        pub = node.create_publisher(Strings, '/topic', TEST_QOS)

        pub.destroy()
        pub.destroy()

        async def callback(msg):
            pass

        sub = node.create_subscription(Strings, '/topic', callback, TEST_QOS)

        sub.destroy()
        sub.destroy()


@pytest.mark.asyncio
async def test_aexit_destroys_on_exception():
    """Node is properly destroyed even when async-with body raises."""
    node = AsyncNode('test_aexit_exc_node')
    pub = node.create_publisher(Strings, '/topic', TEST_QOS)

    with pytest.raises(ExceptionGroup) as exc_info:
        async with node:
            raise RuntimeError('user error')

    assert exc_info.value.subgroup(RuntimeError)

    # Node should be destroyed — creating entities must fail
    with pytest.raises(RuntimeError):
        node.create_publisher(Strings, '/topic2', TEST_QOS)

    # Publisher should be destroyed — publishing must fail
    with pytest.raises(RuntimeError):
        pub.publish(Strings(string_value='nope'))


@pytest.mark.asyncio
async def test_shutdown_destroys_tracked_asyncnode():
    """context.shutdown() destroys AsyncNodes tracked via context.track_node()."""
    context = rclpy.Context()
    context.init()
    node = AsyncNode('test_safety_net_node', context=context)

    # Node is functional before shutdown
    node.create_publisher(Strings, '/test_safety_net_topic', TEST_QOS)

    context.shutdown()

    # Node was destroyed by the safety net — creating entities must fail
    with pytest.raises(RuntimeError):
        node.create_publisher(Strings, '/test_safety_net_topic2', TEST_QOS)

    # destroy_node() after the safety net is idempotent
    node.destroy_node()


@pytest.mark.asyncio
async def test_shutdown_destroys_running_asyncnode():
    """context.shutdown() during an active async-with session destroys the node."""
    context = rclpy.Context()
    context.init()
    callback_ran = asyncio.Event()

    async def tick():
        callback_ran.set()
        await asyncio.sleep(10)  # sleep for a long time

    node = AsyncNode('test_safety_net_running_node', context=context)
    node.create_timer(0.05, tick)

    async with asyncio.timeout(5):
        async with node:
            # Let the timer dispatch at least once
            await callback_ran.wait()
            # External shutdown fires the safety net while tasks are live
            context.shutdown()

    # Node was destroyed by the safety net mid-session
    with pytest.raises(RuntimeError):
        node.create_timer(0.05, tick)


@pytest.mark.asyncio
async def test_destroy_node_from_callback_under_async_with():
    """destroy_node() from a subscription callback inside async-with drains cleanly."""
    callback_ran = asyncio.Event()
    node = AsyncNode('test_aw_cb_shutdown_node')

    async def callback(msg):
        node.destroy_node()
        callback_ran.set()

    pub = node.create_publisher(Strings, '/test_aw_cb_topic', TEST_QOS)
    node.create_subscription(
        Strings, '/test_aw_cb_topic', callback, TEST_QOS)

    async with asyncio.timeout(5):
        async with node:
            pub.publish(Strings(string_value='bye'))
            await callback_ran.wait()

    with pytest.raises(RuntimeError):
        node.create_publisher(Strings, '/test_aw_cb_topic2', TEST_QOS)


@pytest.mark.asyncio
async def test_client_destroy_idempotent():
    """destroy() called twice on a client does not raise."""
    async with AsyncNode('test_client_idem_node') as node:
        client = node.create_client(BasicTypesSrv, '/test_client_idem_svc')
        client.destroy()
        client.destroy()


@pytest.mark.asyncio
async def test_service_destroy_idempotent():
    """destroy() called twice on a service does not raise."""
    async def handler(request, response):
        return response

    async with AsyncNode('test_service_idem_node') as node:
        service = node.create_service(
            BasicTypesSrv, '/test_service_idem_svc', handler)
        service.destroy()
        service.destroy()


@pytest.mark.asyncio
async def test_timer_destroy_idempotent():
    """destroy() called twice on a timer does not raise."""
    async def tick():
        pass

    async with AsyncNode('test_timer_idem_node') as node:
        timer = node.create_timer(1.0, tick)
        timer.destroy()
        timer.destroy()


@pytest.mark.asyncio
async def test_multi_node_aexit_on_body_exception():
    """Multi-node `async with a, b:` cleans both nodes when body raises."""
    a = AsyncNode('test_multi_aexit_a')
    b = AsyncNode('test_multi_aexit_b')

    with pytest.raises(ExceptionGroup):
        async with a, b:
            raise RuntimeError('user err')

    with pytest.raises(RuntimeError):
        a.create_publisher(Strings, '/t', TEST_QOS)
    with pytest.raises(RuntimeError):
        b.create_publisher(Strings, '/t', TEST_QOS)


@pytest.mark.asyncio
async def test_parameter_events_emitted():
    """Parameter event publisher emits on /parameter_events from AsyncNode."""
    received = []
    got_foo = asyncio.Event()

    async def callback(msg):
        if any(p.name == 'foo' for p in msg.new_parameters):
            received.append(msg)
            got_foo.set()

    async with (
        AsyncNode(
            'test_pevt_src_node',
            allow_undeclared_parameters=True,
        ) as src,
        AsyncNode('test_pevt_sink_node') as sink,
    ):
        sink.create_subscription(
            ParameterEvent, '/parameter_events', callback,
            qos_profile_parameter_events)

        async with asyncio.timeout(5):
            while src.count_subscribers('/parameter_events') < 1:
                await asyncio.sleep(0.05)

        src.declare_parameter('foo', 1)

        async with asyncio.timeout(5):
            await got_foo.wait()

    assert any(
        p.name == 'foo' for p in received[0].new_parameters)
