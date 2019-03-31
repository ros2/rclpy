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

from typing import Callable
from typing import List
from typing import Optional
from typing import Tuple
from typing import TypeVar

import weakref

from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import CallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.clock import Clock
from rclpy.clock import ROSClock
from rclpy.constants import S_TO_NS
from rclpy.context import Context
from rclpy.exceptions import NotInitializedException
from rclpy.executors import Executor
from rclpy.expand_topic_name import expand_topic_name
from rclpy.guard_condition import GuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from rclpy.parameter_service import ParameterService
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_default
from rclpy.qos import qos_profile_parameter_events
from rclpy.qos import qos_profile_services_default
from rclpy.qos import QoSProfile
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.time_source import TimeSource
from rclpy.timer import WallTimer
from rclpy.type_support import check_for_type_support
from rclpy.utilities import get_default_context
from rclpy.validate_full_topic_name import validate_full_topic_name
from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name
from rclpy.validate_topic_name import validate_topic_name
from rclpy.waitable import Waitable

HIDDEN_NODE_PREFIX = '_'

# Used for documentation purposes only
MsgType = TypeVar('MsgType')
SrvType = TypeVar('SrvType')
SrvTypeRequest = TypeVar('SrvTypeRequest')
SrvTypeResponse = TypeVar('SrvTypeResponse')


class Node:
    """
    A Node in the ROS graph.

    A Node is the primary entrypoint in a ROS system for communication.
    It can be used to create ROS entities such as publishers, subscribers, services, etc.
    """

    def __init__(
        self,
        node_name: str,
        *,
        context: Context = None,
        cli_args: List[str] = None,
        namespace: str = None,
        use_global_arguments: bool = True,
        start_parameter_services: bool = True,
        initial_parameters: List[Parameter] = None
    ) -> None:
        """
        Constructor.

        :param node_name: A name to give to this node. Validated by :func:`validate_node_name`.
        :param context: The context to be associated with, or ``None`` for the default global
            context.
        :param cli_args: A list of strings of command line args to be used only by this node.
        :param namespace: The namespace to which relative topic and service names will be prefixed.
            Validated by :func:`validate_namespace`.
        :param use_global_arguments: ``False`` if the node should ignore process-wide command line
            args.
        :param start_parameter_services: ``False`` if the node should not create parameter
            services.
        :param initial_parameters: A list of parameters to be set during node creation.
        """
        self._handle = None
        self._context = get_default_context() if context is None else context
        self._parameters: dict = {}
        self.publishers: List[Publisher] = []
        self.subscriptions: List[Subscription] = []
        self.clients: List[Client] = []
        self.services: List[Service] = []
        self.timers: List[WallTimer] = []
        self.guards: List[GuardCondition] = []
        self.waitables: List[Waitable] = []
        self._default_callback_group = MutuallyExclusiveCallbackGroup()
        self._parameters_callback = None

        namespace = namespace or ''
        if not self._context.ok():
            raise NotInitializedException('cannot create node')
        try:
            self._handle = _rclpy.rclpy_create_node(
                node_name, namespace, self._context.handle, cli_args, use_global_arguments)
        except ValueError:
            # these will raise more specific errors if the name or namespace is bad
            validate_node_name(node_name)
            # emulate what rcl_node_init() does to accept '' and relative namespaces
            if not namespace:
                namespace = '/'
            if not namespace.startswith('/'):
                namespace = '/' + namespace
            validate_namespace(namespace)
            # Should not get to this point
            raise RuntimeError('rclpy_create_node failed for unknown reason')
        self._logger = get_logger(_rclpy.rclpy_get_node_logger_name(self.handle))

        # Clock that has support for ROS time.
        self._clock = ROSClock()
        self._time_source = TimeSource(node=self)
        self._time_source.attach_clock(self._clock)

        self.__executor_weakref = None

        self._parameter_event_publisher = self.create_publisher(
            ParameterEvent, 'parameter_events', qos_profile=qos_profile_parameter_events)

        node_parameters = _rclpy.rclpy_get_node_parameters(Parameter, self.handle)
        # Combine parameters from params files with those from the node constructor and
        # use the set_parameters_atomically API so a parameter event is published.
        if initial_parameters is not None:
            node_parameters.update({p.name: p for p in initial_parameters})
        self.set_parameters_atomically(node_parameters.values())

        if start_parameter_services:
            self._parameter_service = ParameterService(self)

    @property
    def executor(self) -> Optional[Executor]:
        """Get the executor if the node has been added to one, else return ``None``."""
        if self.__executor_weakref:
            return self.__executor_weakref()
        return None

    @executor.setter
    def executor(self, new_executor: Executor) -> None:
        """Set or change the executor the node belongs to."""
        current_executor = self.executor
        if current_executor == new_executor:
            return
        if current_executor is not None:
            current_executor.remove_node(self)
        if new_executor is None:
            self.__executor_weakref = None
        else:
            new_executor.add_node(self)
            self.__executor_weakref = weakref.ref(new_executor)

    @property
    def context(self) -> Context:
        """Get the context associated with the node."""
        return self._context

    @property
    def default_callback_group(self) -> CallbackGroup:
        """
        Get the default callback group.

        If no other callback group is provided when the a ROS entity is created with the node,
        then it is added to the default callback group.
        """
        return self._default_callback_group

    @property
    def handle(self):
        """
        Get the handle to the underlying `rcl_node_t`.

        Cannot be modified after node creation.

        :raises AttributeError: if modified after creation.
        """
        return self._handle

    @handle.setter
    def handle(self, value):
        raise AttributeError('handle cannot be modified after node creation')

    def get_name(self) -> str:
        """Get the name of the node."""
        return _rclpy.rclpy_get_node_name(self.handle)

    def get_namespace(self) -> str:
        """Get the namespace of the node."""
        return _rclpy.rclpy_get_node_namespace(self.handle)

    def get_clock(self) -> Clock:
        """Get the clock used by the node."""
        return self._clock

    def get_logger(self):
        """Get the nodes logger."""
        return self._logger

    def get_parameters(self, names: List[str]) -> List[Parameter]:
        """
        Get a list of parameters.

        :param names: The names of the parameters to get.
        :return: The values for the given parameter names.
        """
        if not all(isinstance(name, str) for name in names):
            raise TypeError('All names must be instances of type str')
        return [self.get_parameter(name) for name in names]

    def get_parameter(self, name: str) -> Parameter:
        """
        Get a parameter by name.

        :param name: The name of the parameter.
        :return: The value of the parameter.
        """
        if name not in self._parameters:
            return Parameter(name, Parameter.Type.NOT_SET, None)
        return self._parameters[name]

    def set_parameters(self, parameter_list: List[Parameter]) -> List[SetParametersResult]:
        """
        Set parameters for the node.

        If a callback was registered previously with :func:`set_parameters_callback`, it will be
        called prior to setting the parameters for the node.
        For each successfully set parameter, a :class:`ParameterEvent` message is
        published.

        :param parameter_list: The list of parameters to set.
        :return: A list of SetParametersResult messages.
        """
        results = []
        for param in parameter_list:
            if not isinstance(param, Parameter):
                raise TypeError("parameter must be instance of type '{}'".format(repr(Parameter)))
            results.append(self.set_parameters_atomically([param]))
        return results

    def set_parameters_atomically(self, parameter_list: List[Parameter]) -> SetParametersResult:
        """
        Atomically set parameters for the node.

        If a callback was registered previously with :func:`set_parameters_callback`, it will be
        called prior to setting the parameters for the node.
        If the parameters are set successfully, a :class:`ParameterEvent` message is
        published.

        :param parameter_list: The list of parameters to set.
        """
        result = None
        if self._parameters_callback:
            result = self._parameters_callback(parameter_list)
        else:
            result = SetParametersResult(successful=True)

        if result.successful:
            parameter_event = ParameterEvent()
            # Add fully qualified path of node to parameter event
            if self.get_namespace() == '/':
                parameter_event.node = self.get_namespace() + self.get_name()
            else:
                parameter_event.node = self.get_namespace() + '/' + self.get_name()
            for param in parameter_list:
                if Parameter.Type.NOT_SET == param.type_:
                    if Parameter.Type.NOT_SET != self.get_parameter(param.name).type_:
                        # Parameter deleted. (Parameter had value and new value is not set)
                        parameter_event.deleted_parameters.append(
                            param.to_parameter_msg())
                    # Delete any unset parameters regardless of their previous value.
                    # We don't currently store NOT_SET parameters so this is an extra precaution.
                    if param.name in self._parameters:
                        del self._parameters[param.name]
                else:
                    if Parameter.Type.NOT_SET == self.get_parameter(param.name).type_:
                        #  Parameter is new. (Parameter had no value and new value is set)
                        parameter_event.new_parameters.append(param.to_parameter_msg())
                    else:
                        # Parameter changed. (Parameter had a value and new value is set)
                        parameter_event.changed_parameters.append(
                            param.to_parameter_msg())
                    self._parameters[param.name] = param
            parameter_event.stamp = self._clock.now().to_msg()
            self._parameter_event_publisher.publish(parameter_event)

        return result

    def set_parameters_callback(
        self,
        callback: Callable[[List[Parameter]], SetParametersResult]
    ) -> None:
        """
        Register a set parameters callback.

        Calling this function with override any previously registered callback.

        :param callback: The function that is called whenever parameters are set for the node.
        """
        self._parameters_callback = callback

    def _validate_topic_or_service_name(self, topic_or_service_name, *, is_service=False):
        name = self.get_name()
        namespace = self.get_namespace()
        validate_node_name(name)
        validate_namespace(namespace)
        validate_topic_name(topic_or_service_name, is_service=is_service)
        expanded_topic_or_service_name = expand_topic_name(topic_or_service_name, name, namespace)
        validate_full_topic_name(expanded_topic_or_service_name, is_service=is_service)

    def add_waitable(self, waitable: Waitable) -> None:
        """
        Add a class that is capable of adding things to the wait set.

        :param waitable: An instance of a waitable that the node will add to the waitset.
        """
        self.waitables.append(waitable)

    def remove_waitable(self, waitable: Waitable) -> None:
        """
        Remove a Waitable that was previously added to the node.

        :param waitable: The Waitable to remove.
        """
        self.waitables.remove(waitable)

    def create_publisher(
        self,
        msg_type,
        topic: str,
        *,
        qos_profile: QoSProfile = qos_profile_default
    ) -> Publisher:
        """
        Create a new publisher.

        :param msg_type: The type of ROS messages the publisher will publish.
        :param topic: The name of the topic the publisher will publish to.
        :param qos_profile: The quality of service profile to apply to the publisher.
        :return: The new publisher.
        """
        # this line imports the typesupport for the message module if not already done
        check_for_type_support(msg_type)
        failed = False
        try:
            publisher_handle = _rclpy.rclpy_create_publisher(
                self.handle, msg_type, topic, qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(topic)
        publisher = Publisher(publisher_handle, msg_type, topic, qos_profile, self.handle)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(
        self,
        msg_type,
        topic: str,
        callback: Callable[[MsgType], None],
        *,
        qos_profile: QoSProfile = qos_profile_default,
        callback_group: CallbackGroup = None,
        raw: bool = False
    ) -> Subscription:
        """
        Create a new subscription.

        :param msg_type: The type of ROS messages the subscription will subscribe to.
        :param topic: The name of the topic the subscription will subscribe to.
        :param callback: A user-defined callback function that is called when a message is
            received by the subscription.
        :param qos_profile: The quality of service profile to apply to the subscription.
        :param callback_group: The callback group for the subscription. If ``None``, then the
            nodes default callback group is used.
        :param raw: If ``True``, then received messages will be stored in raw binary
            representation.
        """
        if callback_group is None:
            callback_group = self.default_callback_group
        # this line imports the typesupport for the message module if not already done
        check_for_type_support(msg_type)
        failed = False
        try:
            [subscription_handle, subscription_pointer] = _rclpy.rclpy_create_subscription(
                self.handle, msg_type, topic, qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(topic)

        subscription = Subscription(
            subscription_handle, subscription_pointer, msg_type,
            topic, callback, callback_group, qos_profile, self.handle, raw)
        self.subscriptions.append(subscription)
        callback_group.add_entity(subscription)
        return subscription

    def create_client(
        self,
        srv_type,
        srv_name: str,
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
        callback_group: CallbackGroup = None
    ) -> Client:
        """
        Create a new service client.

        :param srv_type: The service type.
        :param srv_name: The name of the service.
        :param qos_profile: The quality of service profile to apply the service client.
        :param callback_group: The callback group for the service client. If ``None``, then the
            nodes default callback group is used.
        """
        if callback_group is None:
            callback_group = self.default_callback_group
        check_for_type_support(srv_type)
        failed = False
        try:
            [client_handle, client_pointer] = _rclpy.rclpy_create_client(
                self.handle,
                srv_type,
                srv_name,
                qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(srv_name, is_service=True)
        client = Client(
            self.handle, self.context,
            client_handle, client_pointer, srv_type, srv_name, qos_profile,
            callback_group)
        self.clients.append(client)
        callback_group.add_entity(client)
        return client

    def create_service(
        self,
        srv_type,
        srv_name: str,
        callback: Callable[[SrvTypeRequest, SrvTypeResponse], SrvTypeResponse],
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
        callback_group: CallbackGroup = None
    ) -> Service:
        """
        Create a new service server.

        :param srv_type: The service type.
        :param srv_name: The name of the service.
        :param callback: A user-defined callback function that is called when a service request
            received by the server.
        :param qos_profile: The quality of service profile to apply the service server.
        :param callback_group: The callback group for the service server. If ``None``, then the
            nodes default callback group is used.
        """
        if callback_group is None:
            callback_group = self.default_callback_group
        check_for_type_support(srv_type)
        failed = False
        try:
            [service_handle, service_pointer] = _rclpy.rclpy_create_service(
                self.handle,
                srv_type,
                srv_name,
                qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(srv_name, is_service=True)
        service = Service(
            self.handle, service_handle, service_pointer,
            srv_type, srv_name, callback, callback_group, qos_profile)
        self.services.append(service)
        callback_group.add_entity(service)
        return service

    def create_timer(
        self,
        timer_period_sec: float,
        callback: Callable,
        callback_group: CallbackGroup = None
    ) -> WallTimer:
        """
        Create a new timer.

        The timer will be started and every ``timer_period_sec`` number of seconds the provided
        callback function will be called.

        :param timer_period_sec: The period (s) of the timer.
        :param callback: A user-defined callback function that is called when the timer expires.
        :param callback_group: The callback group for the timer. If ``None``, then the nodes
            default callback group is used.
        """
        timer_period_nsec = int(float(timer_period_sec) * S_TO_NS)
        if callback_group is None:
            callback_group = self.default_callback_group
        timer = WallTimer(callback, callback_group, timer_period_nsec, context=self.context)

        self.timers.append(timer)
        callback_group.add_entity(timer)
        return timer

    def create_guard_condition(
        self,
        callback: Callable,
        callback_group: CallbackGroup = None
    ) -> GuardCondition:
        """Create a new guard condition."""
        if callback_group is None:
            callback_group = self.default_callback_group
        guard = GuardCondition(callback, callback_group, context=self.context)

        self.guards.append(guard)
        callback_group.add_entity(guard)
        return guard

    def destroy_publisher(self, publisher: Publisher) -> bool:
        """
        Destroy a publisher created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        for pub in self.publishers:
            if pub.publisher_handle == publisher.publisher_handle:
                _rclpy.rclpy_destroy_node_entity(pub.publisher_handle, self.handle)
                self.publishers.remove(pub)
                return True
        return False

    def destroy_subscription(self, subscription: Subscription) -> bool:
        """
        Destroy a subscription created by the node.

        :return: ``True`` if succesful, ``False`` otherwise.
        """
        for sub in self.subscriptions:
            if sub.subscription_handle == subscription.subscription_handle:
                _rclpy.rclpy_destroy_node_entity(sub.subscription_handle, self.handle)
                self.subscriptions.remove(sub)
                return True
        return False

    def destroy_client(self, client: Client) -> bool:
        """
        Destroy a service client created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        for cli in self.clients:
            if cli.client_handle == client.client_handle:
                _rclpy.rclpy_destroy_node_entity(cli.client_handle, self.handle)
                self.clients.remove(cli)
                return True
        return False

    def destroy_service(self, service: Service) -> bool:
        """
        Destroy a service server created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        for srv in self.services:
            if srv.service_handle == service.service_handle:
                _rclpy.rclpy_destroy_node_entity(srv.service_handle, self.handle)
                self.services.remove(srv)
                return True
        return False

    def destroy_timer(self, timer: WallTimer) -> bool:
        """
        Destroy a timer created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        for tmr in self.timers:
            if tmr.timer_handle == timer.timer_handle:
                _rclpy.rclpy_destroy_entity(tmr.timer_handle)
                # TODO(sloretz) Store clocks on node and destroy them separately
                _rclpy.rclpy_destroy_entity(tmr.clock._clock_handle)
                self.timers.remove(tmr)
                return True
        return False

    def destroy_guard_condition(self, guard: GuardCondition) -> bool:
        """
        Destroy a guard condition created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        for gc in self.guards:
            if gc.guard_handle == guard.guard_handle:
                _rclpy.rclpy_destroy_entity(gc.guard_handle)
                self.guards.remove(gc)
                return True
        return False

    def destroy_node(self) -> bool:
        """
        Destroy the node.

        Frees resources used by the node, including any entities created by the following methods:

        * :func:`create_publisher`
        * :func:`create_subscription`
        * :func:`create_client`
        * :func:`create_service`
        * :func:`create_timer`
        * :func:`create_guard_condition`

        :return: ``True`` if successful, ``False`` otherwise.
        """
        ret = True
        if self.handle is None:
            return ret

        # Drop extra reference to parameter event publisher.
        # It will be destroyed with other publishers below.
        self._parameter_event_publisher = None

        while self.publishers:
            pub = self.publishers.pop()
            _rclpy.rclpy_destroy_node_entity(pub.publisher_handle, self.handle)
        while self.subscriptions:
            sub = self.subscriptions.pop()
            _rclpy.rclpy_destroy_node_entity(sub.subscription_handle, self.handle)
        while self.clients:
            cli = self.clients.pop()
            _rclpy.rclpy_destroy_node_entity(cli.client_handle, self.handle)
        while self.services:
            srv = self.services.pop()
            _rclpy.rclpy_destroy_node_entity(srv.service_handle, self.handle)
        while self.timers:
            tmr = self.timers.pop()
            _rclpy.rclpy_destroy_entity(tmr.timer_handle)
            # TODO(sloretz) Store clocks on node and destroy them separately
            _rclpy.rclpy_destroy_entity(tmr.clock._clock_handle)
        while self.guards:
            gc = self.guards.pop()
            _rclpy.rclpy_destroy_entity(gc.guard_handle)

        _rclpy.rclpy_destroy_entity(self.handle)
        self._handle = None
        return ret

    def get_publisher_names_and_types_by_node(
        self,
        node_name: str,
        node_namespace: str,
        no_demangle: bool = False
    ) -> List[Tuple[str, str]]:
        """
        Get a list of discovered topics for publishers of a remote node.

        :param node_name: Name of a remote node to get publishers for.
        :param node_namespace: Namespace of the remote node.
        :param no_demangle: If ``True``, then topic names and types returned will not be demangled.
        :return: List of tuples containing two strings: the topic name and topic type.
        """
        return _rclpy.rclpy_get_publisher_names_and_types_by_node(
            self.handle, no_demangle, node_name, node_namespace)

    def get_subscriber_names_and_types_by_node(
        self,
        node_name: str,
        node_namespace: str,
        no_demangle: bool = False
    ) -> List[Tuple[str, str]]:
        """
        Get a list of discovered topics for subscriptions of a remote node.

        :param node_name: Name of a remote node to get subscriptions for.
        :param node_namespace: Namespace of the remote node.
        :param no_demangle: If ``True``, then topic names and types returned will not be demangled.
        :return: List of tuples containing two strings: the topic name and topic type.
        """
        return _rclpy.rclpy_get_subscriber_names_and_types_by_node(
            self.handle, no_demangle, node_name, node_namespace)

    def get_service_names_and_types_by_node(
        self,
        node_name: str,
        node_namespace: str
    ) -> List[Tuple[str, str]]:
        """
        Get a list of discovered service topics for a remote node.

        :param node_name: Name of a remote node to get services for.
        :param node_namespace: Namespace of the remote node.
        :return: List of tuples containing two strings: the service name and service type.
        """
        return _rclpy.rclpy_get_service_names_and_types_by_node(
            self.handle, node_name, node_namespace)

    def get_topic_names_and_types(self, no_demangle: bool = False) -> List[Tuple[str, str]]:
        """
        Get a list topic names and types for the node.

        :param no_demangle: If ``True``, then topic names and types returned will not be demangled.
        :return: List of tuples containing two strings: the topic name and topic type.
        """
        return _rclpy.rclpy_get_topic_names_and_types(self.handle, no_demangle)

    def get_service_names_and_types(self) -> List[Tuple[str, str]]:
        """
        Get a list of service topics for the node.

        :return: List of tuples containing two strings: the service name and service type.
        """
        return _rclpy.rclpy_get_service_names_and_types(self.handle)

    def get_node_names(self) -> List[str]:
        """
        Get a list of names for discovered nodes.

        :return: List of node names.
        """
        names_ns = _rclpy.rclpy_get_node_names_and_namespaces(self.handle)
        return [n[0] for n in names_ns]

    def get_node_names_and_namespaces(self) -> List[Tuple[str, str]]:
        """
        Get a list of names and namespaces for discovered nodes.

        :return: List of tuples containing two strings: the node name and node namespace.
        """
        return _rclpy.rclpy_get_node_names_and_namespaces(self.handle)

    def _count_publishers_or_subscribers(self, topic_name, func):
        fq_topic_name = expand_topic_name(topic_name, self.get_name(), self.get_namespace())
        validate_topic_name(fq_topic_name)
        return func(self.handle, fq_topic_name)

    def count_publishers(self, topic_name: str) -> int:
        """
        Return the number of publishers on a given topic.

        `topic_name` may be a relative, private, or fully qualifed topic name.
        A relative or private topic is expanded using this node's namespace and name.
        The queried topic name is not remapped.

        :param topic_name: the topic_name on which to count the number of publishers.
        :return: the number of publishers on the topic.
        """
        return self._count_publishers_or_subscribers(topic_name, _rclpy.rclpy_count_publishers)

    def count_subscribers(self, topic_name: str) -> int:
        """
        Return the number of subscribers on a given topic.

        `topic_name` may be a relative, private, or fully qualifed topic name.
        A relative or private topic is expanded using this node's namespace and name.
        The queried topic name is not remapped.

        :param topic_name: the topic_name on which to count the number of subscribers.
        :return: the number of subscribers on the topic.
        """
        return self._count_publishers_or_subscribers(topic_name, _rclpy.rclpy_count_subscribers)

    def __del__(self):
        self.destroy_node()
