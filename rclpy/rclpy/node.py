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

import math
import time

from typing import Any
from typing import Callable
from typing import Dict
from typing import Iterator
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple
from typing import Type
from typing import TypeVar
from typing import Union

import warnings
import weakref

from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ListParametersResult
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import ListParameters

from rclpy.callback_groups import CallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.clock import Clock
from rclpy.clock import ROSClock
from rclpy.constants import S_TO_NS
from rclpy.context import Context
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.exceptions import InvalidHandle
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.exceptions import InvalidParameterValueException
from rclpy.exceptions import InvalidTopicNameException
from rclpy.exceptions import NotInitializedException
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.exceptions import ParameterImmutableException
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import Executor
from rclpy.expand_topic_name import expand_topic_name
from rclpy.guard_condition import GuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.logging import get_logger
from rclpy.logging_service import LoggingService
from rclpy.parameter import Parameter, PARAMETER_SEPARATOR_STRING
from rclpy.parameter_service import ParameterService
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_parameter_events
from rclpy.qos import qos_profile_services_default
from rclpy.qos import QoSProfile
from rclpy.qos_overriding_options import _declare_qos_parameters
from rclpy.qos_overriding_options import QoSOverridingOptions
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.time_source import TimeSource
from rclpy.timer import Rate
from rclpy.timer import Timer
from rclpy.topic_endpoint_info import TopicEndpointInfo
from rclpy.type_description_service import TypeDescriptionService
from rclpy.type_support import check_is_valid_msg_type
from rclpy.type_support import check_is_valid_srv_type
from rclpy.utilities import get_default_context
from rclpy.validate_full_topic_name import validate_full_topic_name
from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name
from rclpy.validate_parameter_name import validate_parameter_name
from rclpy.validate_topic_name import validate_topic_name
from rclpy.waitable import Waitable

HIDDEN_NODE_PREFIX = '_'

# Used for documentation purposes only
MsgType = TypeVar('MsgType')
SrvType = TypeVar('SrvType')
SrvTypeRequest = TypeVar('SrvTypeRequest')
SrvTypeResponse = TypeVar('SrvTypeResponse')

# Re-export exception defined in _rclpy C extension.
# `Node.get_*_names_and_types_by_node` methods may raise this error.
NodeNameNonExistentError = _rclpy.NodeNameNonExistentError


class Node:
    """
    A Node in the ROS graph.

    A Node is the primary entrypoint in a ROS system for communication.
    It can be used to create ROS entities such as publishers, subscribers, services, etc.
    """

    PARAM_REL_TOL = 1e-6
    """
    Relative tolerance for floating point parameter values' comparison.
    See `math.isclose` documentation.
    """

    def __init__(
        self,
        node_name: str,
        *,
        context: Optional[Context] = None,
        cli_args: Optional[List[str]] = None,
        namespace: Optional[str] = None,
        use_global_arguments: bool = True,
        enable_rosout: bool = True,
        start_parameter_services: bool = True,
        parameter_overrides: Optional[List[Parameter]] = None,
        allow_undeclared_parameters: bool = False,
        automatically_declare_parameters_from_overrides: bool = False,
        enable_logger_service: bool = False
    ) -> None:
        """
        Create a Node.

        :param node_name: A name to give to this node. Validated by :func:`validate_node_name`.
        :param context: The context to be associated with, or ``None`` for the default global
            context.
        :param cli_args: A list of strings of command line args to be used only by this node.
            These arguments are used to extract remappings used by the node and other ROS specific
            settings, as well as user defined non-ROS arguments.
        :param namespace: The namespace to which relative topic and service names will be prefixed.
            Validated by :func:`validate_namespace`.
        :param use_global_arguments: ``False`` if the node should ignore process-wide command line
            args.
        :param enable_rosout: ``False`` if the node should ignore rosout logging.
        :param start_parameter_services: ``False`` if the node should not create parameter
            services.
        :param parameter_overrides: A list of overrides for initial values for parameters declared
            on the node.
        :param allow_undeclared_parameters: True if undeclared parameters are allowed.
            This flag affects the behavior of parameter-related operations.
        :param automatically_declare_parameters_from_overrides: If True, the "parameter overrides"
            will be used to implicitly declare parameters on the node during creation.
        :param enable_logger_service: ``True`` if ROS2 services are created to allow external nodes
            to get and set logger levels of this node. Otherwise, logger levels are only managed
            locally. That is, logger levels cannot be changed remotely.
        """
        self.__handle = None
        self._context = get_default_context() if context is None else context
        self._parameters: dict = {}
        self._publishers: List[Publisher] = []
        self._subscriptions: List[Subscription] = []
        self._clients: List[Client] = []
        self._services: List[Service] = []
        self._timers: List[Timer] = []
        self._guards: List[GuardCondition] = []
        self.__waitables: List[Waitable] = []
        self._default_callback_group = MutuallyExclusiveCallbackGroup()
        self._pre_set_parameters_callbacks: List[Callable[[List[Parameter]], List[Parameter]]] = []
        self._on_set_parameters_callbacks: \
            List[Callable[[List[Parameter]], SetParametersResult]] = []
        self._post_set_parameters_callbacks: List[Callable[[List[Parameter]], None]] = []
        self._rate_group = ReentrantCallbackGroup()
        self._allow_undeclared_parameters = allow_undeclared_parameters
        self._parameter_overrides = {}
        self._descriptors = {}

        namespace = namespace or ''
        if not self._context.ok():
            raise NotInitializedException('cannot create node')
        with self._context.handle:
            try:
                self.__node = _rclpy.Node(
                    node_name,
                    namespace,
                    self._context.handle,
                    cli_args,
                    use_global_arguments,
                    enable_rosout
                )
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
        with self.handle:
            self._logger = get_logger(self.__node.logger_name())

        self.__executor_weakref = None

        self._parameter_event_publisher = self.create_publisher(
            ParameterEvent, '/parameter_events', qos_profile_parameter_events)

        with self.handle:
            self._parameter_overrides = self.__node.get_parameters(Parameter)
        # Combine parameters from params files with those from the node constructor and
        # use the set_parameters_atomically API so a parameter event is published.
        if parameter_overrides is not None:
            self._parameter_overrides.update({p.name: p for p in parameter_overrides})

        # Clock that has support for ROS time.
        self._clock = ROSClock()

        if automatically_declare_parameters_from_overrides:
            self.declare_parameters(
                '',
                [
                    (name, param.value, ParameterDescriptor())
                    for name, param in self._parameter_overrides.items()],
                ignore_override=True,
            )

        # Init a time source.
        # Note: parameter overrides and parameter event publisher need to be ready at this point
        # to be able to declare 'use_sim_time' if it was not declared yet.
        self._time_source = TimeSource(node=self)
        self._time_source.attach_clock(self._clock)

        if start_parameter_services:
            self._parameter_service = ParameterService(self)

        if enable_logger_service:
            self._logger_service = LoggingService(self)

        self._type_description_service = TypeDescriptionService(self)

    @property
    def publishers(self) -> Iterator[Publisher]:
        """Get publishers that have been created on this node."""
        yield from self._publishers

    @property
    def subscriptions(self) -> Iterator[Subscription]:
        """Get subscriptions that have been created on this node."""
        yield from self._subscriptions

    @property
    def clients(self) -> Iterator[Client]:
        """Get clients that have been created on this node."""
        yield from self._clients

    @property
    def services(self) -> Iterator[Service]:
        """Get services that have been created on this node."""
        yield from self._services

    @property
    def timers(self) -> Iterator[Timer]:
        """Get timers that have been created on this node."""
        yield from self._timers

    @property
    def guards(self) -> Iterator[GuardCondition]:
        """Get guards that have been created on this node."""
        yield from self._guards

    @property
    def waitables(self) -> Iterator[Waitable]:
        """Get waitables that have been created on this node."""
        yield from self.__waitables

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

    def _wake_executor(self):
        executor = self.executor
        if executor:
            executor.wake()

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

        :raises: AttributeError if modified after creation.
        """
        return self.__node

    @handle.setter
    def handle(self, value):
        raise AttributeError('handle cannot be modified after node creation')

    def get_name(self) -> str:
        """Get the name of the node."""
        with self.handle:
            return self.handle.get_node_name()

    def get_namespace(self) -> str:
        """Get the namespace of the node."""
        with self.handle:
            return self.handle.get_namespace()

    def get_clock(self) -> Clock:
        """Get the clock used by the node."""
        return self._clock

    def get_logger(self):
        """Get the nodes logger."""
        return self._logger

    def declare_parameter(
        self,
        name: str,
        value: Any = None,
        descriptor: Optional[ParameterDescriptor] = None,
        ignore_override: bool = False
    ) -> Parameter:
        """
        Declare and initialize a parameter.

        This method, if successful, will result in any callback registered with
        :func:`add_on_set_parameters_callback` and :func:`add_post_set_parameters_callback`
        to be called.

        The name and type in the given descriptor is ignored, and should be specified using
        the name argument to this function and the default value's type instead.

        :param name: Fully-qualified name of the parameter, including its namespace.
        :param value: Value of the parameter to declare.
        :param descriptor: Descriptor for the parameter to declare.
        :param ignore_override: True if overrides shall not be taken into account; False otherwise.
        :return: Parameter with the effectively assigned value.
        :raises: ParameterAlreadyDeclaredException if the parameter had already been declared.
        :raises: InvalidParameterException if the parameter name is invalid.
        :raises: InvalidParameterValueException if the registered callback rejects the parameter.
        """
        if value is None and descriptor is None:
            # Temporal patch so we get deprecation warning if only a name is provided.
            args = (name, )
        else:
            descriptor = ParameterDescriptor() if descriptor is None else descriptor
            args = (name, value, descriptor)
        return self.declare_parameters('', [args], ignore_override)[0]

    def declare_parameters(
        self,
        namespace: str,
        parameters: List[Union[
            Tuple[str],
            Tuple[str, Parameter.Type],
            Tuple[str, Any, ParameterDescriptor],
        ]],
        ignore_override: bool = False
    ) -> List[Parameter]:
        """
        Declare a list of parameters.

        The tuples in the given parameter list shall contain the name for each parameter,
        optionally providing a value and a descriptor.
        The name and type in the given descriptors are ignored, and should be specified using
        the name argument to this function and the default value's type instead.
        For each entry in the list, a parameter with a name of "namespace.name"
        will be declared.
        The resulting value for each declared parameter will be returned, considering
        parameter overrides set upon node creation as the first choice,
        or provided parameter values as the second one.

        The name expansion is naive, so if you set the namespace to be "foo.",
        then the resulting parameter names will be like "foo..name".
        However, if the namespace is an empty string, then no leading '.' will be
        placed before each name, which would have been the case when naively
        expanding "namespace.name".
        This allows you to declare several parameters at once without a namespace.

        This method will result in any callback registered with
        :func:`add_on_set_parameters_callback` and :func:`add_post_set_parameters_callback`
        to be called once for each parameter.

        If a callback was registered previously with :func:`add_on_set_parameters_callback`, it
        will be called prior to setting the parameters for the node, once for each parameter.
        If one of the calls due to :func:`add_on_set_parameters_callback` fail, an exception will
        be raised and the remaining parameters will not be declared. Parameters declared up to that
        point will not be undeclared.

        If a callback was registered previously with :func:`add_post_set_parameters_callback`,
        it will be called after setting the parameters successfully for the node,
        once for each parameter.

        This method will `not` result in any callbacks registered with
        :func:`add_pre_set_parameters_callback` to be called.

        :param namespace: Namespace for parameters.
        :param parameters: List of tuples with parameters to declare.
        :param ignore_override: True if overrides shall not be taken into account; False otherwise.
        :return: Parameter list with the effectively assigned values for each of them.
        :raises: ParameterAlreadyDeclaredException if the parameter had already been declared.
        :raises: InvalidParameterException if the parameter name is invalid.
        :raises: InvalidParameterValueException if the registered callback rejects any parameter.
        :raises: TypeError if any tuple in **parameters** does not match the annotated type.
        """
        parameter_list = []
        descriptors = {}
        for index, parameter_tuple in enumerate(parameters):
            if len(parameter_tuple) < 1 or len(parameter_tuple) > 3:
                raise TypeError(
                    'Invalid parameter tuple length at index {index} in parameters list: '
                    '{parameter_tuple}; expecting length between 1 and 3'.format_map(locals())
                )

            value = None
            param_type = None

            # Get the values from the tuple, checking its types.
            # Use defaults if the tuple doesn't contain value and / or descriptor.
            name = parameter_tuple[0]
            if not isinstance(name, str):
                raise TypeError(
                        f'First element {name} at index {index} in parameters list '
                        'is not a str.')
            if namespace:
                name = f'{namespace}.{name}'

            # Note(jubeira): declare_parameters verifies the name, but set_parameters doesn't.
            validate_parameter_name(name)

            second_arg = parameter_tuple[1] if 1 < len(parameter_tuple) else None
            descriptor = parameter_tuple[2] if 2 < len(parameter_tuple) else ParameterDescriptor()

            if not isinstance(descriptor, ParameterDescriptor):
                raise TypeError(
                    f'Third element {descriptor} at index {index} in parameters list '
                    'is not a ParameterDescriptor.'
                )

            if len(parameter_tuple) == 1:
                warnings.warn(
                    f"when declaring parameter named '{name}', "
                    'declaring a parameter only providing its name is deprecated. '
                    'You have to either:\n'
                    '\t- Pass a name and a default value different to "PARAMETER NOT SET"'
                    ' (and optionally a descriptor).\n'
                    '\t- Pass a name and a parameter type.\n'
                    '\t- Pass a name and a descriptor with `dynamic_typing=True')
                descriptor.dynamic_typing = True

            if isinstance(second_arg, Parameter.Type):
                if second_arg == Parameter.Type.NOT_SET:
                    raise ValueError(
                        f'Cannot declare parameter {{{name}}} as statically typed of type NOT_SET')
                if descriptor.dynamic_typing is True:
                    raise ValueError(
                        f'When declaring parameter {{{name}}} passing a descriptor with'
                        '`dynamic_typing=True` is not allowed when the parameter type is provided')
                descriptor.type = second_arg.value
            else:
                value = second_arg
                if not descriptor.dynamic_typing:
                    # infer type from default value
                    if isinstance(value, ParameterValue):
                        descriptor.type = value.type
                    else:
                        descriptor.type = Parameter.Type.from_parameter_value(value).value
                    if descriptor.type == ParameterType.PARAMETER_NOT_SET:
                        raise ValueError(
                            'Cannot declare a statically typed parameter with default value '
                            'of type PARAMETER_NOT_SET')

            # Get value from parameter overrides, of from tuple if it doesn't exist.
            if not ignore_override and name in self._parameter_overrides:
                value = self._parameter_overrides[name].value

            parameter_list.append(Parameter(name, value=value))
            descriptors.update({name: descriptor})

        parameters_already_declared = [
            parameter.name for parameter in parameter_list if parameter.name in self._parameters
        ]
        if any(parameters_already_declared):
            raise ParameterAlreadyDeclaredException(parameters_already_declared)

        # Call the callback once for each of the parameters, using method that doesn't
        # check whether the parameter was declared beforehand or not.
        self._declare_parameter_common(
            parameter_list,
            descriptors
        )
        # Don't call get_parameters() to bypass check for NOT_SET parameters
        return [self._parameters[parameter.name] for parameter in parameter_list]

    def _declare_parameter_common(
        self,
        parameter_list: List[Parameter],
        descriptors: Optional[Dict[str, ParameterDescriptor]] = None
    ) -> List[SetParametersResult]:
        """
        Declare parameters for the node, and return the result for the declare action.

        Method for internal usage; applies a setter method for each parameters in the list.
        By default, it checks if the parameters were declared, raising an exception if at least
        one of them was not.

        This method will result in any callback registered with
        :func:`add_on_set_parameters_callback` and :func:`add_post_set_parameters_callback`
        to be called once for each parameter.

        If a callback was registered previously with :func:`add_on_set_parameters_callback`, it
        will be called prior to setting the parameters for the node, once for each parameter.
        If the callback doesn't succeed for a given parameter an exception will be raised.

        If a callback was registered previously with :func:`add_post_set_parameters_callback`,
        it will be called after setting the parameters successfully for the node,
        once for each parameter.

        This method will `not` result in any callbacks registered with
        :func:`add_pre_set_parameters_callback` to be called.

        :param parameter_list: List of parameters to set.
        :param descriptors: Descriptors to set to the given parameters.
           If descriptors are given, each parameter in the list must have a corresponding one.
        :return: The result for each set action as a list.
        :raises: InvalidParameterValueException if the user-defined callback rejects the
            parameter value.
        :raises: ParameterNotDeclaredException if undeclared parameters are not allowed in this
            method and at least one parameter in the list hadn't been declared beforehand.`
        """
        if descriptors is not None:
            assert all(parameter.name in descriptors for parameter in parameter_list)

        results = []
        for param in parameter_list:
            # If undeclared parameters are allowed, parameters with type NOT_SET shall be stored.
            result = self._set_parameters_atomically_common(
                [param],
                descriptors,
                allow_not_set_type=True
            )
            if not result.successful:
                if result.reason.startswith('Wrong parameter type'):
                    raise InvalidParameterTypeException(
                        param, Parameter.Type(descriptors[param._name].type).name)
                raise InvalidParameterValueException(param.name, param.value, result.reason)
            results.append(result)
        return results

    def undeclare_parameter(self, name: str):
        """
        Undeclare a previously declared parameter.

        This method will not cause a callback registered with any of the
        :func:`add_pre_set_parameters_callback`,
        and :func:`add_post_set_parameters_callback` to be called.

        :param name: Fully-qualified name of the parameter, including its namespace.
        :raises: ParameterNotDeclaredException if parameter had not been declared before.
        :raises: ParameterImmutableException if the parameter was created as read-only.
        """
        if self.has_parameter(name):
            if self._descriptors[name].read_only:
                raise ParameterImmutableException(name)
            else:
                del self._parameters[name]
                del self._descriptors[name]
        else:
            raise ParameterNotDeclaredException(name)

    def has_parameter(self, name: str) -> bool:
        """Return True if parameter is declared; False otherwise."""
        return name in self._parameters

    def get_parameter_types(self, names: List[str]) -> List[Parameter.Type]:
        """
        Get a list of parameter types.

        :param names: Fully-qualified names of the parameters to get, including their namespaces.
        :return: The values for the given parameter types.
            A default Parameter.Type.NOT_SET will be returned for undeclared parameters
            if undeclared parameters are allowed.
        :raises: ParameterNotDeclaredException if undeclared parameters are not allowed,
            and at least one parameter hadn't been declared beforehand.
        """
        if not all(isinstance(name, str) for name in names):
            raise TypeError('All names must be instances of type str')
        return [self.get_parameter_type(name) for name in names]

    def get_parameter_type(self, name: str) -> Parameter.Type:
        """
        Get a parameter type by name.

        :param name: Fully-qualified name of the parameter, including its namespace.
        :return: The type for the given parameter name.
            A default Parameter.Type.NOT_SET will be returned for an undeclared parameter
            if undeclared parameters are allowed.
        :raises: ParameterNotDeclaredException if undeclared parameters are not allowed,
            and the parameter hadn't been declared beforehand.
        """
        if self.has_parameter(name):
            return self._parameters[name].type_.value
        elif self._allow_undeclared_parameters:
            return Parameter.Type.NOT_SET
        else:
            raise ParameterNotDeclaredException(name)

    def get_parameters(self, names: List[str]) -> List[Parameter]:
        """
        Get a list of parameters.

        :param names: Fully-qualified names of the parameters to get, including their namespaces.
        :return: The values for the given parameter names.
            A default Parameter will be returned for undeclared parameters if
            undeclared parameters are allowed.
        :raises: ParameterNotDeclaredException if undeclared parameters are not allowed,
            and at least one parameter hadn't been declared beforehand.
        :raises: ParameterUninitializedException if at least one parameter is statically typed and
            uninitialized.
        """
        if not all(isinstance(name, str) for name in names):
            raise TypeError('All names must be instances of type str')
        return [self.get_parameter(name) for name in names]

    def get_parameter(self, name: str) -> Parameter:
        """
        Get a parameter by name.

        :param name: Fully-qualified name of the parameter, including its namespace.
        :return: The value for the given parameter name.
            A default Parameter will be returned for an undeclared parameter if
            undeclared parameters are allowed.
        :raises: ParameterNotDeclaredException if undeclared parameters are not allowed,
            and the parameter hadn't been declared beforehand.
        :raises: ParameterUninitializedException if the parameter is statically typed and
            uninitialized.
        """
        if self.has_parameter(name):
            parameter = self._parameters[name]
            if (
                parameter.type_ != Parameter.Type.NOT_SET or
                self._descriptors[name].dynamic_typing
            ):
                return self._parameters[name]
            # Statically typed, uninitialized parameter
            raise ParameterUninitializedException(name)
        elif self._allow_undeclared_parameters:
            return Parameter(name, Parameter.Type.NOT_SET, None)
        else:
            raise ParameterNotDeclaredException(name)

    def get_parameter_or(
            self, name: str, alternative_value: Optional[Parameter] = None) -> Parameter:
        """
        Get a parameter or the alternative value.

        If the alternative value is None, a default Parameter with the given name and NOT_SET
        type will be returned if the parameter was not declared.

        :param name: Fully-qualified name of the parameter, including its namespace.
        :param alternative_value: Alternative parameter to get if it had not been declared before.
        :return: Requested parameter, or alternative value if it hadn't been declared before or is
          an uninitialized statically typed parameter.
        """
        if alternative_value is None:
            alternative_value = Parameter(name, Parameter.Type.NOT_SET)

        if not self.has_parameter(name):
            return alternative_value

        # Return alternative for uninitialized parameters
        if (self._parameters[name].type_ == Parameter.Type.NOT_SET):
            return alternative_value

        return self._parameters[name]

    def get_parameters_by_prefix(self, prefix: str) -> Dict[str, Optional[Union[
        bool, int, float, str, bytes,
        Sequence[bool], Sequence[int], Sequence[float], Sequence[str]
    ]]]:
        """
        Get parameters that have a given prefix in their names as a dictionary.

        The names which are used as keys in the returned dictionary have the prefix removed.
        For example, if you use the prefix "foo" and the parameters "foo.ping", "foo.pong"
        and "bar.baz" exist, then the returned dictionary will have the keys "ping" and "pong".
        Note that the parameter separator is also removed from the parameter name to create the
        keys.

        An empty string for the prefix will match all parameters.

        If no parameters with the prefix are found, an empty dictionary will be returned.

        :param prefix: The prefix of the parameters to get.
        :return: Dict of parameters with the given prefix.
        """
        if prefix:
            prefix = prefix + PARAMETER_SEPARATOR_STRING
        prefix_len = len(prefix)
        return {
            param_name[prefix_len:]: param_value
            for param_name, param_value in self._parameters.items()
            if param_name.startswith(prefix)
        }

    def set_parameters(self, parameter_list: List[Parameter]) -> List[SetParametersResult]:
        """
        Set parameters for the node, and return the result for the set action.

        If any parameter in the list was not declared beforehand and undeclared parameters are not
        allowed for the node, this method will raise a ParameterNotDeclaredException exception.

        Parameters are set in the order they are declared in the list.
        If setting a parameter fails due to not being declared, then the
        parameters which have already been set will stay set, and no attempt will
        be made to set the parameters which come after.

        If undeclared parameters are allowed, then all the parameters will be implicitly
        declared before being set even if they were not declared beforehand.
        Parameter overrides are ignored by this method.

        This method will result in any callback registered with
        :func:`add_pre_set_parameters_callback`, :func:`add_on_set_parameters_callback` and
        :func:`add_post_set_parameters_callback` to be called once for each parameter.

        If a callback was registered previously with :func:`add_pre_set_parameters_callback`, it
        will be called prior to the validation of parameters for the node, once for each parameter.
        If this callback makes modified parameter list empty, then it will be reflected in the
        returned result; no exceptions will be raised in this case.

        If a callback was registered previously with :func:`add_on_set_parameters_callback`, it
        will be called prior to setting the parameters for the node, once for each parameter.
        If this callback prevents a parameter from being set, then it will be reflected in the
        returned result; no exceptions will be raised in this case.

        If a callback was registered previously with :func:`add_post_set_parameters_callback`,
        it will be called after setting the parameters successfully for the node,
        once for each parameter.

        For each successfully set parameter, a :class:`ParameterEvent` message is
        published.

        If the value type of the parameter is NOT_SET, and the existing parameter type is
        something else, then the parameter will be implicitly undeclared.

        :param parameter_list: The list of parameters to set.
        :return: The result for each set action as a list.
        :raises: ParameterNotDeclaredException if undeclared parameters are not allowed,
            and at least one parameter in the list hadn't been declared beforehand.
        """
        results = []
        for param in parameter_list:
            result = self._set_parameters_atomically([param])
            results.append(result)

        return results

    def set_parameters_atomically(self, parameter_list: List[Parameter]) -> SetParametersResult:
        """
        Set the given parameters, all at one time, and then aggregate result.

        If any parameter in the list was not declared beforehand and undeclared parameters are not
        allowed for the node, this method will raise a ParameterNotDeclaredException exception.

        Parameters are set all at once.
        If setting a parameter fails due to not being declared, then no parameter will be set.
        Either all of the parameters are set or none of them are set.

        If undeclared parameters are allowed for the node, then all the parameters will be
        implicitly declared before being set even if they were not declared beforehand.

        This method will result in any callback registered with
        :func:`add_pre_set_parameters_callback` :func:`add_on_set_parameters_callback` and
        :func:`add_post_set_parameters_callback` to be called only once for all parameters.

        If a callback was registered previously with :func:`add_pre_set_parameters_callback`, it
        will be called prior to the validation of node parameters only once for all parameters.
        If this callback makes modified parameter list empty, then it will be reflected in the
        returned result; no exceptions will be raised in this case.

        If a callback was registered previously with :func:`add_on_set_parameters_callback`, it
        will be called prior to setting the parameters for the node only once for all parameters.
        If the callback prevents the parameters from being set, then it will be reflected in the
        returned result; no exceptions will be raised in this case.

        If a callback was registered previously with :func:`add_post_set_parameters_callback`, it
        will be called after setting the node parameters successfully only once for all parameters.

        For each successfully set parameter, a :class:`ParameterEvent` message is published.

        If the value type of the parameter is NOT_SET, and the existing parameter type is
        something else, then the parameter will be implicitly undeclared.

        :param parameter_list: The list of parameters to set.
        :return: Aggregate result of setting all the parameters atomically.
        :raises: ParameterNotDeclaredException if undeclared parameters are not allowed,
            and at least one parameter in the list hadn't been declared beforehand.
        """
        return self._set_parameters_atomically(parameter_list)

    def _set_parameters_atomically(
        self,
        parameter_list: List[Parameter],
    ) -> SetParametersResult:

        modified_parameter_list = self._call_pre_set_parameters_callback(parameter_list)
        if modified_parameter_list is not None:
            parameter_list = modified_parameter_list

            if len(parameter_list) == 0:
                result = SetParametersResult()
                result.successful = False
                result.reason = 'parameter list cannot be empty, this might be due to ' \
                                'pre_set_parameters_callback modifying the original parameters ' \
                                'list.'
                return result

        self._check_undeclared_parameters(parameter_list)
        return self._set_parameters_atomically_common(parameter_list)

    def _set_parameters_atomically_common(
        self,
        parameter_list: List[Parameter],
        descriptors: Optional[Dict[str, ParameterDescriptor]] = None,
        allow_not_set_type: bool = False
    ) -> SetParametersResult:
        """
        Set the given parameters, all at one time, and then aggregate result.

        This internal method does not reject undeclared parameters.
        If :param:`allow_not_set_type` is False, a parameter with type NOT_SET will be undeclared.

        This method will result in any callback registered with
        :func:`add_on_set_parameters_callback` and :func:`add_post_set_parameters_callback`
        to be called only once for all parameters.

        If a callback was registered previously with :func:`add_on_set_parameters_callback`, it
        will be called prior to setting the parameters for the node only once for all parameters.
        If the callback prevents the parameters from being set, then it will be reflected in the
        returned result; no exceptions will be raised in this case.

        If a callback was registered previously with :func:`add_post_set_parameters_callback`,
        it will be called after setting the node parameters successfully only once for all
        parameters.

        For each successfully set parameter, a :class:`ParameterEvent` message is
        published.

        :param parameter_list: The list of parameters to set.
        :param descriptors: New descriptors to apply to the parameters before setting them.
            If descriptors are given, each parameter in the list must have a corresponding one.
        :param allow_not_set_type: False if parameters with NOT_SET type shall be undeclared,
            True if they should be stored despite not having an actual value.
        :return: Aggregate result of setting all the parameters atomically.
        """
        if descriptors is not None:
            # If new descriptors are provided, ensure every parameter has an assigned descriptor
            # and do not check for read-only.
            assert all(parameter.name in descriptors for parameter in parameter_list)
            result = self._apply_descriptors(parameter_list, descriptors, False)
        else:
            # If new descriptors are not provided, use existing ones and check for read-only.
            result = self._apply_descriptors(parameter_list, self._descriptors, True)

        if not result.successful:
            return result
        elif self._on_set_parameters_callbacks:
            for callback in self._on_set_parameters_callbacks:
                result = callback(parameter_list)
                if not result.successful:
                    return result
        result = SetParametersResult(successful=True)

        if result.successful:
            parameter_event = ParameterEvent()
            # Add fully qualified path of node to parameter event
            if self.get_namespace() == '/':
                parameter_event.node = self.get_namespace() + self.get_name()
            else:
                parameter_event.node = self.get_namespace() + '/' + self.get_name()

            for param in parameter_list:
                # If parameters without type and value are not allowed, they shall be undeclared.
                if not allow_not_set_type and Parameter.Type.NOT_SET == param.type_:
                    # Parameter deleted. (Parameter had value and new value is not set).
                    parameter_event.deleted_parameters.append(param.to_parameter_msg())
                    # Delete any unset parameters regardless of their previous value.
                    if param.name in self._parameters:
                        del self._parameters[param.name]
                    if param.name in self._descriptors:
                        del self._descriptors[param.name]
                else:
                    # Update descriptors; set a default if it doesn't exist.
                    # Don't update if it already exists for the current parameter and a new one
                    # was not specified in this method call.
                    if descriptors is not None:
                        self._descriptors[param.name] = descriptors[param.name]
                    elif param.name not in self._descriptors:
                        descriptor = ParameterDescriptor()
                        descriptor.dynamic_typing = True
                        self._descriptors[param.name] = descriptor

                    if Parameter.Type.NOT_SET == self.get_parameter_or(param.name).type_:
                        #  Parameter is new. (Parameter had no value and new value is set)
                        parameter_event.new_parameters.append(param.to_parameter_msg())
                    else:
                        parameter_event.changed_parameters.append(
                            param.to_parameter_msg())

                    # Descriptors have already been applied by this point.
                    self._parameters[param.name] = param

            parameter_event.stamp = self._clock.now().to_msg()
            self._parameter_event_publisher.publish(parameter_event)

            # call post set parameter registered callbacks
            self._call_post_set_parameters_callback(parameter_list)

        return result

    def list_parameters(
        self,
        prefixes: List[str],
        depth: int
    ) -> ListParametersResult:
        """
        Get a list of parameter names and their prefixes.

        :param prefixes: A list of prefixes to filter the parameter names.
            Only the parameter names that start with any of the prefixes are returned.
            If empty, all parameter names are returned.
        :param depth: The depth of nested parameter names to include.
        :return: The list of parameter names and their prefixes.
        :raises: TypeError if the type of any of the passed arguments is not an expected type.
        :raises: ValueError if depth is a negative integer.
        """
        if not isinstance(prefixes, list):
            raise TypeError('The prefixes argument must be a list')
        if not all(isinstance(prefix, str) for prefix in prefixes):
            raise TypeError('All prefixes must be instances of type str')
        if not isinstance(depth, int):
            raise TypeError('The depth must be instance of type int')
        if depth < 0:
            raise ValueError('The depth must be greater than or equal to zero')

        result = ListParametersResult()

        separator_less_than_depth: Callable[[str], bool] = \
            lambda s: s.count(PARAMETER_SEPARATOR_STRING) < depth

        recursive: bool = \
            (len(prefixes) == 0) and (depth == ListParameters.Request.DEPTH_RECURSIVE)

        for param_name in self._parameters.keys():
            if not recursive:
                get_all: bool = (len(prefixes) == 0) and (separator_less_than_depth(param_name))
                if not get_all:
                    prefix_matches = any(
                        param_name == prefix or
                        (
                            param_name.startswith(prefix+PARAMETER_SEPARATOR_STRING) and
                            (
                                depth == ListParameters.Request.DEPTH_RECURSIVE or
                                separator_less_than_depth(param_name[len(prefix)+1:])
                            )
                        )
                        for prefix in prefixes
                    )
                    if not prefix_matches:
                        continue

            result.names.append(param_name)
            last_separator = param_name.rfind(PARAMETER_SEPARATOR_STRING)
            if last_separator != -1:
                prefix = param_name[:last_separator]
                if prefix not in result.prefixes:
                    result.prefixes.append(prefix)

        return result

    def _check_undeclared_parameters(self, parameter_list: List[Parameter]):
        """
        Check if parameter list has correct types and was declared beforehand.

        :raises: ParameterNotDeclaredException if at least one parameter in the list was not
            declared beforehand.
        """
        if not all(isinstance(parameter, Parameter) for parameter in parameter_list):
            raise TypeError("parameter must be instance of type '{}'".format(repr(Parameter)))

        undeclared_parameters = (
            param.name for param in parameter_list if param.name not in self._parameters
        )
        if not self._allow_undeclared_parameters and any(undeclared_parameters):
            raise ParameterNotDeclaredException(list(undeclared_parameters))

    def _call_pre_set_parameters_callback(self, parameter_list: [List[Parameter]]):
        if self._pre_set_parameters_callbacks:
            modified_parameter_list = []
            for callback in self._pre_set_parameters_callbacks:
                modified_parameter_list.extend(callback(parameter_list))

            return modified_parameter_list
        else:
            return None

    def _call_post_set_parameters_callback(self, parameter_list: [List[Parameter]]):
        if self._post_set_parameters_callbacks:
            for callback in self._post_set_parameters_callbacks:
                callback(parameter_list)

    def add_pre_set_parameters_callback(
            self,
            callback: Callable[[List[Parameter]], List[Parameter]]
    ) -> None:
        """
        Add a callback gets triggered before parameters are validated.

        Calling this function will add a callback in self._pre_set_parameter_callbacks list.

        This callback can be used to modify the original list of parameters being
        set by the user. The modified list of parameters is then forwarded to the
        "on set parameter" callback for validation.

        The callback takes a list of parameters to be set and returns a list of
        modified parameters.

        One of the use case of "pre set callback" can be updating additional parameters
        conditioned on changes to a parameter.

        All parameters in the modified list will be set atomically.

        Note that the callback is only called while setting parameters with ``set_parameters``,
        ``set_parameters_atomically``, or externally with a parameters service.

        The callback is not called when parameters are declared with ``declare_parameter``
        or ``declare_parameters``.

        The callback is not called when parameters are undeclared with ``undeclare_parameter``.

        An empty modified parameter list from the callback will result in ``set_parameter*``
        returning an unsuccessful result.

        :param callback: The function that is called before parameters are validated.
        """
        self._pre_set_parameters_callbacks.insert(0, callback)

    def add_on_set_parameters_callback(
            self,
            callback: Callable[[List[Parameter]], SetParametersResult]
    ) -> None:
        """
        Add a callback in front to the list of callbacks.

        Calling this function will add a callback in self._on_set_parameter_callbacks list.

        It is considered bad practice to reject changes for "unknown" parameters as this prevents
        other parts of the node (that may be aware of these parameters) from handling them.

        :param callback: The function that is called whenever parameters are being validated
                         for the node.
        """
        self._on_set_parameters_callbacks.insert(0, callback)

    def add_post_set_parameters_callback(
            self,
            callback: Callable[[List[Parameter]], None]
    ) -> None:
        """
        Add a callback gets triggered after parameters are set successfully.

        Calling this function will add a callback in self._post_set_parameter_callbacks list.

        The callback signature is designed to allow handling of the ``set_parameter*``
        or ``declare_parameter*`` methods. The callback takes a list of parameters that
        have been set successfully.

        The callback can be valuable as a place to cause side effects based on parameter
        changes. For instance updating the internally tracked class attributes once the params
        have been changed successfully.

        :param callback: The function that is called after parameters are set for the node.
        """
        self._post_set_parameters_callbacks.insert(0, callback)

    def remove_pre_set_parameters_callback(
            self,
            callback: Callable[[List[Parameter]], List[Parameter]]
    ) -> None:
        """
        Remove a callback from list of callbacks.

        Calling this function will remove the callback from self._pre_set_parameter_callbacks list.

        :param callback: The function that is called whenever parameters are set for the node.
        :raises: ValueError if a callback is not present in the list of callbacks.
        """
        self._pre_set_parameters_callbacks.remove(callback)

    def remove_on_set_parameters_callback(
            self,
            callback: Callable[[List[Parameter]], SetParametersResult]
    ) -> None:
        """
        Remove a callback from list of callbacks.

        Calling this function will remove the callback from self._parameter_callbacks list.

        :param callback: The function that is called whenever parameters are set for the node.
        :raises: ValueError if a callback is not present in the list of callbacks.
        """
        self._on_set_parameters_callbacks.remove(callback)

    def remove_post_set_parameters_callback(
            self,
            callback: Callable[[List[Parameter]], None]
    ) -> None:
        """
        Remove a callback from list of callbacks.

        Calling this function will remove the callback from self._parameter_callbacks list.

        :param callback: The function that is called whenever parameters are set for the node.
        :raises: ValueError if a callback is not present in the list of callbacks.
        """
        self._post_set_parameters_callbacks.remove(callback)

    def _apply_descriptors(
        self,
        parameter_list: List[Parameter],
        descriptors: Dict[str, ParameterDescriptor],
        check_read_only: bool = True
    ) -> SetParametersResult:
        """
        Apply descriptors to parameters and return an aggregated result without saving parameters.

        In case no descriptors are provided to the method, existing descriptors shall be used.
        In any case, if a given parameter doesn't have a descriptor it shall be skipped.

        :param parameter_list: Parameters to be checked.
        :param descriptors: Descriptors to apply.
        :param check_read_only: ``True`` if read-only check has to be applied.
        :return: SetParametersResult; successful if checks passed, unsuccessful otherwise.
        :raises: ParameterNotDeclaredException if a descriptor is not provided, the given parameter
            name had not been declared and undeclared parameters are not allowed.
        """
        for param in parameter_list:
            if param.name in descriptors:
                result = self._apply_descriptor(param, descriptors[param.name], check_read_only)
                if not result.successful:
                    return result
        return SetParametersResult(successful=True)

    def _apply_descriptor(
        self,
        parameter: Parameter,
        descriptor: Optional[ParameterDescriptor] = None,
        check_read_only: bool = True
    ) -> SetParametersResult:
        """
        Apply a descriptor to a parameter and return a result without saving the parameter.

        This method sets the type in the descriptor to match the parameter type.
        If a descriptor is provided, its name will be set to the name of the parameter.

        :param parameter: Parameter to be checked.
        :param descriptor: Descriptor to apply. If ``None``, the stored descriptor for the given
            parameter's name is used instead.
        :param check_read_only: ``True`` if read-only check has to be applied.
        :return: SetParametersResult; successful if checks passed, unsuccessful otherwise.
        :raises: ParameterNotDeclaredException if a descriptor is not provided, the given parameter
            name had not been declared and undeclared parameters are not allowed.
        """
        if descriptor is None:
            descriptor = self.describe_parameter(parameter.name)
        else:
            descriptor.name = parameter.name

        if check_read_only and descriptor.read_only:
            return SetParametersResult(
                successful=False,
                reason='Trying to set a read-only parameter: {}.'.format(parameter.name))

        if descriptor.dynamic_typing:
            descriptor.type = parameter.type_.value
        # If this parameter has already been declared, do not allow undeclaring it
        elif self.has_parameter(parameter.name) and parameter.type_ == Parameter.Type.NOT_SET:
            return SetParametersResult(
                successful=False,
                reason='Static parameter cannot be undeclared'
            )
        elif (
            parameter.type_ != Parameter.Type.NOT_SET and
            parameter.type_.value != descriptor.type
        ):
            return SetParametersResult(
                successful=False,
                reason=(
                    'Wrong parameter type, expected '
                    f"'{Parameter.Type(descriptor.type)}'"
                    f" got '{parameter.type_}'")
            )

        if parameter.type_ == Parameter.Type.INTEGER and descriptor.integer_range:
            return self._apply_integer_range(parameter, descriptor.integer_range[0])

        if parameter.type_ == Parameter.Type.DOUBLE and descriptor.floating_point_range:
            return self._apply_floating_point_range(parameter, descriptor.floating_point_range[0])

        return SetParametersResult(successful=True)

    def _apply_integer_range(
        self,
        parameter: Parameter,
        integer_range: IntegerRange
    ) -> SetParametersResult:
        min_value = min(integer_range.from_value, integer_range.to_value)
        max_value = max(integer_range.from_value, integer_range.to_value)

        # Values in the edge are always OK.
        if parameter.value == min_value or parameter.value == max_value:
            return SetParametersResult(successful=True)

        if not min_value < parameter.value < max_value:
            return SetParametersResult(
                successful=False,
                reason='Parameter {} out of range. '
                       'Min: {}, Max: {}, value: {}'.format(
                            parameter.name, min_value, max_value, parameter.value
                        )
            )

        if integer_range.step != 0 and (parameter.value - min_value) % integer_range.step != 0:
            return SetParametersResult(
                successful=False,
                reason='The parameter value for {} is not a valid step. '
                       'Min: {}, max: {}, value: {}, step: {}'.format(
                            parameter.name,
                            min_value,
                            max_value,
                            parameter.value,
                            integer_range.step
                        )
            )

        return SetParametersResult(successful=True)

    def _apply_floating_point_range(
        self,
        parameter: Parameter,
        floating_point_range: FloatingPointRange
    ) -> SetParametersResult:
        min_value = min(floating_point_range.from_value, floating_point_range.to_value)
        max_value = max(floating_point_range.from_value, floating_point_range.to_value)

        # Values in the edge are always OK.
        if (
            math.isclose(parameter.value, min_value, rel_tol=self.PARAM_REL_TOL) or
            math.isclose(parameter.value, max_value, rel_tol=self.PARAM_REL_TOL)
        ):
            return SetParametersResult(successful=True)

        if not min_value < parameter.value < max_value:
            return SetParametersResult(
                successful=False,
                reason='Parameter {} out of range '
                       'Min: {}, Max: {}, value: {}'.format(
                            parameter.name, min_value, max_value, parameter.value
                        )
            )

        if floating_point_range.step != 0.0:
            distance_int_steps = round((parameter.value - min_value) / floating_point_range.step)
            if not math.isclose(
                min_value + distance_int_steps * floating_point_range.step,
                parameter.value,
                rel_tol=self.PARAM_REL_TOL
            ):
                return SetParametersResult(
                    successful=False,
                    reason='The parameter value for {} is not close enough to a valid step. '
                           'Min: {}, max: {}, value: {}, step: {}'.format(
                                parameter.name,
                                min_value,
                                max_value,
                                parameter.value,
                                floating_point_range.step
                            )
                )

        return SetParametersResult(successful=True)

    def _apply_descriptor_and_set(
        self,
        parameter: Parameter,
        descriptor: Optional[ParameterDescriptor] = None,
        check_read_only: bool = True
    ) -> SetParametersResult:
        """Apply parameter descriptor and set parameter if successful."""
        result = self._apply_descriptor(parameter, descriptor, check_read_only)
        if result.successful:
            self._parameters[parameter.name] = parameter

        return result

    def describe_parameter(self, name: str) -> ParameterDescriptor:
        """
        Get the parameter descriptor of a given parameter.

        :param name: Fully-qualified name of the parameter, including its namespace.
        :return: ParameterDescriptor corresponding to the parameter,
            or default ParameterDescriptor if parameter had not been declared before
            and undeclared parameters are allowed.
        :raises: ParameterNotDeclaredException if parameter had not been declared before
            and undeclared parameters are not allowed.
        """
        try:
            return self._descriptors[name]
        except KeyError:
            if self._allow_undeclared_parameters:
                return ParameterDescriptor()
            else:
                raise ParameterNotDeclaredException(name)

    def describe_parameters(self, names: List[str]) -> List[ParameterDescriptor]:
        """
        Get the parameter descriptors of a given list of parameters.

        :param name: List of fully-qualified names of the parameters to describe.
        :return: List of ParameterDescriptors corresponding to the given parameters.
            Default ParameterDescriptors shall be returned for parameters that
            had not been declared before if undeclared parameters are allowed.
        :raises: ParameterNotDeclaredException if at least one parameter
            had not been declared before and undeclared parameters are not allowed.
        """
        parameter_descriptors = []
        for name in names:
            parameter_descriptors.append(self.describe_parameter(name))

        return parameter_descriptors

    def set_descriptor(
        self,
        name: str,
        descriptor: ParameterDescriptor,
        alternative_value: Optional[ParameterValue] = None
    ) -> ParameterValue:
        """
        Set a new descriptor for a given parameter.

        The name in the descriptor is ignored and set to ``name``.

        :param name: Fully-qualified name of the parameter to set the descriptor to.
        :param descriptor: New descriptor to apply to the parameter.
        :param alternative_value: Value to set to the parameter if the existing value does not
            comply with the new descriptor.
        :return: ParameterValue for the given parameter name after applying the new descriptor.
        :raises: ParameterNotDeclaredException if parameter had not been declared before
            and undeclared parameters are not allowed.
        :raises: ParameterImmutableException if the parameter exists and is read-only.
        :raises: ParameterValueException if neither the existing value nor the alternative value
            complies with the provided descriptor.
        """
        if not self.has_parameter(name):
            if not self._allow_undeclared_parameters:
                raise ParameterNotDeclaredException(name)
            else:
                return self.get_parameter(name).get_parameter_value()

        if self.describe_parameter(name).read_only:
            raise ParameterImmutableException(name)

        current_parameter = self.get_parameter(name)
        if alternative_value is None:
            alternative_parameter = current_parameter
        else:
            alternative_parameter = Parameter.from_parameter_msg(
                ParameterMsg(name=name, value=alternative_value))

        # First try keeping the parameter, then try the alternative one.
        # Don't check for read-only since we are applying a new descriptor now.
        if not self._apply_descriptor_and_set(current_parameter, descriptor, False).successful:
            alternative_set_result = (
                self._apply_descriptor_and_set(alternative_parameter, descriptor, False)
            )
            if not alternative_set_result.successful:
                raise InvalidParameterValueException(
                    name,
                    alternative_parameter.value,
                    alternative_set_result.reason
                )

        self._descriptors[name] = descriptor
        return self.get_parameter(name).get_parameter_value()

    def _validate_topic_or_service_name(self, topic_or_service_name, *, is_service=False):
        name = self.get_name()
        namespace = self.get_namespace()
        validate_node_name(name)
        validate_namespace(namespace)
        validate_topic_name(topic_or_service_name, is_service=is_service)
        expanded_topic_or_service_name = expand_topic_name(topic_or_service_name, name, namespace)
        validate_full_topic_name(expanded_topic_or_service_name, is_service=is_service)

    def _validate_qos_or_depth_parameter(self, qos_or_depth) -> QoSProfile:
        if isinstance(qos_or_depth, QoSProfile):
            return qos_or_depth
        elif isinstance(qos_or_depth, int):
            if qos_or_depth < 0:
                raise ValueError('history depth must be greater than or equal to zero')
            return QoSProfile(depth=qos_or_depth)
        else:
            raise TypeError(
                'Expected QoSProfile or int, but received {!r}'.format(type(qos_or_depth)))

    def add_waitable(self, waitable: Waitable) -> None:
        """
        Add a class that is capable of adding things to the wait set.

        :param waitable: An instance of a waitable that the node will add to the waitset.
        """
        self.__waitables.append(waitable)
        self._wake_executor()

    def remove_waitable(self, waitable: Waitable) -> None:
        """
        Remove a Waitable that was previously added to the node.

        :param waitable: The Waitable to remove.
        """
        self.__waitables.remove(waitable)
        self._wake_executor()

    def resolve_topic_name(self, topic: str, *, only_expand: bool = False) -> str:
        """
        Return a topic name expanded and remapped.

        :param topic: Topic name to be expanded and remapped.
        :param only_expand: If ``True``, remapping rules won't be applied.
        :return: A fully qualified topic name,
            the result of applying expansion and remapping to the given ``topic``.
        """
        with self.handle:
            return _rclpy.rclpy_resolve_name(self.handle, topic, only_expand, False)

    def resolve_service_name(
        self, service: str, *, only_expand: bool = False
    ) -> str:
        """
        Return a service name expanded and remapped.

        :param service: Service name to be expanded and remapped.
        :param only_expand: If ``True``, remapping rules won't be applied.
        :return: A fully qualified service name,
            the result of applying expansion and remapping to the given ``service``.
        """
        with self.handle:
            return _rclpy.rclpy_resolve_name(self.handle, service, only_expand, True)

    def create_publisher(
        self,
        msg_type,
        topic: str,
        qos_profile: Union[QoSProfile, int],
        *,
        callback_group: Optional[CallbackGroup] = None,
        event_callbacks: Optional[PublisherEventCallbacks] = None,
        qos_overriding_options: Optional[QoSOverridingOptions] = None,
        publisher_class: Type[Publisher] = Publisher,
    ) -> Publisher:
        """
        Create a new publisher.

        :param msg_type: The type of ROS messages the publisher will publish.
        :param topic: The name of the topic the publisher will publish to.
        :param qos_profile: A QoSProfile or a history depth to apply to the publisher.
            In the case that a history depth is provided, the QoS history is set to
            KEEP_LAST, the QoS history depth is set to the value
            of the parameter, and all other QoS settings are set to their default values.
        :param callback_group: The callback group for the publisher's event handlers.
            If ``None``, then the default callback group for the node is used.
        :param event_callbacks: User-defined callbacks for middleware events.
        :return: The new publisher.
        """
        qos_profile = self._validate_qos_or_depth_parameter(qos_profile)

        callback_group = callback_group or self.default_callback_group

        failed = False
        try:
            final_topic = self.resolve_topic_name(topic)
        except RuntimeError:
            # if it's name validation error, raise a more appropriate exception.
            try:
                self._validate_topic_or_service_name(topic)
            except InvalidTopicNameException as ex:
                raise ex from None
            # else reraise the previous exception
            raise

        if qos_overriding_options is None:
            qos_overriding_options = QoSOverridingOptions([])
        _declare_qos_parameters(
            Publisher, self, final_topic, qos_profile, qos_overriding_options)

        # this line imports the typesupport for the message module if not already done
        failed = False
        check_is_valid_msg_type(msg_type)
        try:
            with self.handle:
                publisher_object = _rclpy.Publisher(
                    self.handle, msg_type, topic, qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(topic)

        try:
            publisher = publisher_class(
                publisher_object, msg_type, topic, qos_profile,
                event_callbacks=event_callbacks or PublisherEventCallbacks(),
                callback_group=callback_group)
        except Exception:
            publisher_object.destroy_when_not_in_use()
            raise
        self._publishers.append(publisher)
        self._wake_executor()

        for event_callback in publisher.event_handlers:
            self.add_waitable(event_callback)

        return publisher

    def create_subscription(
        self,
        msg_type,
        topic: str,
        callback: Callable[[MsgType], None],
        qos_profile: Union[QoSProfile, int],
        *,
        callback_group: Optional[CallbackGroup] = None,
        event_callbacks: Optional[SubscriptionEventCallbacks] = None,
        qos_overriding_options: Optional[QoSOverridingOptions] = None,
        raw: bool = False
    ) -> Subscription:
        """
        Create a new subscription.

        :param msg_type: The type of ROS messages the subscription will subscribe to.
        :param topic: The name of the topic the subscription will subscribe to.
        :param callback: A user-defined callback function that is called when a message is
            received by the subscription.
        :param qos_profile: A QoSProfile or a history depth to apply to the subscription.
            In the case that a history depth is provided, the QoS history is set to
            KEEP_LAST, the QoS history depth is set to the value
            of the parameter, and all other QoS settings are set to their default values.
        :param callback_group: The callback group for the subscription. If ``None``, then the
            default callback group for the node is used.
        :param event_callbacks: User-defined callbacks for middleware events.
        :param raw: If ``True``, then received messages will be stored in raw binary
            representation.
        """
        qos_profile = self._validate_qos_or_depth_parameter(qos_profile)

        callback_group = callback_group or self.default_callback_group

        try:
            final_topic = self.resolve_topic_name(topic)
        except RuntimeError:
            # if it's name validation error, raise a more appropriate exception.
            try:
                self._validate_topic_or_service_name(topic)
            except InvalidTopicNameException as ex:
                raise ex from None
            # else reraise the previous exception
            raise

        if qos_overriding_options is None:
            qos_overriding_options = QoSOverridingOptions([])
        _declare_qos_parameters(
            Subscription, self, final_topic, qos_profile, qos_overriding_options)

        # this line imports the typesupport for the message module if not already done
        failed = False
        check_is_valid_msg_type(msg_type)
        try:
            with self.handle:
                subscription_object = _rclpy.Subscription(
                    self.handle, msg_type, topic, qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(topic)

        try:
            subscription = Subscription(
                subscription_object, msg_type,
                topic, callback, callback_group, qos_profile, raw,
                event_callbacks=event_callbacks or SubscriptionEventCallbacks())
        except Exception:
            subscription_object.destroy_when_not_in_use()
            raise
        callback_group.add_entity(subscription)
        self._subscriptions.append(subscription)
        self._wake_executor()

        for event_handler in subscription.event_handlers:
            self.add_waitable(event_handler)

        return subscription

    def create_client(
        self,
        srv_type,
        srv_name: str,
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
        callback_group: Optional[CallbackGroup] = None
    ) -> Client:
        """
        Create a new service client.

        :param srv_type: The service type.
        :param srv_name: The name of the service.
        :param qos_profile: The quality of service profile to apply the service client.
        :param callback_group: The callback group for the service client. If ``None``, then the
            default callback group for the node is used.
        """
        if callback_group is None:
            callback_group = self.default_callback_group
        check_is_valid_srv_type(srv_type)
        failed = False
        try:
            with self.handle:
                client_impl = _rclpy.Client(
                    self.handle,
                    srv_type,
                    srv_name,
                    qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(srv_name, is_service=True)

        client = Client(
            self.context,
            client_impl, srv_type, srv_name, qos_profile,
            callback_group)
        callback_group.add_entity(client)
        self._clients.append(client)
        self._wake_executor()
        return client

    def create_service(
        self,
        srv_type,
        srv_name: str,
        callback: Callable[[SrvTypeRequest, SrvTypeResponse], SrvTypeResponse],
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
        callback_group: Optional[CallbackGroup] = None
    ) -> Service:
        """
        Create a new service server.

        :param srv_type: The service type.
        :param srv_name: The name of the service.
        :param callback: A user-defined callback function that is called when a service request
            received by the server.
        :param qos_profile: The quality of service profile to apply the service server.
        :param callback_group: The callback group for the service server. If ``None``, then the
            default callback group for the node is used.
        """
        if callback_group is None:
            callback_group = self.default_callback_group
        check_is_valid_srv_type(srv_type)
        failed = False
        try:
            with self.handle:
                service_impl = _rclpy.Service(
                    self.handle,
                    srv_type,
                    srv_name,
                    qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(srv_name, is_service=True)

        service = Service(
            service_impl,
            srv_type, srv_name, callback, callback_group, qos_profile)
        callback_group.add_entity(service)
        self._services.append(service)
        self._wake_executor()
        return service

    def create_timer(
        self,
        timer_period_sec: float,
        callback: Callable,
        callback_group: Optional[CallbackGroup] = None,
        clock: Optional[Clock] = None,
        autostart: bool = True,
    ) -> Timer:
        """
        Create a new timer.

        If autostart is ``True`` (the default), the timer will be started and every
        ``timer_period_sec`` number of seconds the provided callback function will be called.
        If autostart is ``False``, the timer will be created but not started; it can then be
        started by calling ``reset()`` on the timer object.

        :param timer_period_sec: The period (in seconds) of the timer.
        :param callback: A user-defined callback function that is called when the timer expires.
        :param callback_group: The callback group for the timer. If ``None``, then the
            default callback group for the node is used.
        :param clock: The clock which the timer gets time from.
        :param autostart: Whether to automatically start the timer after creation; defaults to
            ``True``.
        """
        timer_period_nsec = int(float(timer_period_sec) * S_TO_NS)
        if callback_group is None:
            callback_group = self.default_callback_group
        if clock is None:
            clock = self._clock
        timer = Timer(
            callback, callback_group, timer_period_nsec, clock, context=self.context,
            autostart=autostart)

        callback_group.add_entity(timer)
        self._timers.append(timer)
        self._wake_executor()
        return timer

    def create_guard_condition(
        self,
        callback: Callable,
        callback_group: Optional[CallbackGroup] = None
    ) -> GuardCondition:
        """Create a new guard condition."""
        if callback_group is None:
            callback_group = self.default_callback_group
        guard = GuardCondition(callback, callback_group, context=self.context)

        callback_group.add_entity(guard)
        self._guards.append(guard)
        self._wake_executor()
        return guard

    def create_rate(
        self,
        frequency: float,
        clock: Optional[Clock] = None,
    ) -> Rate:
        """
        Create a Rate object.

        :param frequency: The frequency the Rate runs at (Hz).
        :param clock: The clock the Rate gets time from.
        """
        if frequency <= 0:
            raise ValueError('frequency must be > 0')
        # Create a timer and give it to the rate object
        period = 1.0 / frequency
        # Rate will set its own callback
        callback = None
        # Rates get their own group so timing is not messed up by other callbacks
        group = self._rate_group
        timer = self.create_timer(period, callback, group, clock)
        return Rate(timer, context=self.context)

    def destroy_publisher(self, publisher: Publisher) -> bool:
        """
        Destroy a publisher created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        if publisher in self._publishers:
            self._publishers.remove(publisher)
            for event_handler in publisher.event_handlers:
                self.__waitables.remove(event_handler)
            try:
                publisher.destroy()
            except InvalidHandle:
                return False
            self._wake_executor()
            return True
        return False

    def destroy_subscription(self, subscription: Subscription) -> bool:
        """
        Destroy a subscription created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        if subscription in self._subscriptions:
            self._subscriptions.remove(subscription)
            for event_handler in subscription.event_handlers:
                self.__waitables.remove(event_handler)
            try:
                subscription.destroy()
            except InvalidHandle:
                return False
            self._wake_executor()
            return True
        return False

    def destroy_client(self, client: Client) -> bool:
        """
        Destroy a service client created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        if client in self._clients:
            self._clients.remove(client)
            try:
                client.destroy()
            except InvalidHandle:
                return False
            self._wake_executor()
            return True
        return False

    def destroy_service(self, service: Service) -> bool:
        """
        Destroy a service server created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        if service in self._services:
            self._services.remove(service)
            try:
                service.destroy()
            except InvalidHandle:
                return False
            self._wake_executor()
            return True
        return False

    def destroy_timer(self, timer: Timer) -> bool:
        """
        Destroy a timer created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        if timer in self._timers:
            self._timers.remove(timer)
            try:
                timer.destroy()
            except InvalidHandle:
                return False
            self._wake_executor()
            return True
        return False

    def destroy_guard_condition(self, guard: GuardCondition) -> bool:
        """
        Destroy a guard condition created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        if guard in self._guards:
            self._guards.remove(guard)
            try:
                guard.destroy()
            except InvalidHandle:
                return False
            self._wake_executor()
            return True
        return False

    def destroy_rate(self, rate: Rate) -> bool:
        """
        Destroy a Rate object created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        success = self.destroy_timer(rate._timer)
        rate.destroy()
        return success

    def destroy_node(self):
        """
        Destroy the node.

        Frees resources used by the node, including any entities created by the following methods:

        * :func:`create_publisher`
        * :func:`create_subscription`
        * :func:`create_client`
        * :func:`create_service`
        * :func:`create_timer`
        * :func:`create_guard_condition`

        """
        # Drop extra reference to parameter event publisher.
        # It will be destroyed with other publishers below.
        self._parameter_event_publisher = None

        # Destroy dependent items eagerly to work around a possible hang
        # https://github.com/ros2/build_cop/issues/248
        while self._publishers:
            self.destroy_publisher(self._publishers[0])
        while self._subscriptions:
            self.destroy_subscription(self._subscriptions[0])
        while self._clients:
            self.destroy_client(self._clients[0])
        while self._services:
            self.destroy_service(self._services[0])
        while self._timers:
            self.destroy_timer(self._timers[0])
        while self._guards:
            self.destroy_guard_condition(self._guards[0])
        self._type_description_service.destroy()
        self.__node.destroy_when_not_in_use()
        self._wake_executor()

    def get_publisher_names_and_types_by_node(
        self,
        node_name: str,
        node_namespace: str,
        no_demangle: bool = False
    ) -> List[Tuple[str, List[str]]]:
        """
        Get a list of discovered topics for publishers of a remote node.

        :param node_name: Name of a remote node to get publishers for.
        :param node_namespace: Namespace of the remote node.
        :param no_demangle: If ``True``, then topic names and types returned will not be demangled.
        :return: List of tuples.
          The first element of each tuple is the topic name and the second element is a list of
          topic types.
        :raise NodeNameNonExistentError: If the node wasn't found.
        :raise RuntimeError: Unexpected failure.
        """
        with self.handle:
            return _rclpy.rclpy_get_publisher_names_and_types_by_node(
                self.handle, no_demangle, node_name, node_namespace)

    def get_subscriber_names_and_types_by_node(
        self,
        node_name: str,
        node_namespace: str,
        no_demangle: bool = False
    ) -> List[Tuple[str, List[str]]]:
        """
        Get a list of discovered topics for subscriptions of a remote node.

        :param node_name: Name of a remote node to get subscriptions for.
        :param node_namespace: Namespace of the remote node.
        :param no_demangle: If ``True``, then topic names and types returned will not be demangled.
        :return: List of tuples.
          The first element of each tuple is the topic name and the second element is a list of
          topic types.
        :raise NodeNameNonExistentError: If the node wasn't found.
        :raise RuntimeError: Unexpected failure.
        """
        with self.handle:
            return _rclpy.rclpy_get_subscriber_names_and_types_by_node(
                self.handle, no_demangle, node_name, node_namespace)

    def get_service_names_and_types_by_node(
        self,
        node_name: str,
        node_namespace: str
    ) -> List[Tuple[str, List[str]]]:
        """
        Get a list of discovered service servers for a remote node.

        :param node_name: Name of a remote node to get services for.
        :param node_namespace: Namespace of the remote node.
        :return: List of tuples.
          The first element of each tuple is the service server name
          and the second element is a list of service types.
        :raise NodeNameNonExistentError: If the node wasn't found.
        :raise RuntimeError: Unexpected failure.
        """
        with self.handle:
            return _rclpy.rclpy_get_service_names_and_types_by_node(
                self.handle, node_name, node_namespace)

    def get_client_names_and_types_by_node(
        self,
        node_name: str,
        node_namespace: str
    ) -> List[Tuple[str, List[str]]]:
        """
        Get a list of discovered service clients for a remote node.

        :param node_name: Name of a remote node to get service clients for.
        :param node_namespace: Namespace of the remote node.
        :return: List of tuples.
          The first element of each tuple is the service client name
          and the second element is a list of service client types.
        :raise NodeNameNonExistentError: If the node wasn't found.
        :raise RuntimeError: Unexpected failure.
        """
        with self.handle:
            return _rclpy.rclpy_get_client_names_and_types_by_node(
                self.handle, node_name, node_namespace)

    def get_topic_names_and_types(self, no_demangle: bool = False) -> List[Tuple[str, List[str]]]:
        """
        Get a list of discovered topic names and types.

        :param no_demangle: If ``True``, then topic names and types returned will not be demangled.
        :return: List of tuples.
          The first element of each tuple is the topic name and the second element is a list of
          topic types.
        """
        with self.handle:
            return _rclpy.rclpy_get_topic_names_and_types(self.handle, no_demangle)

    def get_service_names_and_types(self) -> List[Tuple[str, List[str]]]:
        """
        Get a list of discovered service names and types.

        :return: List of tuples.
          The first element of each tuple is the service name and the second element is a list of
          service types.
        """
        with self.handle:
            return _rclpy.rclpy_get_service_names_and_types(self.handle)

    def get_node_names(self) -> List[str]:
        """
        Get a list of names for discovered nodes.

        :return: List of node names.
        """
        with self.handle:
            names_ns = self.handle.get_node_names_and_namespaces()
        return [n[0] for n in names_ns]

    def get_fully_qualified_node_names(self) -> List[str]:
        """
        Get a list of fully qualified names for discovered nodes.

        Similar to ``get_node_names_namespaces()``, but concatenates the names and namespaces.

        :return: List of fully qualified node names.
        """
        names_and_namespaces = self.get_node_names_and_namespaces()
        return [
            ns + ('' if ns.endswith('/') else '/') + name
            for name, ns in names_and_namespaces
        ]

    def get_node_names_and_namespaces(self) -> List[Tuple[str, str]]:
        """
        Get a list of names and namespaces for discovered nodes.

        :return: List of tuples containing two strings: the node name and node namespace.
        """
        with self.handle:
            return self.handle.get_node_names_and_namespaces()

    def get_node_names_and_namespaces_with_enclaves(self) -> List[Tuple[str, str, str]]:
        """
        Get a list of names, namespaces and enclaves for discovered nodes.

        :return: List of tuples containing three strings: the node name, node namespace
            and enclave.
        """
        with self.handle:
            return self.handle.get_node_names_and_namespaces_with_enclaves()

    def get_fully_qualified_name(self) -> str:
        """
        Get the node's fully qualified name.

        :return: Fully qualified node name.
        """
        with self.handle:
            return self.handle.get_fully_qualified_name()

    def _count_publishers_or_subscribers(self, topic_name, func):
        fq_topic_name = expand_topic_name(topic_name, self.get_name(), self.get_namespace())
        validate_full_topic_name(fq_topic_name)
        with self.handle:
            return func(fq_topic_name)

    def count_publishers(self, topic_name: str) -> int:
        """
        Return the number of publishers on a given topic.

        ``topic_name`` may be a relative, private, or fully qualified topic name.
        A relative or private topic is expanded using this node's namespace and name.
        The queried topic name is not remapped.

        :param topic_name: The topic name on which to count the number of publishers.
        :return: The number of publishers on the topic.
        """
        with self.handle:
            return self._count_publishers_or_subscribers(
                topic_name, self.handle.get_count_publishers)

    def count_subscribers(self, topic_name: str) -> int:
        """
        Return the number of subscribers on a given topic.

        ``topic_name`` may be a relative, private, or fully qualified topic name.
        A relative or private topic is expanded using this node's namespace and name.
        The queried topic name is not remapped.

        :param topic_name: The topic name on which to count the number of subscribers.
        :return: The number of subscribers on the topic.
        """
        with self.handle:
            return self._count_publishers_or_subscribers(
                topic_name, self.handle.get_count_subscribers)

    def _count_clients_or_servers(self, service_name, func):
        fq_service_name = expand_topic_name(service_name, self.get_name(), self.get_namespace())
        validate_full_topic_name(fq_service_name, is_service=True)
        with self.handle:
            return func(fq_service_name)

    def count_clients(self, service_name: str) -> int:
        """
        Return the number of clients on a given service.

        `service_name` may be a relative, private, or fully qualified service name.
        A relative or private service is expanded using this node's namespace and name.
        The queried service name is not remapped.

        :param service_name: the service_name on which to count the number of clients.
        :return: the number of clients on the service.
        """
        with self.handle:
            return self._count_clients_or_servers(
                service_name, self.handle.get_count_clients)

    def count_services(self, service_name: str) -> int:
        """
        Return the number of servers on a given service.

        `service_name` may be a relative, private, or fully qualified service name.
        A relative or private service is expanded using this node's namespace and name.
        The queried service name is not remapped.

        :param service_name: the service_name on which to count the number of clients.
        :return: the number of servers on the service.
        """
        with self.handle:
            return self._count_clients_or_servers(
                service_name, self.handle.get_count_services)

    def _get_info_by_topic(
        self,
        topic_name: str,
        no_mangle: bool,
        func: Callable[[object, str, bool], List[Dict]]
    ) -> List[TopicEndpointInfo]:
        with self.handle:
            if no_mangle:
                fq_topic_name = topic_name
            else:
                fq_topic_name = expand_topic_name(
                    topic_name, self.get_name(), self.get_namespace())
                validate_full_topic_name(fq_topic_name)
                fq_topic_name = _rclpy.rclpy_remap_topic_name(self.handle, fq_topic_name)

            info_dicts = func(self.handle, fq_topic_name, no_mangle)
            infos = [TopicEndpointInfo(**x) for x in info_dicts]
            return infos

    def get_publishers_info_by_topic(
        self,
        topic_name: str,
        no_mangle: bool = False
    ) -> List[TopicEndpointInfo]:
        """
        Return a list of publishers on a given topic.

        The returned parameter is a list of TopicEndpointInfo objects, where each will contain
        the node name, node namespace, topic type, topic endpoint's GID, and its QoS profile.

        When the ``no_mangle`` parameter is ``True``, the provided ``topic_name`` should be a valid
        topic name for the middleware (useful when combining ROS with native middleware (e.g. DDS)
        apps).  When the ``no_mangle`` parameter is ``False``, the provided ``topic_name`` should
        follow ROS topic name conventions.

        ``topic_name`` may be a relative, private, or fully qualified topic name.
        A relative or private topic will be expanded using this node's namespace and name.
        The queried ``topic_name`` is not remapped.

        :param topic_name: The topic_name on which to find the publishers.
        :param no_mangle: If ``True``, ``topic_name`` needs to be a valid middleware topic
            name, otherwise it should be a valid ROS topic name. Defaults to ``False``.
        :return: a list of TopicEndpointInfo for all the publishers on this topic.
        """
        return self._get_info_by_topic(
            topic_name,
            no_mangle,
            _rclpy.rclpy_get_publishers_info_by_topic)

    def get_subscriptions_info_by_topic(
        self,
        topic_name: str,
        no_mangle: bool = False
    ) -> List[TopicEndpointInfo]:
        """
        Return a list of subscriptions on a given topic.

        The returned parameter is a list of TopicEndpointInfo objects, where each will contain
        the node name, node namespace, topic type, topic endpoint's GID, and its QoS profile.

        When the ``no_mangle`` parameter is ``True``, the provided ``topic_name`` should be a valid
        topic name for the middleware (useful when combining ROS with native middleware (e.g. DDS)
        apps).  When the ``no_mangle`` parameter is ``False``, the provided ``topic_name`` should
        follow ROS topic name conventions.

        ``topic_name`` may be a relative, private, or fully qualified topic name.
        A relative or private topic will be expanded using this node's namespace and name.
        The queried ``topic_name`` is not remapped.

        :param topic_name: The topic_name on which to find the subscriptions.
        :param no_mangle: If ``True``, `topic_name` needs to be a valid middleware topic
            name, otherwise it should be a valid ROS topic name. Defaults to ``False``.
        :return: A list of TopicEndpointInfo for all the subscriptions on this topic.
        """
        return self._get_info_by_topic(
            topic_name,
            no_mangle,
            _rclpy.rclpy_get_subscriptions_info_by_topic)

    def wait_for_node(
        self,
        fully_qualified_node_name: str,
        timeout: float
    ) -> bool:
        """
        Wait until node name is present in the system or timeout.

        The node name should be the full name with namespace.

        :param node_name: Fully qualified name of the node to wait for.
        :param timeout: Seconds to wait for the node to be present. If negative, the function
                         won't timeout.
        :return: ``True`` if the node was found, ``False`` if timeout.
        """
        if not fully_qualified_node_name.startswith('/'):
            fully_qualified_node_name = f'/{fully_qualified_node_name}'

        start = time.time()
        flag = False
        # TODO refactor this implementation when we can react to guard condition events, or replace
        # it entirely with an implementation in rcl. see https://github.com/ros2/rclpy/issues/929
        while time.time() - start < timeout and not flag:
            fully_qualified_node_names = self.get_fully_qualified_node_names()
            flag = fully_qualified_node_name in fully_qualified_node_names
            time.sleep(0.1)
        return flag
