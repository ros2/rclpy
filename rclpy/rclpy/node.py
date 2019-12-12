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

from typing import Any
from typing import Callable
from typing import Dict
from typing import Iterator
from typing import List
from typing import Optional
from typing import Tuple
from typing import TypeVar
from typing import Union

import weakref

from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import SetParametersResult

from rclpy.callback_groups import CallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.clock import Clock
from rclpy.clock import ROSClock
from rclpy.constants import S_TO_NS
from rclpy.context import Context
from rclpy.exceptions import InvalidParameterValueException
from rclpy.exceptions import NotInitializedException
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.exceptions import ParameterImmutableException
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import Executor
from rclpy.expand_topic_name import expand_topic_name
from rclpy.guard_condition import GuardCondition
from rclpy.handle import Handle
from rclpy.handle import InvalidHandle
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.logging import get_logger
from rclpy.parameter import Parameter, PARAMETER_SEPARATOR_STRING
from rclpy.parameter_service import ParameterService
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_parameter_events
from rclpy.qos import qos_profile_services_default
from rclpy.qos import QoSProfile
from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.time_source import TimeSource
from rclpy.timer import Rate
from rclpy.timer import Timer
from rclpy.type_support import check_for_type_support
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

    PARAM_REL_TOL = 1e-6

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
        enable_rosout: bool = True,
        start_parameter_services: bool = True,
        parameter_overrides: List[Parameter] = None,
        allow_undeclared_parameters: bool = False,
        automatically_declare_parameters_from_overrides: bool = False
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
        """
        self.__handle = None
        self._context = get_default_context() if context is None else context
        self._parameters: dict = {}
        self.__publishers: List[Publisher] = []
        self.__subscriptions: List[Subscription] = []
        self.__clients: List[Client] = []
        self.__services: List[Service] = []
        self.__timers: List[Timer] = []
        self.__guards: List[GuardCondition] = []
        self.__waitables: List[Waitable] = []
        self._default_callback_group = MutuallyExclusiveCallbackGroup()
        self._rate_group = ReentrantCallbackGroup()
        self._parameters_callback = None
        self._allow_undeclared_parameters = allow_undeclared_parameters
        self._parameter_overrides = {}
        self._descriptors = {}

        namespace = namespace or ''
        if not self._context.ok():
            raise NotInitializedException('cannot create node')
        try:
            self.__handle = Handle(_rclpy.rclpy_create_node(
                node_name,
                namespace,
                self._context.handle,
                cli_args,
                use_global_arguments,
                enable_rosout
            ))
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
        with self.handle as capsule:
            self._logger = get_logger(_rclpy.rclpy_get_node_logger_name(capsule))

        self.__executor_weakref = None

        self._parameter_event_publisher = self.create_publisher(
            ParameterEvent, 'parameter_events', qos_profile_parameter_events)

        with self.handle as capsule:
            self._parameter_overrides = _rclpy.rclpy_get_node_parameters(Parameter, capsule)
        # Combine parameters from params files with those from the node constructor and
        # use the set_parameters_atomically API so a parameter event is published.
        if parameter_overrides is not None:
            self._parameter_overrides.update({p.name: p for p in parameter_overrides})

        if automatically_declare_parameters_from_overrides:
            self._parameters.update(self._parameter_overrides)
            self._descriptors.update({p: ParameterDescriptor() for p in self._parameters})

        # Clock that has support for ROS time.
        # Note: parameter overrides and parameter event publisher need to be ready at this point
        # to be able to declare 'use_sim_time' if it was not declared yet.
        self._clock = ROSClock()
        self._time_source = TimeSource(node=self)
        self._time_source.attach_clock(self._clock)

        if start_parameter_services:
            self._parameter_service = ParameterService(self)

    @property
    def publishers(self) -> Iterator[Publisher]:
        """Get publishers that have been created on this node."""
        yield from self.__publishers

    @property
    def subscriptions(self) -> Iterator[Subscription]:
        """Get subscriptions that have been created on this node."""
        yield from self.__subscriptions

    @property
    def clients(self) -> Iterator[Client]:
        """Get clients that have been created on this node."""
        yield from self.__clients

    @property
    def services(self) -> Iterator[Service]:
        """Get services that have been created on this node."""
        yield from self.__services

    @property
    def timers(self) -> Iterator[Timer]:
        """Get timers that have been created on this node."""
        yield from self.__timers

    @property
    def guards(self) -> Iterator[GuardCondition]:
        """Get guards that have been created on this node."""
        yield from self.__guards

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
        return self.__handle

    @handle.setter
    def handle(self, value):
        raise AttributeError('handle cannot be modified after node creation')

    def get_name(self) -> str:
        """Get the name of the node."""
        with self.handle as capsule:
            return _rclpy.rclpy_get_node_name(capsule)

    def get_namespace(self) -> str:
        """Get the namespace of the node."""
        with self.handle as capsule:
            return _rclpy.rclpy_get_node_namespace(capsule)

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
        descriptor: ParameterDescriptor = ParameterDescriptor(),
        ignore_override: bool = False
    ) -> Parameter:
        """
        Declare and initialize a parameter.

        This method, if successful, will result in any callback registered with
        :func:`set_parameters_callback` to be called.

        :param name: Fully-qualified name of the parameter, including its namespace.
        :param value: Value of the parameter to declare.
        :param descriptor: Descriptor for the parameter to declare.
        :param ignore_override: True if overrides shall not be taken into account; False otherwise.
        :return: Parameter with the effectively assigned value.
        :raises: ParameterAlreadyDeclaredException if the parameter had already been declared.
        :raises: InvalidParameterException if the parameter name is invalid.
        :raises: InvalidParameterValueException if the registered callback rejects the parameter.
        """
        return self.declare_parameters('', [(name, value, descriptor)], ignore_override)[0]

    def declare_parameters(
        self,
        namespace: str,
        parameters: List[Union[
            Tuple[str],
            Tuple[str, Any],
            Tuple[str, Any, ParameterDescriptor],
        ]],
        ignore_override: bool = False
    ) -> List[Parameter]:
        """
        Declare a list of parameters.

        The tuples in the given parameter list shall contain the name for each parameter,
        optionally providing a value and a descriptor.
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

        This method, if successful, will result in any callback registered with
        :func:`set_parameters_callback` to be called once for each parameter.
        If one of those calls fail, an exception will be raised and the remaining parameters will
        not be declared.
        Parameters declared up to that point will not be undeclared.

        :param namespace: Namespace for parameters.
        :param parameters: List of tuples with parameters to declare.
        :param ignore_override: True if overrides shall not be taken into account; False otherwise.
        :return: Parameter list with the effectively assigned values for each of them.
        :raises: ParameterAlreadyDeclaredException if the parameter had already been declared.
        :raises: InvalidParameterException if the parameter name is invalid.
        :raises: InvalidParameterValueException if the registered callback rejects any parameter.
        :raises: TypeError if any tuple in :param:`parameters` does not match the annotated type.
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
            descriptor = ParameterDescriptor()

            # Get the values from the tuple, checking its types.
            # Use defaults if the tuple doesn't contain value and / or descriptor.
            try:
                name = parameter_tuple[0]
                assert \
                    isinstance(name, str), \
                    (
                        'First element {name} at index {index} in parameters list '
                        'is not a str.'.format_map(locals())
                    )

                # Get value from parameter overrides, of from tuple if it doesn't exist.
                if not ignore_override and name in self._parameter_overrides:
                    value = self._parameter_overrides[name].value
                else:
                    # This raises a TypeError if it's not possible to get a type from the tuple.
                    value = parameter_tuple[1]

                # Get descriptor from tuple.
                descriptor = parameter_tuple[2]
                assert \
                    isinstance(descriptor, ParameterDescriptor), \
                    (
                        'Third element {descriptor} at index {index} in parameters list '
                        'is not a ParameterDescriptor.'.format_map(locals())
                    )
            except AssertionError as assertion_error:
                raise TypeError(assertion_error)
            except IndexError:
                # This means either value or descriptor were not defined which is fine.
                pass

            if namespace:
                name = '{namespace}.{name}'.format_map(locals())

            # Note(jubeira): declare_parameters verifies the name, but set_parameters doesn't.
            validate_parameter_name(name)

            parameter_list.append(Parameter(name, value=value))
            descriptors.update({name: descriptor})

        parameters_already_declared = [
            parameter.name for parameter in parameter_list if parameter.name in self._parameters
        ]
        if any(parameters_already_declared):
            raise ParameterAlreadyDeclaredException(parameters_already_declared)

        # Call the callback once for each of the parameters, using method that doesn't
        # check whether the parameter was declared beforehand or not.
        self._set_parameters(
            parameter_list,
            descriptors,
            raise_on_failure=True,
            allow_undeclared_parameters=True
        )
        return self.get_parameters([parameter.name for parameter in parameter_list])

    def undeclare_parameter(self, name: str):
        """
        Undeclare a previously declared parameter.

        This method will not cause a callback registered with
        :func:`set_parameters_callback` to be called.

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

    def get_parameters(self, names: List[str]) -> List[Parameter]:
        """
        Get a list of parameters.

        :param names: Fully-qualified names of the parameters to get, including their namespaces.
        :return: The values for the given parameter names.
            A default Parameter will be returned for undeclared parameters if
            undeclared parameters are allowed.
        :raises: ParameterNotDeclaredException if undeclared parameters are not allowed,
            and at least one parameter hadn't been declared beforehand.
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
        """
        if self.has_parameter(name):
            return self._parameters[name]
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
        :return: Requested parameter, or alternative value if it hadn't been declared before.
        """
        if alternative_value is None:
            alternative_value = Parameter(name, Parameter.Type.NOT_SET)

        return self._parameters.get(name, alternative_value)

    def get_parameters_by_prefix(self, prefix: str) -> Dict[str, Parameter]:
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
        parameters_with_prefix = {}
        if prefix:
            prefix = prefix + PARAMETER_SEPARATOR_STRING
        prefix_len = len(prefix)
        for parameter_name in self._parameters:
            if parameter_name.startswith(prefix):
                parameters_with_prefix.update(
                    {parameter_name[prefix_len:]: self._parameters.get(parameter_name)})

        return parameters_with_prefix

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

        If a callback was registered previously with :func:`set_parameters_callback`, it will be
        called prior to setting the parameters for the node, once for each parameter.
        If the callback prevents a parameter from being set, then it will be reflected in the
        returned result; no exceptions will be raised in this case.
        For each successfully set parameter, a :class:`ParameterEvent` message is
        published.

        If the value type of the parameter is NOT_SET, and the existing parameter type is
        something else, then the parameter will be implicitly undeclared.

        :param parameter_list: The list of parameters to set.
        :return: The result for each set action as a list.
        :raises: ParameterNotDeclaredException if undeclared parameters are not allowed,
            and at least one parameter in the list hadn't been declared beforehand.
        """
        return self._set_parameters(parameter_list)

    def _set_parameters(
        self,
        parameter_list: List[Parameter],
        descriptors: Optional[Dict[str, ParameterDescriptor]] = None,
        raise_on_failure: bool = False,
        allow_undeclared_parameters: bool = False
    ) -> List[SetParametersResult]:
        """
        Set parameters for the node, and return the result for the set action.

        Method for internal usage; applies a setter method for each parameters in the list.
        By default it checks if the parameters were declared, raising an exception if at least
        one of them was not.

        If a callback was registered previously with :func:`set_parameters_callback`, it will be
        called prior to setting the parameters for the node, once for each parameter.
        If the callback doesn't succeed for a given parameter, it won't be set and either an
        unsuccessful result will be returned for that parameter, or an exception will be raised
        according to `raise_on_failure` flag.

        :param parameter_list: List of parameters to set.
        :param descriptors: Descriptors to set to the given parameters.
            If descriptors are given, each parameter in the list must have a corresponding one.
        :param raise_on_failure: True if InvalidParameterValueException has to be raised when
            the user callback rejects a parameter, False otherwise.
        :param allow_undeclared_parameters: If False, this method will check for undeclared
            parameters for each of the elements in the parameter list.
        :return: The result for each set action as a list.
        :raises: InvalidParameterValueException if the user-defined callback rejects the
            parameter value and raise_on_failure flag is True.
        :raises: ParameterNotDeclaredException if undeclared parameters are not allowed in this
            method and at least one parameter in the list hadn't been declared beforehand.
        """
        if descriptors is not None:
            assert all(parameter.name in descriptors for parameter in parameter_list)

        results = []
        for param in parameter_list:
            if not allow_undeclared_parameters:
                self._check_undeclared_parameters([param])
            # If undeclared parameters are allowed, parameters with type NOT_SET shall be stored.
            result = self._set_parameters_atomically(
                [param],
                descriptors,
                allow_not_set_type=allow_undeclared_parameters
            )
            if raise_on_failure and not result.successful:
                raise InvalidParameterValueException(param.name, param.value, result.reason)
            results.append(result)
        return results

    def set_parameters_atomically(self, parameter_list: List[Parameter]) -> SetParametersResult:
        """
        Set the given parameters, all at one time, and then aggregate result.

        If any parameter in the list was not declared beforehand and undeclared parameters are not
        allowed for the node, this method will raise a ParameterNotDeclaredException exception.

        Parameters are set all at once.
        If setting a parameter fails due to not being declared, then no parameter will be set set.
        Either all of the parameters are set or none of them are set.

        If undeclared parameters are allowed for the node, then all the parameters will be
        implicitly declared before being set even if they were not declared beforehand.

        If a callback was registered previously with :func:`set_parameters_callback`, it will be
        called prior to setting the parameters for the node only once for all parameters.
        If the callback prevents the parameters from being set, then it will be reflected in the
        returned result; no exceptions will be raised in this case.
        For each successfully set parameter, a :class:`ParameterEvent` message is published.

        If the value type of the parameter is NOT_SET, and the existing parameter type is
        something else, then the parameter will be implicitly undeclared.

        :param parameter_list: The list of parameters to set.
        :return: Aggregate result of setting all the parameters atomically.
        :raises: ParameterNotDeclaredException if undeclared parameters are not allowed,
            and at least one parameter in the list hadn't been declared beforehand.
        """
        self._check_undeclared_parameters(parameter_list)
        return self._set_parameters_atomically(parameter_list)

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
        if (not self._allow_undeclared_parameters and any(undeclared_parameters)):
            raise ParameterNotDeclaredException(list(undeclared_parameters))

    def _set_parameters_atomically(
        self,
        parameter_list: List[Parameter],
        descriptors: Optional[Dict[str, ParameterDescriptor]] = None,
        allow_not_set_type: bool = False
    ) -> SetParametersResult:
        """
        Set the given parameters, all at one time, and then aggregate result.

        This internal method does not reject undeclared parameters.
        If :param:`allow_not_set_type` is False, a parameter with type NOT_SET will be undeclared.

        If a callback was registered previously with :func:`set_parameters_callback`, it will be
        called prior to setting the parameters for the node only once for all parameters.
        If the callback prevents the parameters from being set, then it will be reflected in the
        returned result; no exceptions will be raised in this case.
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
        elif self._parameters_callback:
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
                        self._descriptors[param.name] = ParameterDescriptor()

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

        return result

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
        :param check_read_only: True if read-only check has to be applied.
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
        :param descriptor: Descriptor to apply. If None, the stored descriptor for the given
            parameter's name is used instead.
        :param check_read_only: True if read-only check has to be applied.
        :return: SetParametersResult; successful if checks passed, unsuccessful otherwise.
        :raises: ParameterNotDeclaredException if a descriptor is not provided, the given parameter
            name had not been declared and undeclared parameters are not allowed.
        """
        if descriptor is None:
            descriptor = self.describe_parameter(parameter.name)
        else:
            descriptor.name = parameter.name

        # The type in the descriptor has to match the type of the parameter.
        descriptor.type = parameter.type_.value

        if check_read_only and descriptor.read_only:
            return SetParametersResult(
                successful=False,
                reason='Trying to set a read-only parameter: {}.'.format(parameter.name))

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

        The name in the descriptor is ignored and set to :param:`name`.

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

    def create_publisher(
        self,
        msg_type,
        topic: str,
        qos_profile: Union[QoSProfile, int],
        *,
        callback_group: Optional[CallbackGroup] = None,
        event_callbacks: Optional[PublisherEventCallbacks] = None,
    ) -> Publisher:
        """
        Create a new publisher.

        :param msg_type: The type of ROS messages the publisher will publish.
        :param topic: The name of the topic the publisher will publish to.
        :param qos_profile: A QoSProfile or a history depth to apply to the publisher.
          In the case that a history depth is provided, the QoS history is set to
          RMW_QOS_POLICY_HISTORY_KEEP_LAST, the QoS history depth is set to the value
          of the parameter, and all other QoS settings are set to their default values.
        :param callback_group: The callback group for the publisher's event handlers.
            If ``None``, then the node's default callback group is used.
        :param event_callbacks: User-defined callbacks for middleware events.
        :return: The new publisher.
        """
        qos_profile = self._validate_qos_or_depth_parameter(qos_profile)

        callback_group = callback_group or self.default_callback_group

        # this line imports the typesupport for the message module if not already done
        check_for_type_support(msg_type)
        failed = False
        try:
            with self.handle as node_capsule:
                publisher_capsule = _rclpy.rclpy_create_publisher(
                    node_capsule, msg_type, topic, qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(topic)

        publisher_handle = Handle(publisher_capsule)
        publisher_handle.requires(self.handle)

        publisher = Publisher(
            publisher_handle, msg_type, topic, qos_profile,
            event_callbacks=event_callbacks or PublisherEventCallbacks(),
            callback_group=callback_group)
        self.__publishers.append(publisher)
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
          RMW_QOS_POLICY_HISTORY_KEEP_LAST, the QoS history depth is set to the value
          of the parameter, and all other QoS settings are set to their default values.
        :param callback_group: The callback group for the subscription. If ``None``, then the
            nodes default callback group is used.
        :param event_callbacks: User-defined callbacks for middleware events.
        :param raw: If ``True``, then received messages will be stored in raw binary
            representation.
        """
        qos_profile = self._validate_qos_or_depth_parameter(qos_profile)

        callback_group = callback_group or self.default_callback_group

        # this line imports the typesupport for the message module if not already done
        check_for_type_support(msg_type)
        failed = False
        try:
            with self.handle as capsule:
                subscription_capsule = _rclpy.rclpy_create_subscription(
                    capsule, msg_type, topic, qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(topic)

        subscription_handle = Handle(subscription_capsule)
        subscription_handle.requires(self.handle)

        subscription = Subscription(
            subscription_handle, msg_type,
            topic, callback, callback_group, qos_profile, raw,
            event_callbacks=event_callbacks or SubscriptionEventCallbacks())
        self.__subscriptions.append(subscription)
        callback_group.add_entity(subscription)

        for event_handler in subscription.event_handlers:
            self.add_waitable(event_handler)

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
            with self.handle as node_capsule:
                client_capsule = _rclpy.rclpy_create_client(
                    node_capsule,
                    srv_type,
                    srv_name,
                    qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(srv_name, is_service=True)

        client_handle = Handle(client_capsule)
        client_handle.requires(self.handle)

        client = Client(
            self.context,
            client_handle, srv_type, srv_name, qos_profile,
            callback_group)
        self.__clients.append(client)
        callback_group.add_entity(client)
        self._wake_executor()
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
            with self.handle as node_capsule:
                service_capsule = _rclpy.rclpy_create_service(
                    node_capsule,
                    srv_type,
                    srv_name,
                    qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(srv_name, is_service=True)

        service_handle = Handle(service_capsule)
        service_handle.requires(self.handle)

        service = Service(
            service_handle,
            srv_type, srv_name, callback, callback_group, qos_profile)
        self.__services.append(service)
        callback_group.add_entity(service)
        self._wake_executor()
        return service

    def create_timer(
        self,
        timer_period_sec: float,
        callback: Callable,
        callback_group: CallbackGroup = None,
        clock: Clock = None,
    ) -> Timer:
        """
        Create a new timer.

        The timer will be started and every ``timer_period_sec`` number of seconds the provided
        callback function will be called.

        :param timer_period_sec: The period (s) of the timer.
        :param callback: A user-defined callback function that is called when the timer expires.
        :param callback_group: The callback group for the timer. If ``None``, then the nodes
            default callback group is used.
        :param clock: The clock which the timer gets time from.
        """
        timer_period_nsec = int(float(timer_period_sec) * S_TO_NS)
        if callback_group is None:
            callback_group = self.default_callback_group
        if clock is None:
            clock = self._clock
        timer = Timer(callback, callback_group, timer_period_nsec, clock, context=self.context)
        timer.handle.requires(self.handle)

        self.__timers.append(timer)
        callback_group.add_entity(timer)
        self._wake_executor()
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
        guard.handle.requires(self.handle)

        self.__guards.append(guard)
        callback_group.add_entity(guard)
        self._wake_executor()
        return guard

    def create_rate(
        self,
        frequency: float,
        clock: Clock = None,
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
        if publisher in self.__publishers:
            self.__publishers.remove(publisher)
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

        :return: ``True`` if succesful, ``False`` otherwise.
        """
        if subscription in self.__subscriptions:
            self.__subscriptions.remove(subscription)
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
        if client in self.__clients:
            self.__clients.remove(client)
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
        if service in self.__services:
            self.__services.remove(service)
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
        if timer in self.__timers:
            self.__timers.remove(timer)
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
        if guard in self.__guards:
            self.__guards.remove(guard)
            try:
                guard.destroy()
            except InvalidHandle:
                return False
            self._wake_executor()
            return True
        return False

    def destroy_rate(self, rate: Rate):
        """
        Destroy a Rate object created by the node.

        :return: ``True`` if successful, ``False`` otherwise.
        """
        self.destroy_timer(rate._timer)
        rate.destroy()

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

        """
        # Drop extra reference to parameter event publisher.
        # It will be destroyed with other publishers below.
        self._parameter_event_publisher = None

        # Destroy dependent items eagerly to work around a possible hang
        # https://github.com/ros2/build_cop/issues/248
        while self.__publishers:
            self.destroy_publisher(self.__publishers[0])
        while self.__subscriptions:
            self.destroy_subscription(self.__subscriptions[0])
        while self.__clients:
            self.destroy_client(self.__clients[0])
        while self.__services:
            self.destroy_service(self.__services[0])
        while self.__timers:
            self.destroy_timer(self.__timers[0])
        while self.__guards:
            self.destroy_guard_condition(self.__guards[0])
        self.handle.destroy()
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
        with self.handle as capsule:
            return _rclpy.rclpy_get_publisher_names_and_types_by_node(
                capsule, no_demangle, node_name, node_namespace)

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
        with self.handle as capsule:
            return _rclpy.rclpy_get_subscriber_names_and_types_by_node(
                capsule, no_demangle, node_name, node_namespace)

    def get_service_names_and_types_by_node(
        self,
        node_name: str,
        node_namespace: str
    ) -> List[Tuple[str, List[str]]]:
        """
        Get a list of discovered service server topics for a remote node.

        :param node_name: Name of a remote node to get services for.
        :param node_namespace: Namespace of the remote node.
        :return: List of tuples.
          The first element of each tuple is the service server name
          and the second element is a list of service types.
        :raise NodeNameNonExistentError: If the node wasn't found.
        :raise RuntimeError: Unexpected failure.
        """
        with self.handle as capsule:
            return _rclpy.rclpy_get_service_names_and_types_by_node(
                capsule, node_name, node_namespace)

    def get_client_names_and_types_by_node(
        self,
        node_name: str,
        node_namespace: str
    ) -> List[Tuple[str, List[str]]]:
        """
        Get a list of discovered service client topics for a remote node.

        :param node_name: Name of a remote node to get service clients for.
        :param node_namespace: Namespace of the remote node.
        :return: List of tuples.
          The fist element of each tuple is the service client name
          and the second element is a list of service client types.
        :raise NodeNameNonExistentError: If the node wasn't found.
        :raise RuntimeError: Unexpected failure.
        """
        with self.handle as capsule:
            return _rclpy.rclpy_get_client_names_and_types_by_node(
                capsule, node_name, node_namespace)

    def get_topic_names_and_types(self, no_demangle: bool = False) -> List[Tuple[str, List[str]]]:
        """
        Get a list topic names and types for the node.

        :param no_demangle: If ``True``, then topic names and types returned will not be demangled.
        :return: List of tuples.
          The first element of each tuple is the topic name and the second element is a list of
          topic types.
        """
        with self.handle as capsule:
            return _rclpy.rclpy_get_topic_names_and_types(capsule, no_demangle)

    def get_service_names_and_types(self) -> List[Tuple[str, List[str]]]:
        """
        Get a list of service topics for the node.

        :return: List of tuples.
          The first element of each tuple is the service name and the second element is a list of
          service types.
        """
        with self.handle as capsule:
            return _rclpy.rclpy_get_service_names_and_types(capsule)

    def get_node_names(self) -> List[str]:
        """
        Get a list of names for discovered nodes.

        :return: List of node names.
        """
        with self.handle as capsule:
            names_ns = _rclpy.rclpy_get_node_names_and_namespaces(capsule)
        return [n[0] for n in names_ns]

    def get_node_names_and_namespaces(self) -> List[Tuple[str, str]]:
        """
        Get a list of names and namespaces for discovered nodes.

        :return: List of tuples containing two strings: the node name and node namespace.
        """
        with self.handle as capsule:
            return _rclpy.rclpy_get_node_names_and_namespaces(capsule)

    def _count_publishers_or_subscribers(self, topic_name, func):
        fq_topic_name = expand_topic_name(topic_name, self.get_name(), self.get_namespace())
        validate_topic_name(fq_topic_name)
        with self.handle as node_capsule:
            return func(node_capsule, fq_topic_name)

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

    def assert_liveliness(self) -> None:
        """
        Manually assert that this Node is alive.

        If the QoS Liveliness policy is set to RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE, the
        application must call this at least as often as ``QoSProfile.liveliness_lease_duration``.
        """
        with self.handle as capsule:
            _rclpy.rclpy_assert_liveliness(capsule)
