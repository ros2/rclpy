# Copyright 2023 Open Source Robotics Foundation, Inc.
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

from collections import defaultdict
from itertools import chain
from multiprocessing import Lock
from typing import Callable, Dict, Iterable, List, Optional, Tuple


from rcl_interfaces.msg import ParameterEvent
from rclpy.callback_groups import CallbackGroup
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_parameter_events
from rclpy.qos import QoSProfile
from rclpy.qos_overriding_options import QoSOverridingOptions


class ParameterCallbackHandle:
    def __init__(
        self,
        parameter_name: str,
        node_name: str,
        callback: Callable[[Parameter], None],
    ):
        self.parameter_name = parameter_name
        self.node_name = node_name
        self.callback = callback
        self.mutex = Lock()


class ParameterEventCallbackHandle:
    def __init__(
        self,
        callback: Callable[[ParameterEvent], None]
    ):
        self.callback = callback
        self.mutex = Lock()


class ParameterEventHandler:

    class Callbacks:
        def __init__(
            self
        ):
            """
            Create a Callbacks container for ParameterEventHandler.

            Callbacks container is used to store and manage parameter
            and parameter-event callbacks
            """
            self.parameter_callbacks: Dict[
                Tuple[str, str], List[ParameterCallbackHandle]
            ] = defaultdict(list)
            self.event_callbacks: List[ParameterEventCallbackHandle] = []
            self.mutex = Lock()

        def event_callback(self, event: ParameterEvent):
            """
            Search for callback and execute it.

            Method to be utilized by ParameterEventHandler object
            as a callback for subscription to a /parameter_events topic.
            Used to traverse parameter_callbacks dict and perform callbacks,
            related to Parameter, specified in event.

            :param event: ParameterEvent message.
                By design, originates from /parameter_events topic
            """
            with self.mutex:
                for (param_name, node_name), callbacks_list in self.parameter_callbacks.items():
                    if parameter := ParameterEventHandler.get_parameter_from_event(
                        event, parameter_name=param_name, node_name=node_name
                    ):
                        for callback_handle in callbacks_list:
                            with callback_handle.mutex:
                                callback_handle.callback(parameter)

                for event_callback in self.event_callbacks:
                    with event_callback.mutex:
                        event_callback.callback(event)

        def add_parameter_callback(
            self,
            parameter_name: str,
            node_name: str,
            callback: Callable[[Parameter], None],
        ) -> ParameterCallbackHandle:
            """
            Add new parameter callback.

            Callbacks are called in FILO manner.

            :param parameter_name: Name of a parameter to bind the callback to
            :param node_name: Name of a node, that the parameter should be related to
            :param callback: A callable to be called when a parameter event occurs

            :return ParameterCallbackHandle: A handle that should be saved by the user
                so that the callback can be later removed.
            """
            handle = ParameterCallbackHandle(
                parameter_name=parameter_name,
                node_name=node_name,
                callback=callback,
            )

            with self.mutex:
                self.parameter_callbacks[(parameter_name, node_name)].insert(0, handle)

            return handle

        def remove_parameter_callback(
            self,
            handle: ParameterCallbackHandle,
        ) -> None:
            """
            Remove the parameter callback related to provided handle.

            :param handle: The handle of the callback that is to be removed
            """
            with self.mutex:
                handle_key = (handle.parameter_name, handle.node_name)
                if handle_key in self.parameter_callbacks:
                    if handle in self.parameter_callbacks[handle_key]:
                        self.parameter_callbacks[handle_key].remove(handle)
                    else:
                        raise RuntimeError("Callback doesn't exist")

                    if len(self.parameter_callbacks[handle_key]) == 0:
                        self.parameter_callbacks.pop(handle_key)
                else:
                    raise RuntimeError("Callback doesn't exist")

        def add_parameter_event_callback(
            self,
            callback: Callable[[ParameterEvent], None],
        ) -> ParameterEventCallbackHandle:
            """
            Add new parameter event callback.

            Callbacks are called in FILO manner.

            :param callback: A callable to be referenced on a ParameterEvent

            :return ParameterEventCallbackHandle: A handle that should be saved by the user
                so that the event_callback can be later removed.
            """
            handle = ParameterEventCallbackHandle(callback=callback)

            with self.mutex:
                self.event_callbacks.insert(0, handle)

            return handle

        def remove_parameter_event_callback(
            self,
            handle: ParameterEventCallbackHandle,
        ) -> None:
            """
            Remove the parameter event callback related to provided handle.

            :param handle: A handle of the callback that is to be removed
            """
            with self.mutex:
                if handle in self.event_callbacks:
                    self.event_callbacks.remove(handle)
                else:
                    raise RuntimeError("Callback doesn't exist")

    def __init__(
        self,
        node: Node,
        qos_profile: QoSProfile = qos_profile_parameter_events,
        callback_group: Optional[CallbackGroup] = None,
        event_callbacks: Optional[SubscriptionEventCallbacks] = None,
        qos_overriding_options: Optional[QoSOverridingOptions] = None,
        raw: bool = False,
    ):
        """
        Create ParameterEventHandler.

        Usage example:

        .. code-block:: python
            import rclpy
            from rclpy.parameter_event_handler import ParameterEventHandler

            handler = ParameterEventHandler(node)

            # Add parameter callback
            handle = handler.add_parameter_callback(
                parameter_name="example_parameter",
                node_name="example",
                callback=example_callable,
            )

            # Remove parameter callback
            handler.remove_parameter_callback(handle)

            # Add parameter event callback
            handle = handler.add_parameter_event_callback(
                callback=example_callable,
            )

            # Remove parameter event callback
            handler.remove_parameter_event_callback(handle)
        ..

        A class used to "handle" (monitor and respond to) changes to parameters.
        :param node: Used to subscribe to parameter_events topic
        :param qos_profile: A QoSProfile or a history depth to apply to the subscription.
            In the case that a history depth is provided, the QoS history is set to
            KEEP_LAST, the QoS history depth is set to the value
            of the parameter, and all other QoS settings are set to their default values.
        :param callback_group: The callback group for the subscription. If ``None``, then the
            default callback group for the node is used.
        :param event_callbacks: User-defined callbacks for middleware events.
        :param qos_overriding_options: Options to customize QoS parameter overrides.
        :param raw: If ``True``, then received messages will be stored in raw binary
            representation.
        """
        self.node = node
        self.qos_profile = qos_profile

        self._callbacks = ParameterEventHandler.Callbacks()

        self.parameter_event_subscription = node.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self._callbacks.event_callback,
            self.qos_profile,
            callback_group=callback_group,
            event_callbacks=event_callbacks,
            qos_overriding_options=qos_overriding_options,
            raw=raw,
        )

    def destroy(self):
        self.node.destroy_subscription(
            self.parameter_event_subscription
        )

    @staticmethod
    def get_parameter_from_event(
        event: ParameterEvent,
        parameter_name: str,
        node_name: str,
    ) -> Optional[Parameter]:
        """
        Get specified parameter value from ParameterEvent message.

        :param event: ParameterEvent message to be read
        :param parameter_name: Name of a parameter to get from ParameterEvent message
        :param node_name: Name of a node, that the parameter should be related to

        :return Optional[Parameter]: If specified parameter is found, returns Parameter object.
                                     Otherwise, returns None
        """
        if event.node == node_name:
            for parameter in chain(event.new_parameters, event.changed_parameters):
                if parameter.name == parameter_name:
                    return parameter

        return None

    @staticmethod
    def get_parameters_from_event(event: ParameterEvent) -> Iterable[Parameter]:
        """
        Get all parameters from a ParameterEvent message.

        :param event: ParameterEvent message to read
        """
        for parameter in chain(event.new_parameters, event.changed_parameters):
            yield parameter

    def add_parameter_callback(
        self,
        parameter_name: str,
        node_name: str,
        callback: Callable[[Parameter], None],
    ) -> ParameterCallbackHandle:
        """
        Add new parameter callback.

        Callbacks are called in FILO manner.

        :param parameter_name: Name of a parameter to tie callback to
        :param node_name: Name of a node, that the parameter should be related to
        :param callback: A callable to be called when the parameter is modified

        :return ParameterCallbackHandle: A handle that should be saved by the user
            so that the event_callback can be later removed.
        """
        return self._callbacks.add_parameter_callback(
            parameter_name=parameter_name,
            node_name=self._resolve_path(node_name),
            callback=callback,
        )

    def remove_parameter_callback(
        self,
        handle: ParameterCallbackHandle,
    ) -> None:
        """
        Remove a ParameterCallbackHandle.

        Perform no callbacks on this parameter events in the future.

        :param handle: ParameterCallbackHandle of a callback to be removed
        """
        self._callbacks.remove_parameter_callback(handle)

    def add_parameter_event_callback(
        self,
        callback: Callable[[ParameterEvent], None],
    ) -> ParameterEventCallbackHandle:
        """
        Add new parameter callback.

        Callbacks are called in FILO manner.

        :param callback: A callable to be referenced on a ParameterEvent

        :return ParameterEventCallbackHandle: A handle that should be saved by the user
            so that the event_callback can be later removed.
        """
        return self._callbacks.add_parameter_event_callback(callback)

    def remove_parameter_event_callback(
        self,
        handle: ParameterEventCallbackHandle,
    ) -> None:
        """
        Remove a ParameterEventCallbackHandle.

        Perform no callbacks on parameter events in the future.

        :param handle: ParameterEventCallbackHandle of a callback to be removed
        """
        self._callbacks.remove_parameter_event_callback(handle)

    def _resolve_path(
        self,
        node_path: Optional[str] = None,
    ) -> str:
        """
        Get full name of a node.

        :param node_path: Name of a node with namespaces.
        """
        if not node_path:
            return self.node.get_fully_qualified_name()

        if node_path.startswith('/'):
            return node_path

        node_namespace = self.node.get_namespace().lstrip('/')
        resolved_path = '/'.join([node_namespace, node_path])
        return resolved_path if resolved_path.startswith('/') else f'/{resolved_path}'
