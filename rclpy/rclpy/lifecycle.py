# Copyright 2021 Open Source Robotics Foundation, Inc.
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
from typing import Dict
from typing import Optional

import lifecycle_msgs.srv

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.service import Service
from rclpy.type_support import check_is_valid_msg_type
from rclpy.type_support import check_is_valid_srv_type


TransitionCallbackReturn = _rclpy.TransitionCallbackReturnType

class LifecycleMixin:

    # TODO(ivanpauno): We could make this a bit nicer by adding State, Transition, StateMachine abstractions
    # in this module.
    # Anyways those are not directly useful to the user and the implementation is not too bad without them.
    def __init__(
        self,
        *,
        enable_communication_interface: bool = True,
        callback_group: Optional[CallbackGroup] = None,
    ):
        if callback_group is None:
            callback_group = self.default_callback_group
        self._callbacks : Dict[int, Callable[[int], TransitionCallbackReturn]] = {}
        for srv_type in (
            lifecycle_msgs.srv.ChangeState,
            lifecycle_msgs.srv.GetState,
            lifecycle_msgs.srv.GetAvailableStates,
            lifecycle_msgs.srv.GetAvailableTransitions,
        ):
            # this doesn't only checks, but also imports some stuff we need later
            check_is_valid_srv_type(srv_type)

        with self.handle:
            self._state_machine = _rclpy.LifecycleStateMachine(self.handle, enable_communication_interface)
        self._service_change_state = Service(
            self._state_machine.service_change_state,
            lifecycle_msgs.srv.ChangeState,
            self._state_machine.service_change_state.name,
            self.__on_change_state,
            callback_group,
            QoSProfile(**self._state_machine.service_change_state.qos))
        self._service_get_state = Service(
            self._state_machine.service_get_state,
            lifecycle_msgs.srv.GetState,
            self._state_machine.service_get_state.name,
            self._on_get_state,
            callback_group,
            QoSProfile(**self._state_machine.service_get_state.qos))
        self._service_get_available_states = Service(
            self._state_machine.service_get_available_states,
            lifecycle_msgs.srv.GetAvailableStates,
            self._state_machine.service_get_available_states.name,
            self._on_get_available_states,
            callback_group,
            QoSProfile(**self._state_machine.service_get_available_states.qos))
        self._service_get_available_transitions = Service(
            self._state_machine.service_get_available_transitions,
            lifecycle_msgs.srv.GetAvailableTransitions,
            self._state_machine.service_get_available_transitions.name,
            self._on_get_available_transitions,
            callback_group,
            QoSProfile(**self._state_machine.service_get_available_transitions.qos))
        self._service_get_transition_graph = Service(
            self._state_machine.service_get_transition_graph,
            lifecycle_msgs.srv.GetAvailableTransitions,
            self._state_machine.service_get_transition_graph.name,
            self._on_get_transition_graph,
            callback_group,
            QoSProfile(**self._state_machine.service_get_transition_graph.qos))

        lifecycle_services = [
            self._service_change_state,
            self._service_get_state,
            self._service_get_available_states,
            self._service_get_available_transitions,
            self._service_get_transition_graph,
        ]
        for s in lifecycle_services:
            callback_group.add_entity(s)
        # TODO(ivanpauno): Modify attribute in Node to be "protected" instead of "private".
        # i.e. Node.__services -> Node._services
        # Maybe the same with similar attributes (__publishers, etc).
        # Maybe have some interface to add a service/etc instead (?).
        self._Node__services.extend(lifecycle_services)

    def register_callback(
        self, state_id: int, callback: Callable[[int], TransitionCallbackReturn]
    ) -> bool:
        self._callbacks[state_id] = callback
        # TODO(ivanpauno): Copying rclcpp style.
        # Maybe having a return value doesn't make sense (?).
        # Should we error/warn if overridding an existing callback?
        return True

    def __execute_callback(self, current_state: int, previous_state: int) -> TransitionCallbackReturn:
        cb = self._callbacks.get(current_state, None)
        if not cb:
            return TransitionCallbackReturn.SUCCESS
        try:
            return cb(previous_state)
        except Exception:
            # TODO(ivanpauno): log sth here
            return TransitionCallbackReturn.ERROR

    def __change_state(self, transition_id: int):
        self.__check_is_initialized()
        initial_state_id = self._state_machine.current_state[0]
        self._state_machine.trigger_transition_by_id(transition_id, True)

        cb_return_code = self.__execute_callback(self._state_machine.current_state[0], initial_state_id)
        self._state_machine.trigger_transition_by_label(cb_return_code.to_label(), True)


        if cb_return_code == TransitionCallbackReturn.ERROR:
            # TODO(ivanpauno): I don't understand what rclcpp is doing here ...
            # It's triggering the error transition twice (?)
            # https://github.com/ros2/rclcpp/blob/b2b676d3172ada509e58fa552a676a446809d83c/rclcpp_lifecycle/src/lifecycle_node_interface_impl.hpp#L428-L443
            pass
        return cb_return_code

    def __check_is_initialized(self):
        # TODO(ivanpauno): This sanity check is probably not needed, just doing the same checks as
        # rclcpp for the moment.
        if not self._state_machine.initialized:
            raise RuntimeError(
                'Internal error: got service request while lifecycle state machine '
                'is not initialized.')

    def __on_change_state(
        self,
        req: lifecycle_msgs.srv.ChangeState.Request,
        resp: lifecycle_msgs.srv.ChangeState.Response
    ):
        self.__check_is_initialized()
        transition_id = req.transition.id;
        if req.transition.label:
            try:
                transition_id = self._state_machine.get_transition_by_label(req.transition.label)
            except _rclpy.RCLError:
                resp.success = False
                return resp
        cb_return_code = self.__change_state(transition_id);
        resp.success = cb_return_code == TransitionCallbackReturn.SUCCESS
        return resp

    def _on_get_state(
        self,
        req: lifecycle_msgs.srv.GetState.Request,
        resp: lifecycle_msgs.srv.GetState.Response
    ):
        self.__check_is_initialized()
        resp.current_state.id, resp.current_state.label = self._state_machine.current_state
        return resp

    def _on_get_available_states(
        self,
        req: lifecycle_msgs.srv.GetAvailableStates.Request,
        resp: lifecycle_msgs.srv.GetAvailableStates.Response
    ):
        self.__check_is_initialized()
        for id, label in self._state_machine.available_states:
            resp.available_states.append(lifecycle_msgs.msg.State(id=id, label=label))
        return resp

    def _on_get_available_transitions(
        self,
        req: lifecycle_msgs.srv.GetAvailableTransitions.Request,
        resp: lifecycle_msgs.srv.GetAvailableTransitions.Response
    ):
        self.__check_is_initialized()
        for id, label, start_id, start_label, goal_id, goal_label in self._state_machine.available_transitions:
            item = lifecycle_msgs.msg.TransitionDescription()
            item.transition.id = id
            item.transition.label = label
            item.start_state.id = start_id
            item.start_state.label = start_label
            item.goal_state.id = goal_id
            item.goal_state.label = goal_label
            resp.available_transitions.append(item)
        return resp

    def _on_get_transition_graph(
        self,
        req: lifecycle_msgs.srv.GetAvailableTransitions.Request,
        resp: lifecycle_msgs.srv.GetAvailableTransitions.Response
    ):
        self.__check_is_initialized()
        for id, label, start_id, start_label, goal_id, goal_label in self._state_machine.transition_graph:
            item = lifecycle_msgs.msg.TransitionDescription()
            item.transition.id = id
            item.transition.label = label
            item.start_state.id = start_id
            item.start_state.label = start_label
            item.goal_state.id = goal_id
            item.goal_state.label = goal_label
            resp.available_transitions.append(item)
        return resp

    def on_configure(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS


class LifecycleNode(LifecycleMixin, Node):
    def __init__(self, node_name, *, enable_communication_interface: bool = True, **kwargs):
        Node.__init__(self, node_name, **kwargs)
        LifecycleMixin.__init__(self,
            enable_communication_interface=enable_communication_interface)
