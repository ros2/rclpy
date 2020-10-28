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

from typing import Callable
from typing import Iterable
from typing import Optional
from typing import Text
from typing import Type
from typing import Union

from rcl_interfaces.msg import ParameterDescriptor

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.qos import QoSPolicyKind
from rclpy.subscription import Subscription


class InvalidQosOverridesError(Exception):
    pass


class QoSOverridingOptions:
    """
    Options to customize qos parameter overrides.
    """

    def __init__(
        self,
        *,
        policy_kinds: Iterable[QoSPolicyKind],
        callback: Optional[Callable[[QoSProfile], bool]] = None,
        id: Optional[Text] = None
    ):
        """
        Construct a QoSOverridingOptions object.

        :param policy_kinds: Qos kinds that will have a declared parameter.
        :param callback: Callback that will be used to validate the qos profile
            after the paramter overrides get applied.
        :param id: Optional identifier, to disambiguate in the case that different qos
            policies for the same topic are desired.
        """
        self._policy_kinds = policy_kinds
        self._callback = callback
        self._id = id

    @property
    def policy_kinds(self) -> Iterable[QoSPolicyKind]:
        """Get qos policy kinds that will have a parameter override."""
        return self._policy_kinds

    @property
    def callback(self) -> Optional[Callable[[QoSProfile], bool]]:
        """Get the validation callback."""
        return self._callback

    @property
    def id(self) -> Optional[Text]:
        """Get the optional entity id."""
        return self._id


def _declare_qos_parameteres(
    entity_type: Union[Type[Publisher], Type[Subscription]],
    node: Node,
    topic_name: Text,
    qos: QoSProfile,
    options: QoSOverridingOptions) -> QoSProfile:
    """
    Internal.
    Declares qos parameters for a Publisher or a Subscription.

    :param entity_type: Either `rclpy.node.Publisher` or `rclpy.node.Subscription`.
    :param node: Node used to declare the parameters.
    :param topic_name: Topic name of the entity being created.
    :param qos: Default qos settings of the entity being created, that will be overriden
        with the user provided qos parameter overrides.
    :param options: Options that indicates which parameters are going to be declared.
    """
    if not issubclass(entity_type, (Publisher, Subscription)):
        raise TypeError("Argument `entity_type` should be a subclass of Publisher or Subscription")
    entity_type_str = 'publisher' if issubclass(entity_type, Publisher) else Subscription
    id_suffix = '' if options.id is None else f'_{options.id}'
    name = f'qos_overrides.{topic_name}.{entity_type_str}{id_suffix}.' '{}'
    description = '{}' f' for {entity_type_str} `{topic_name}` with id `{id}`'
    allowed_policies = _get_allowed_policies(entity_type)
    for policy in options.policy_kinds:
        if policy not in allowed_policies:
            continue
        policy_name = policy.name.lower()
        descriptor = ParameterDescriptor()
        descriptor.description = description.format(policy_name)
        descriptor.read_only = True
        param = node.declare_parameter(
            name.format(policy_name),
            _get_qos_policy_parameter(qos, policy),
            descriptor)
        _override_qos_policy_with_param(qos, policy, param)
    if options.callback is not None and not options.callback(qos):
        raise InvalidQosOverridesError(
            description.format('Provided qos overrides') + ', are not valid')


def _get_allowed_policies(entity_type: Union[Type[Publisher], Type[Subscription]]):
    allowed_policies = list(QoSPolicyKind.__members__.values())
    if issubclass(entity_type, Subscription):
        allowed_policies.remove(QoSPolicyKind.LIFESPAN)
    return allowed_policies


def _get_qos_policy_parameter(qos: QoSProfile, policy: QoSPolicyKind) -> Parameter:
    value = getattr(qos, policy.name.lower())
    if policy in (
        QoSPolicyKind.LIVELINESS, QoSPolicyKind.RELIABILITY,
        QoSPolicyKind.HISTORY, QoSPolicyKind.DURABILITY
    ):
        value = value.name.lower()
        if value == 'unknown':
            raise ValueError('User provided qos profile is invalid')
    if policy in (
        QoSPolicyKind.LIFESPAN, QoSPolicyKind.DEADLINE, QoSPolicyKind.LIVELINESS_LEASE_DURATION
    ):
        value = value.nanoseconds()
    return value


def _override_qos_policy_with_param(qos: QoSProfile, policy: QoSPolicyKind, param: Parameter):
    # policy_enum_map = {
    #     QoSPolicyKind.LIVELINESS: rclpy.qos.LivelinessPolicy,
    #     QoSPolicyKind.RELIABILITY: rclpy.qos.LivelinessPolicy,
    #     QoSPolicyKind.LIVELINESS: rclpy.qos.LivelinessPolicy,
    #     QoSPolicyKind.LIVELINESS: rclpy.qos.LivelinessPolicy,
    # }
    value = param.value
    policy_name = policy.name.lower()
    if policy in (
        QoSPolicyKind.LIVELINESS, QoSPolicyKind.RELIABILITY,
        QoSPolicyKind.HISTORY, QoSPolicyKind.DURABILITY
    ):
        def capitalize_first_letter(x):
            return x[0].upper() + x[1:]
        # e.g. `policy=QosPolicyKind.LIVELINESS` -> `policy_enum_class=rclpy.qos.LivelinessPolicy`
        policy_enum_class = getattr(
            rclpy.qos, f"{capitalize_first_letter(policy_name)}Policy")
        try:
            value = policy_enum_class[value.upper()]
        except KeyError:
            raise RuntimeError(
                f'Unexpected qos override for policy `{policy.name.lower()}`: `{value}`')
    if policy in (
        QoSPolicyKind.LIFESPAN, QoSPolicyKind.DEADLINE, QoSPolicyKind.LIVELINESS_LEASE_DURATION
    ):
        value = Duration(nanoseconds=value)
    setattr(qos, policy.name.lower(), value)
