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
from typing import List
from typing import Optional
from typing import Text
from typing import Type
from typing import TYPE_CHECKING
from typing import Union

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.duration import Duration
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSPolicyKind
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.subscription import Subscription

if TYPE_CHECKING:
    from typing import TypeAlias
    from rclpy.node import Node


class InvalidQosOverridesError(Exception):
    pass


# Return type of qos validation callbacks
QosCallbackResult: 'TypeAlias' = SetParametersResult
# Qos callback type annotation
QosCallbackType = Callable[[QoSProfile], QosCallbackResult]


class QoSOverridingOptions:
    """Options to customize QoS parameter overrides."""

    def __init__(
        self,
        policy_kinds: Iterable[QoSPolicyKind],
        *,
        callback: Optional[QosCallbackType] = None,
        entity_id: Optional[Text] = None
    ):
        """
        Construct a QoSOverridingOptions object.

        :param policy_kinds: QoS kinds that will have a declared parameter.
        :param callback: Callback that will be used to validate the QoS profile
            after the paramter overrides get applied.
        :param entity_id: Optional identifier, to disambiguate in the case that different QoS
            policies for the same topic are desired.
        """
        self._policy_kinds = policy_kinds
        self._callback = callback
        self._entity_id = entity_id

    @property
    def policy_kinds(self) -> Iterable[QoSPolicyKind]:
        """Get QoS policy kinds that will have a parameter override."""
        return self._policy_kinds

    @property
    def callback(self) -> Optional[QosCallbackType]:
        """Get the validation callback."""
        return self._callback

    @property
    def entity_id(self) -> Optional[Text]:
        """Get the optional entity ID."""
        return self._entity_id

    @classmethod
    def with_default_policies(
        cls, *,
        callback: Optional[QosCallbackType] = None,
        entity_id: Optional[Text] = None
    ) -> 'QoSOverridingOptions':
        return cls(
            policy_kinds=(QoSPolicyKind.HISTORY, QoSPolicyKind.DEPTH, QoSPolicyKind.RELIABILITY),
            callback=callback,
            entity_id=entity_id,
        )


def _declare_qos_parameters(
    entity_type: Union[Type[Publisher], Type[Subscription]],
    node: 'Node',
    topic_name: Text,
    qos: QoSProfile,
    options: QoSOverridingOptions
) -> None:
    """
    Declare QoS parameters for a Publisher or a Subscription.

    :param entity_type: Either `rclpy.node.Publisher` or `rclpy.node.Subscription`.
    :param node: Node used to declare the parameters.
    :param topic_name: Topic name of the entity being created.
    :param qos: Default QoS settings of the entity being created, that will be overridden
        with the user provided QoS parameter overrides.
    :param options: Options that indicates which parameters are going to be declared.
    """
    if not issubclass(entity_type, (Publisher, Subscription)):
        raise TypeError('Argument `entity_type` should be a subclass of Publisher or Subscription')
    entity_type_str = 'publisher' if issubclass(entity_type, Publisher) else 'subscription'
    id_suffix = '' if options.entity_id is None else f'_{options.entity_id}'
    name = f'qos_overrides.{topic_name}.{entity_type_str}{id_suffix}.' '{}'
    description = '{}' f' for {entity_type_str} `{topic_name}` with id `{options.entity_id}`'
    allowed_policies = _get_allowed_policies(entity_type)
    for policy in options.policy_kinds:
        if policy not in allowed_policies:
            continue
        policy_name = policy.name.lower()
        descriptor = ParameterDescriptor()
        descriptor.description = description.format(policy_name)
        descriptor.read_only = True
        try:
            param = node.declare_parameter(
                name.format(policy_name),
                _get_qos_policy_parameter(qos, policy),
                descriptor)
        except ParameterAlreadyDeclaredException:
            param = node.get_parameter(name.format(policy_name))
        _override_qos_policy_with_param(qos, policy, param)
    if options.callback is not None:
        result = options.callback(qos)
        if not result.successful:
            raise InvalidQosOverridesError(
                f"{description.format('Provided QoS overrides')}, are not valid: {result.reason}")


def _get_allowed_policies(entity_type: Union[Type[Publisher],
                                             Type[Subscription]]) -> List[QoSPolicyKind]:
    allowed_policies = list(QoSPolicyKind.__members__.values())
    if issubclass(entity_type, Subscription):
        allowed_policies.remove(QoSPolicyKind.LIFESPAN)
    return allowed_policies


QoSProfileAttributes = Union[QoSHistoryPolicy, int, QoSReliabilityPolicy, QoSDurabilityPolicy,
                             Duration, QoSLivelinessPolicy, bool]


def _get_qos_policy_parameter(qos: QoSProfile, policy: QoSPolicyKind) -> Union[str, int, bool]:
    value: QoSProfileAttributes = getattr(qos, policy.name.lower())
    if isinstance(value, (QoSHistoryPolicy, QoSReliabilityPolicy,
                  QoSDurabilityPolicy, QoSLivelinessPolicy)):
        return_value: Union[str, int, bool] = value.name.lower()
        if return_value == 'unknown':
            raise ValueError('User provided QoS profile is invalid')
    elif isinstance(value, Duration):
        return_value = value.nanoseconds
    else:
        return_value = value
    return return_value


def _override_qos_policy_with_param(qos: QoSProfile,
                                    policy: QoSPolicyKind,
                                    param: Parameter) -> None:
    value = param.value
    policy_name = policy.name.lower()
    if policy in (
        QoSPolicyKind.LIVELINESS, QoSPolicyKind.RELIABILITY,
        QoSPolicyKind.HISTORY, QoSPolicyKind.DURABILITY
    ):
        def capitalize_first_letter(x: str) -> str:
            return x[0].upper() + x[1:]
        # e.g. `policy=QosPolicyKind.LIVELINESS` -> `policy_enum_class=rclpy.qos.LivelinessPolicy`
        policy_enum_class = getattr(
            rclpy.qos, f'{capitalize_first_letter(policy_name)}Policy')
        try:
            value = policy_enum_class[value.upper()]
        except KeyError:
            raise RuntimeError(
                f'Unexpected QoS override for policy `{policy.name.lower()}`: `{value}`')
    if policy in (
        QoSPolicyKind.LIFESPAN, QoSPolicyKind.DEADLINE, QoSPolicyKind.LIVELINESS_LEASE_DURATION
    ):
        value = Duration(nanoseconds=value)
    setattr(qos, policy.name.lower(), value)
