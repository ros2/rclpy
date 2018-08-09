# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from enum import Enum

from rcl_interfaces.msg import ParameterDescriptor, ParameterType, ParameterValue


class Parameter:

    class Type(Enum):
        NOT_SET = ParameterType.PARAMETER_NOT_SET
        BOOL = ParameterType.PARAMETER_BOOL
        INTEGER = ParameterType.PARAMETER_INTEGER
        DOUBLE = ParameterType.PARAMETER_DOUBLE
        STRING = ParameterType.PARAMETER_STRING
        BYTE_ARRAY = ParameterType.PARAMETER_BYTE_ARRAY
        BOOL_ARRAY = ParameterType.PARAMETER_BOOL_ARRAY
        INTEGER_ARRAY = ParameterType.PARAMETER_INTEGER_ARRAY
        DOUBLE_ARRAY = ParameterType.PARAMETER_DOUBLE_ARRAY
        STRING_ARRAY = ParameterType.PARAMETER_STRING_ARRAY

    @classmethod
    def from_rcl_interface_parameter(cls, rcl_param):
        value = None
        type_ = Parameter.Type(value=rcl_param.value.type)
        if Parameter.Type.BOOL == type_:
            value = rcl_param.value.bool_value
        elif Parameter.Type.INTEGER == type_:
            value = rcl_param.value.integer_value
        elif Parameter.Type.DOUBLE == type_:
            value = rcl_param.value.double_value
        elif Parameter.Type.STRING == type_:
            value = rcl_param.value.string_value
        elif Parameter.Type.BYTE_ARRAY == type_:
            value = rcl_param.value.byte_array_value
        elif Parameter.Type.BOOL_ARRAY == type_:
            value = rcl_param.value.bool_array_value
        elif Parameter.Type.INTEGER_ARRAY == type_:
            value = rcl_param.value.integer_array_value
        elif Parameter.Type.DOUBLE_ARRAY == type_:
            value = rcl_param.value.double_array_value
        elif Parameter.Type.STRING_ARRAY == type_:
            value = rcl_param.value.string_array_value
        return cls(rcl_param.name, type_, value)

    def __init__(self, name, type_, value):
        if not isinstance(type_, Parameter.Type):
            raise TypeError("type must be an instance of '{}'".format(repr(Parameter.Type)))

        self._type = type_
        self._name = name
        self._value = value

    @property
    def name(self):
        return self._name

    @property  # noqa: A003
    def type(self):
        return self._type

    @property
    def value(self):
        return self._value

    def get_descriptor(self):
        return ParameterDescriptor(name=self.name, type=self.type.value)

    def get_parameter_value(self):
        parameter_value = ParameterValue(type=self.type.value)
        if Parameter.Type.BOOL == self.type:
            parameter_value.bool_value = self.value
        elif Parameter.Type.INTEGER == self.type:
            parameter_value.integer_value = self.value
        elif Parameter.Type.DOUBLE == self.type:
            parameter_value.double_value = self.value
        elif Parameter.Type.STRING == self.type:
            parameter_value.string_value = self.value
        elif Parameter.Type.BYTE_ARRAY == self.type:
            parameter_value.byte_array_value = self.value
        elif Parameter.Type.BOOL_ARRAY == self.type:
            parameter_value.bool_array_value = self.value
        elif Parameter.Type.INTEGER_ARRAY == self.type:
            parameter_value.integer_array_value = self.value
        elif Parameter.Type.DOUBLE_ARRAY == self.type:
            parameter_value.double_array_value = self.value
        elif Parameter.Type.STRING_ARRAY == self.type:
            parameter_value.string_array_value = self.value
        return parameter_value
