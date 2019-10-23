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

import array
from enum import Enum

from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue

PARAMETER_SEPARATOR_STRING = '.'


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
        def from_parameter_value(cls, parameter_value):
            """
            Get a Parameter.Type from a given variable.

            :return: A Parameter.Type corresponding to the instance type of the given value.
            :raises: TypeError if the conversion to a type was not possible.
            """
            if parameter_value is None:
                return Parameter.Type.NOT_SET
            elif isinstance(parameter_value, bool):
                return Parameter.Type.BOOL
            elif isinstance(parameter_value, int):
                return Parameter.Type.INTEGER
            elif isinstance(parameter_value, float):
                return Parameter.Type.DOUBLE
            elif isinstance(parameter_value, str):
                return Parameter.Type.STRING
            elif isinstance(parameter_value, (list, tuple, array.array)):
                if all(isinstance(v, bytes) for v in parameter_value):
                    return Parameter.Type.BYTE_ARRAY
                elif all(isinstance(v, bool) for v in parameter_value):
                    return Parameter.Type.BOOL_ARRAY
                elif all(isinstance(v, int) for v in parameter_value):
                    return Parameter.Type.INTEGER_ARRAY
                elif all(isinstance(v, float) for v in parameter_value):
                    return Parameter.Type.DOUBLE_ARRAY
                elif all(isinstance(v, str) for v in parameter_value):
                    return Parameter.Type.STRING_ARRAY
                else:
                    raise TypeError('The given value is not a list of one of the allowed types.')
            else:
                raise TypeError('The given value is not one of the allowed types.')

        def check(self, parameter_value):
            if Parameter.Type.NOT_SET == self:
                return parameter_value is None
            if Parameter.Type.BOOL == self:
                return isinstance(parameter_value, bool)
            if Parameter.Type.INTEGER == self:
                return isinstance(parameter_value, int)
            if Parameter.Type.DOUBLE == self:
                return isinstance(parameter_value, float)
            if Parameter.Type.STRING == self:
                return isinstance(parameter_value, str)
            if Parameter.Type.BYTE_ARRAY == self:
                return isinstance(parameter_value, (list, tuple)) and \
                    all(isinstance(v, bytes) and len(v) == 1 for v in parameter_value)
            if Parameter.Type.BOOL_ARRAY == self:
                return isinstance(parameter_value, (list, tuple)) and \
                    all(isinstance(v, bool) for v in parameter_value)
            if Parameter.Type.INTEGER_ARRAY == self:
                return isinstance(parameter_value, (list, tuple, array.array)) and \
                    all(isinstance(v, int) for v in parameter_value)
            if Parameter.Type.DOUBLE_ARRAY == self:
                return isinstance(parameter_value, (list, tuple, array.array)) and \
                    all(isinstance(v, float) for v in parameter_value)
            if Parameter.Type.STRING_ARRAY == self:
                return isinstance(parameter_value, (list, tuple)) and \
                    all(isinstance(v, str) for v in parameter_value)
            return False

    @classmethod
    def from_parameter_msg(cls, param_msg):
        value = None
        type_ = Parameter.Type(value=param_msg.value.type)
        if Parameter.Type.BOOL == type_:
            value = param_msg.value.bool_value
        elif Parameter.Type.INTEGER == type_:
            value = param_msg.value.integer_value
        elif Parameter.Type.DOUBLE == type_:
            value = param_msg.value.double_value
        elif Parameter.Type.STRING == type_:
            value = param_msg.value.string_value
        elif Parameter.Type.BYTE_ARRAY == type_:
            value = param_msg.value.byte_array_value
        elif Parameter.Type.BOOL_ARRAY == type_:
            value = param_msg.value.bool_array_value
        elif Parameter.Type.INTEGER_ARRAY == type_:
            value = param_msg.value.integer_array_value
        elif Parameter.Type.DOUBLE_ARRAY == type_:
            value = param_msg.value.double_array_value
        elif Parameter.Type.STRING_ARRAY == type_:
            value = param_msg.value.string_array_value
        return cls(param_msg.name, type_, value)

    def __init__(self, name, type_=None, value=None):
        if type_ is None:
            # This will raise a TypeError if it is not possible to get a type from the value.
            type_ = Parameter.Type.from_parameter_value(value)

        if not isinstance(type_, Parameter.Type):
            raise TypeError("type must be an instance of '{}'".format(repr(Parameter.Type)))

        if not type_.check(value):
            raise ValueError("Type '{}' and value '{}' do not agree".format(type_, value))

        self._type_ = type_
        self._name = name
        self._value = value

    @property
    def name(self):
        return self._name

    @property
    def type_(self):
        return self._type_

    @property
    def value(self):
        return self._value

    def get_parameter_value(self):
        parameter_value = ParameterValue(type=self.type_.value)
        if Parameter.Type.BOOL == self.type_:
            parameter_value.bool_value = self.value
        elif Parameter.Type.INTEGER == self.type_:
            parameter_value.integer_value = self.value
        elif Parameter.Type.DOUBLE == self.type_:
            parameter_value.double_value = self.value
        elif Parameter.Type.STRING == self.type_:
            parameter_value.string_value = self.value
        elif Parameter.Type.BYTE_ARRAY == self.type_:
            parameter_value.byte_array_value = self.value
        elif Parameter.Type.BOOL_ARRAY == self.type_:
            parameter_value.bool_array_value = self.value
        elif Parameter.Type.INTEGER_ARRAY == self.type_:
            parameter_value.integer_array_value = self.value
        elif Parameter.Type.DOUBLE_ARRAY == self.type_:
            parameter_value.double_array_value = self.value
        elif Parameter.Type.STRING_ARRAY == self.type_:
            parameter_value.string_array_value = self.value
        return parameter_value

    def to_parameter_msg(self):
        return ParameterMsg(name=self.name, value=self.get_parameter_value())
