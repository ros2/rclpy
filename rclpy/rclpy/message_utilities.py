# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from collections import OrderedDict
from typing import Any
from typing import Dict
import sys
import yaml


__yaml_representer_registered = False


class SetFieldError(Exception):

    def __init__(self, field_name, exception):
        super(SetFieldError, self).__init__()
        self.field_name = field_name
        self.exception = exception


def set_msg_fields(msg: Any, values: Dict[str, str]) -> None:
    """
    Set the fields of a ROS message.

    :param msg: The ROS message to populate.
    :param values: The values to set in the ROS message. The keys of the dictionary represent
        fields of the message.
    :raises SetFieldError: If there is an error setting a message field (e.g. the message does
        not have a field provided in the input dictionary).
    """
    for field_name, field_value in values.items():
        field_type = type(getattr(msg, field_name))
        try:
            value = field_type(field_value)
        except TypeError:
            value = field_type()
            try:
                set_msg_fields(value, field_value)
            except SetFieldError as e:
                raise SetFieldError(
                    '{field_name}.{e.field_name}'.format_map(locals()),
                    e.exception)
        except ValueError as e:
            raise SetFieldError(field_name, e)
        try:
            setattr(msg, field_name, value)
        except Exception as e:
            raise SetFieldError(field_name, e)


# Custom representer for getting clean YAML output that preserves the order in
# an OrderedDict.
# Inspired by:
# http://stackoverflow.com/a/16782282/7169408
def __represent_ordereddict(dumper, data):
    items = []
    for k, v in data.items():
        items.append((dumper.represent_data(k), dumper.represent_data(v)))
    return yaml.nodes.MappingNode(u'tag:yaml.org,2002:map', items)


def __convert_value(value, truncate_length=None):
    if isinstance(value, bytes):
        if truncate_length is not None and len(value) > truncate_length:
            value = ''.join([chr(c) for c in value[:truncate_length]]) + '...'
        else:
            value = ''.join([chr(c) for c in value])
    elif isinstance(value, str):
        if truncate_length is not None and len(value) > truncate_length:
            value = value[:truncate_length] + '...'
    elif isinstance(value, tuple) or isinstance(value, list):
        if truncate_length is not None and len(value) > truncate_length:
            # Truncate the sequence
            value = value[:truncate_length]
            # Truncate every item in the sequence
            value = type(value)([__convert_value(v, truncate_length) for v in value] + ['...'])
        else:
            # Truncate every item in the list
            value = type(value)([__convert_value(v, truncate_length) for v in value])
    elif isinstance(value, dict) or isinstance(value, OrderedDict):
        # Convert each key and value in the mapping
        new_value = {} if isinstance(value, dict) else OrderedDict()
        for k, v in value.items():
            # Don't truncate keys because that could result in key collisions and data loss
            new_value[__convert_value(k)] = __convert_value(v, truncate_length=truncate_length)
        value = new_value
    elif not any(isinstance(value, t) for t in (bool, float, int)):
        # Assuming value is a message since it is neither a collection nor a primitive type
        value = msg_to_ordereddict(value, truncate_length=truncate_length)
    return value


# Convert a msg to an OrderedDict. We do this instead of implementing a generic
# __dict__() method in the msg because we want to preserve order of fields from
# the .msg file(s).
def msg_to_ordereddict(msg: Any, truncate_length: int = None) -> OrderedDict:
    """
    Convert a ROS message to an OrderedDict.

    :param msg: The ROS message to convert.
    :param truncate_length: Truncate values for all message fields to this length.
        This does not truncate the list of fields (ie. the dictionary keys).
    :returns: An OrderedDict where the keys are the ROS message fields and the values are
        set to the values of the input message.
    """
    d = OrderedDict()
    # We rely on __slots__ retaining the order of the fields in the .msg file.
    for field_name in msg.__slots__:
        value = getattr(msg, field_name, None)
        value = __convert_value(value, truncate_length=truncate_length)
        # Remove leading underscore from field name
        d[field_name[1:]] = value
    return d


def msg_to_csv(msg: Any, truncate_length: int = None) -> str:
    """
    Convert a ROS message to string of comma-separated values.

    :param msg: The ROS message to convert.
    :param truncate_length: Truncate values for all message fields to this length.
        This does not truncate the list of message fields.
    :returns: A string of comma-separated values representing the input message.
    """
    def to_string(val):
        nonlocal truncate_length
        r = ''
        if any(isinstance(val, t) for t in [list, tuple]):
            for i, v in enumerate(val):
                if r:
                    r += ','
                if truncate_legnth is not None and i >= truncate_length:
                    r += '...'
                    break
                r += to_string(v)
        elif any(isinstance(val, t) for t in [bool, bytes, float, int, str]):
            if any(isinstance(val, t) for t in [bytes, str]):
                if truncate_length is not None and len(val) > truncate_length:
                    val = val[:truncate_length]
                    if isinstance(val, bytes):
                        val += b'...'
                    else:
                        val += '...'
            r = str(val)
        else:
            r = msg_to_csv(val, truncate_length)
        return r
    result = ''
    # We rely on __slots__ retaining the order of the fields in the .msg file.
    for field_name in msg.__slots__:
        value = getattr(msg, field_name, None)
        if result:
            result += ','
        result += to_string(value)
    return result


def msg_to_yaml(msg: Any, truncate_length: int = None) -> str:
    """
    Convert a ROS message to a YAML string.

    :param msg: The ROS message to convert.
    :param truncate_length: Truncate values for all message fields to this length.
        This does not truncate the list of message fields.
    :returns: A YAML string representation of the input ROS message.
    """
    global __yaml_representer_registered

    # Register our custom representer for YAML output
    if not __yaml_representer_registered:
        yaml.add_representer(OrderedDict, __represent_ordereddict)
        __yaml_representer_registered = True

    return yaml.dump(msg_to_ordereddict(msg, truncate_length=truncate_length), width=sys.maxsize)
