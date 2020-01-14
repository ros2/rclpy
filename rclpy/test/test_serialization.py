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

import pytest

from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

from test_msgs.message_fixtures import get_test_msg
from test_msgs.msg import Arrays
from test_msgs.msg import BasicTypes
from test_msgs.msg import BoundedSequences
from test_msgs.msg import Builtins
from test_msgs.msg import Constants
from test_msgs.msg import Defaults
from test_msgs.msg import Empty
from test_msgs.msg import MultiNested
from test_msgs.msg import Nested
from test_msgs.msg import Strings
from test_msgs.msg import UnboundedSequences
from test_msgs.msg import WStrings

test_msgs = [
  (get_test_msg('Arrays'), Arrays),
  (get_test_msg('BasicTypes'), BasicTypes),
  (get_test_msg('BoundedSequences'), BoundedSequences),
  (get_test_msg('Builtins'), Builtins),
  (get_test_msg('Constants'), Constants),
  (get_test_msg('Defaults'), Defaults),
  (get_test_msg('Empty'), Empty),
  (get_test_msg('MultiNested'), MultiNested),
  (get_test_msg('Nested'), Nested),
  (get_test_msg('Strings'), Strings),
  (get_test_msg('UnboundedSequences'), UnboundedSequences),
  (get_test_msg('WStrings'), WStrings),
]


@pytest.mark.parametrize('msgs,msg_type', test_msgs)
def test_serialize_deserialize(msgs, msg_type):
    """Test message serialization/deserialization."""
    for msg in msgs:
        msg_serialized = serialize_message(msg)
        msg_deserialized = deserialize_message(msg_serialized, msg_type)
        assert msg == msg_deserialized


def test_set_float32():
    """Test message serialization/deserialization of float32 type."""
    # During (de)serialization we convert to a C float before converting to a PyObject.
    # This can result in a loss of precision
    msg = BasicTypes()
    msg.float32_value = 1.125  # can be represented without rounding
    msg_serialized = serialize_message(msg)
    msg_deserialized = deserialize_message(msg_serialized, BasicTypes)
    assert msg.float32_value == msg_deserialized.float32_value

    msg = BasicTypes()
    msg.float32_value = 3.14  # can NOT be represented without rounding
    msg_serialized = serialize_message(msg)
    msg_deserialized = deserialize_message(msg_serialized, BasicTypes)
    assert msg.float32_value == round(msg_deserialized.float32_value, 2)
