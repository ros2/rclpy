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

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support


def serialize_message(message) -> bytes:
    """
    Serialize a ROS message.

    :param message: The ROS message to serialize.
    :return: The serialized bytes.
    """
    message_type = type(message)
    # this line imports the typesupport for the message module if not already done
    check_for_type_support(message_type)
    return _rclpy.rclpy_serialize(message, message_type)


def deserialize_message(serialized_message: bytes, message_type):
    """
    Deserialize a ROS message.

    :param serialized_message: The ROS message to deserialize.
    :param message_type: The type of the serialized ROS message.
    :return: The deserialized ROS message.
    """
    # this line imports the typesupport for the message module if not already done
    check_for_type_support(message_type)
    return _rclpy.rclpy_deserialize(serialized_message, message_type)
