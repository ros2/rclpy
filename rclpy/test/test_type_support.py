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

import pytest

from rclpy import type_support
from rclpy.exceptions import NoTypeSupportImportedException

from test_msgs.msg import Strings
from test_msgs.srv import Empty


class MockTypeMetaclass(type):
    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        pass


class MockType(metaclass=MockTypeMetaclass):
    pass


def test_check_for_type_support():
    type_support.check_for_type_support(Strings)
    type_support.check_for_type_support(Empty)
    with pytest.raises(AttributeError):
        type_support.check_for_type_support(object())
    with pytest.raises(NoTypeSupportImportedException):
        type_support.check_for_type_support(MockType)


def test_check_valid_msg_type():
    type_support.check_is_valid_msg_type(Strings)
    with pytest.raises(RuntimeError):
        type_support.check_is_valid_msg_type(Empty)


def test_check_valid_srv_type():
    type_support.check_is_valid_srv_type(Empty)
    with pytest.raises(RuntimeError):
        type_support.check_is_valid_srv_type(Strings)
