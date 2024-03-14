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


class TypeHash:
    """Type hash."""

    _TYPE_HASH_SIZE = 32

    # ros2cli needs __slots__ to avoid API break from https://github.com/ros2/rclpy/pull/1243.
    # __slots__ is also used improve performance of Python objects.
    __slots__ = [
        '_version',
        '_value',
    ]

    def __init__(self, version: int = -1, value: bytes = bytes(_TYPE_HASH_SIZE)):
        self.version = version
        self.value = value

    @property
    def version(self) -> int:
        """
        Get field 'version'.

        :returns: version attribute
        """
        return self._version

    @version.setter
    def version(self, value: int) -> None:
        assert isinstance(value, int)
        self._version = value

    @property
    def value(self) -> bytes:
        """
        Get field 'value'.

        :returns: value attribute
        """
        return self._value

    @value.setter
    def value(self, value: bytes) -> None:
        assert isinstance(value, bytes)
        self._value = value

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, TypeHash):
            return False
        return all(
            self.__getattribute__(slot) == other.__getattribute__(slot)
            for slot in self.__slots__)

    def __str__(self) -> str:
        if self._version <= 0 or len(self._value) != self._TYPE_HASH_SIZE:
            return 'INVALID'

        return f'RIHS{self._version:02}_{self._value.hex()}'
