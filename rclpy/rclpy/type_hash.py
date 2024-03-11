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

    __slots__ = [
        '_version',
        '_value',
    ]

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %r' % kwargs.keys()

        self.version = kwargs.get('version', -1)
        self.value = kwargs.get('value', bytes(self._TYPE_HASH_SIZE))

    @property
    def version(self):
        """
        Get field 'version'.

        :returns: version attribute
        :rtype: int
        """
        return self._version

    @version.setter
    def version(self, value):
        assert isinstance(value, int)
        self._version = value

    @property
    def value(self):
        """
        Get field 'value'.

        :returns: value attribute
        :rtype: bytes
        """
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, bytes)
        self._value = value

    def __eq__(self, other):
        if not isinstance(other, TypeHash):
            return False
        return all(
            self.__getattribute__(slot) == other.__getattribute__(slot)
            for slot in self.__slots__)

    def __str__(self):
        if self._version <= 0 or len(self._value) != self._TYPE_HASH_SIZE:
            return 'INVALID'

        return f'RIHS{self._version:02}_{self._value.hex()}'
