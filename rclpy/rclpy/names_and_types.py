# Copyright 2016 Open Source Robotics Foundation, Inc.
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


class TopicNamesAndTypes:
    """Class defining information gathered about advertised topics."""

    __slots__ = [
        '_topic_count',
        '_topic_names',
        '_type_names',
    ]

    def __init__(self, **kwargs):
        assert all(['_' + key in self.__slots__ for key in kwargs.keys()]), \
            "Invalid arguments passed to constructor: %r" % kwargs.keys()
        self.topic_count = kwargs.get('topic_count', int())
        self.topic_names = kwargs.get('topic_names', list())
        self.type_names = kwargs.get('type_names', list())

    def __repr__(self):
        s = 'number of topics = ' + repr(self._topic_count) + '\n'
        s += '[topic_name] : [ROS type name]\n'
        for i in range(self._topic_count):
            s += '{} : {}\n'.format(
                self._topic_names[i], self._type_names[i])
        return s

    @property
    def topic_count(self):
        """
        Get field topic_count.

        :returns: number of advertised topics
        :rtype: int
        """
        return self._topic_count

    @topic_count.setter
    def topic_count(self, value):
        assert isinstance(value, int)
        self._topic_count = value

    @property
    def topic_names(self):
        """
        Get field topic_names.

        :returns: name of advertised topics
        :rtype: list
        """
        return self._topic_names

    @topic_names.setter
    def topic_names(self, value):
        from collections import Sequence
        from collections import Set
        from collections import UserList
        from collections import UserString
        assert \
            ((isinstance(value, Sequence) or
              isinstance(value, Set) or
              isinstance(value, UserList)) and
             not isinstance(value, str) and
             not isinstance(value, UserString) and
             all([isinstance(v, str) for v in value]))
        self._topic_names = value

    @property
    def type_names(self):
        """
        Get field type_names.

        :returns: message type of advertised topics
        :rtype: list
        """
        return self._type_names

    @type_names.setter
    def type_names(self, value):
        from collections import Sequence
        from collections import Set
        from collections import UserList
        from collections import UserString
        assert \
            ((isinstance(value, Sequence) or
              isinstance(value, Set) or
              isinstance(value, UserList)) and
             not isinstance(value, str) and
             not isinstance(value, UserString) and
             all([isinstance(v, str) for v in value]))
        self._type_names = value
