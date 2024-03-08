# Copyright 2017 Open Source Robotics Foundation, Inc.
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

HIDDEN_TOPIC_PREFIX = '_'


def topic_or_service_is_hidden(name: str) -> bool:
    """
    Return True if a given topic or service name is hidden, otherwise False.

    A topic or service name is considered hidden if any of the name tokens
    begins with an underscore (``_``).
    See:

    http://design.ros2.org/articles/topic_and_service_names.html#hidden-topic-or-service-names

    :param name: topic or service name to test
    :returns: True if name is hidden, otherwise False
    """
    return any(token for token in name.split('/') if token.startswith(HIDDEN_TOPIC_PREFIX))
