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

from unittest.mock import Mock as _standard_mock


if hasattr(_standard_mock, 'assert_called_once'):
    # Python 3.6+
    Mock = _standard_mock
else:
    # Python 3.5

    class Mock(_standard_mock):
        """Add methods added in python 3.6 for python 3.5 compatibility."""

        def assert_called_once(self, *args, **kwargs):
            if len(args) or len(kwargs):
                return self.assert_called_once_with(*args, **kwargs)
            else:
                return 1 == self.call_count

        def assert_called(self, *args, **kwargs):
            if len(args) or len(kwargs):
                return self.assert_called_with(*args, **kwargs)
            else:
                return self.call_count > 0
