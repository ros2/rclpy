# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

import importlib
import os
from pathlib import Path

from rpyutils import add_dll_directories_from_env


def _import(name):
    try:
        # Since Python 3.8, on Windows we should ensure DLL directories are
        # explicitly added to the search path.
        # See https://docs.python.org/3/whatsnew/3.8.html#bpo-36085-whatsnew
        with add_dll_directories_from_env('PATH'):
            return importlib.import_module(name, package='rclpy')
    except ImportError as e:
        if e.path is None:
            import sysconfig
            expected_path = Path(__file__).parents[1] / (
                name[1:] + sysconfig.get_config_var('EXT_SUFFIX'))
            assert not expected_path.is_file()
            e.msg += \
                f"\nThe C extension '{expected_path}' isn't present on the " \
                "system. Please refer to 'https://index.ros.org/doc/ros2/" \
                'Troubleshooting/Installation-Troubleshooting/#import-' \
                "failing-without-library-present-on-the-system' for " \
                'possible solutions'
        if e.path is not None and os.path.isfile(e.path):
            e.msg += \
                f"\nThe C extension '{e.path}' failed to be imported while " \
                "being present on the system. Please refer to 'https://" \
                'index.ros.org/doc/ros2/Troubleshooting/Installation-' \
                'Troubleshooting/#import-failing-even-with-library-present-' \
                "on-the-system' for possible solutions"
        raise
