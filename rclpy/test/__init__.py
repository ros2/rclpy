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
import sys

assert 'rclpy' not in sys.modules, 'rclpy should not have been imported before running tests'

# this will make the extensions load from the build folder
import rclpy.impl  # noqa
import test_rclpy  # noqa


def _custom_import(name):
    # New in Python 3.8: on Windows we should call 'add_dll_directory()' for directories
    # containing DLLs we depend on.
    # https://docs.python.org/3/whatsnew/3.8.html#bpo-36085-whatsnew
    dll_dir_handles = []
    if os.name == 'nt' and hasattr(os, 'add_dll_directory'):
        path_env = os.environ['PATH'].split(';')
        for prefix_path in path_env:
            if os.path.exists(prefix_path):
                dll_dir_handles.append(os.add_dll_directory(prefix_path))
    import_module = importlib.import_module(name, package='test_rclpy')
    for handle in dll_dir_handles:
        handle.close()
    return import_module


rclpy.impl._import = _custom_import
