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

import importlib

import ament_index_python

rmw_implementations = sorted(ament_index_python.get_resources('rmw_implementation').keys())
_rmw_implementation = None


def select(rmw_implementation):
    global _rmw_implementation
    _rmw_implementation = rmw_implementation


def import_rmw_implementation():
    if _rmw_implementation is None:
        global _rmw_implementation
        _rmw_implementation = rmw_implementations[0]
    module_name = '._rclpy__{rmw_implementation}'.format(
        rmw_implementation=_rmw_implementation,
    )
    return importlib.import_module(module_name, package='rclpy')
