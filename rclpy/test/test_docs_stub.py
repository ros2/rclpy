# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Keep the C extension stand-in in docs/source/conf.py in sync with the real extension."""

import pathlib
import runpy
import subprocess
import sys
import textwrap
from typing import Any, Dict
import unittest

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

CONF_PY = pathlib.Path(__file__).parents[1] / 'docs' / 'source' / 'conf.py'

PREDEFINED_QOS_PROFILES = (
    'qos_profile_sensor_data',
    'qos_profile_default',
    'qos_profile_system_default',
    'qos_profile_services_default',
    'qos_profile_unknown',
    'qos_profile_parameters',
    'qos_profile_parameter_events',
    'qos_profile_best_available',
    'qos_profile_rosout_default',
)
ACTION_QOS_PROFILE = 'rcl_action_qos_profile_status_default'


def _shape(profile: Dict[str, Any]) -> Dict[str, type]:
    return {key: type(value) for key, value in profile.items()}


class TestDocsStub(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls.stub = runpy.run_path(str(CONF_PY))['_RclpyStub']()

    def test_constants(self) -> None:
        for name in (
            'RMW_DURATION_INFINITE',
            'RMW_QOS_DEADLINE_BEST_AVAILABLE',
            'RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE',
        ):
            with self.subTest(name=name):
                self.assertEqual(getattr(_rclpy, name), getattr(self.stub, name))

    def test_predefined_qos_profiles(self) -> None:
        for name in PREDEFINED_QOS_PROFILES:
            with self.subTest(name=name):
                expected = _rclpy.rmw_qos_profile_t.predefined(name).to_dict()
                actual = self.stub.rmw_qos_profile_t.predefined(name).to_dict()
                self.assertEqual(_shape(expected), _shape(actual))

    def test_action_qos_profile(self) -> None:
        expected = _rclpy.rclpy_action_get_rmw_qos_profile(ACTION_QOS_PROFILE)
        actual = self.stub.rclpy_action_get_rmw_qos_profile(ACTION_QOS_PROFILE)
        self.assertEqual(_shape(expected), _shape(actual))

    def test_import_all_modules(self) -> None:
        # Import every rclpy module with the stand-in in place of the extension, as the docs do
        code = textwrap.dedent("""
            import importlib
            import pkgutil
            import runpy
            import sys
            import types

            stub = runpy.run_path(sys.argv[1])['_RclpyStub']()
            fake = types.ModuleType('rpyutils')
            fake.import_c_library = lambda name, package=None: stub
            fake.add_dll_directories_from_env = lambda *args, **kwargs: None
            sys.modules['rpyutils'] = fake

            import rclpy
            for info in pkgutil.walk_packages(rclpy.__path__, 'rclpy.'):
                if not info.name.endswith('_pybind11'):
                    importlib.import_module(info.name)
        """)
        result = subprocess.run(
            [sys.executable, '-c', code, str(CONF_PY)], capture_output=True, text=True)
        self.assertEqual(0, result.returncode, result.stderr)


if __name__ == '__main__':
    unittest.main()
