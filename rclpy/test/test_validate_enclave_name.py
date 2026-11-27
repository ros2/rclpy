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

import unittest

from rclpy.exceptions import InvalidEnclaveNameException
from rclpy.validate_enclave_name import validate_enclave_name


class TestValidateEnclaveName(unittest.TestCase):

    def test_validate_enclave_name(self) -> None:
        tests = [
            '/',
            '/my_enclave',
            '/my/nested/enclave',
        ]
        for enclave_name in tests:
            # Will raise if invalid
            self.assertTrue(validate_enclave_name(enclave_name))

    def test_validate_enclave_name_failures(self) -> None:
        # enclave name must not be empty
        with self.assertRaisesRegex(InvalidEnclaveNameException, 'empty'):
            validate_enclave_name('')
        # enclave name must start with /
        with self.assertRaisesRegex(InvalidEnclaveNameException, 'must be absolute'):
            validate_enclave_name('my_enclave')
        # enclave name must not end with /
        with self.assertRaisesRegex(InvalidEnclaveNameException, 'must not end with'):
            validate_enclave_name('/my_enclave/')
        # enclave name must not contain repeated /
        with self.assertRaisesRegex(InvalidEnclaveNameException, 'repeated'):
            validate_enclave_name('/my//enclave')
        # enclave name must only contain allowed characters
        with self.assertRaisesRegex(InvalidEnclaveNameException, 'characters'):
            validate_enclave_name('/my_enclave!')
        # enclave name tokens must not start with a number
        with self.assertRaisesRegex(InvalidEnclaveNameException, 'number'):
            validate_enclave_name('/1enclave')


if __name__ == '__main__':
    unittest.main()
