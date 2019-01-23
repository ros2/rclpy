# Copyright 2019 Open Source Robotics Foundation, Inc.
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
import uuid

from rclpy.action_client import ClientGoalHandle

from test_msgs.action import Fibonacci

from unique_identifier_msgs.msg import UUID


class TestClientGoalHandle(unittest.TestCase):

    def test_constructor(self):
        ClientGoalHandle(UUID(), Fibonacci.GoalRequestService.Response())
        with self.assertRaises(TypeError):
            ClientGoalHandle('not a uuid', Fibonacci.GoalRequestService.Response())

    def test_accepted(self):
        accepted_response = Fibonacci.GoalRequestService.Response()
        accepted_response.accepted = True
        accepted_goal_handle = ClientGoalHandle(UUID(), accepted_response)
        self.assertTrue(accepted_goal_handle.accepted)

        rejected_response = Fibonacci.GoalRequestService.Response()
        rejected_response.accepted = False
        rejected_goal_handle = ClientGoalHandle(UUID(), rejected_response)
        self.assertFalse(rejected_goal_handle.accepted)

    def test_eq(self):
        uuid0 = uuid.uuid4()
        uuid1 = uuid.uuid4()
        empty_response = Fibonacci.GoalRequestService.Response()

        self.assertEqual(ClientGoalHandle(UUID(uuid=uuid0.bytes), empty_response),
                         ClientGoalHandle(UUID(uuid=uuid0.bytes), empty_response))
        self.assertNotEqual(ClientGoalHandle(UUID(uuid=uuid0.bytes), empty_response),
                            ClientGoalHandle(UUID(uuid=uuid1.bytes), empty_response))


if __name__ == '__main__':
    unittest.main()
