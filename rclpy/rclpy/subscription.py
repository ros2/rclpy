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


class Subscription:

    def __init__(
            self, subscription_handle, subscription_pointer,
            msg_type, topic, callback, callback_group, qos_profile, node_handle):
        self.node_handle = node_handle
        self.subscription_handle = subscription_handle
        self.subscription_pointer = subscription_pointer
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False
        self.qos_profile = qos_profile
