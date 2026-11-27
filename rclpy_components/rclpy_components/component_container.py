# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .component_manager import ComponentManager


def main():
    rclpy.init()

    executor = SingleThreadedExecutor()
    component_manager = ComponentManager(executor, 'PyComponentManager')

    executor.add_node(component_manager)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print('KeyboardInterrupt received, exit')

    component_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
