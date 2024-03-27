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

import unittest
import rclpy
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from rclpy.executors import Executor, SingleThreadedExecutor, MultiThreadedExecutor
from rclpy_components.component_manager import ComponentManager

TEST_COMPOSITION = 'test_composition'
TEST_COMPOSITION_FOO = 'test_composition::TestFoo'


class TestComponentManagerUT(unittest.TestCase):
    executor: Executor
    component_manager: ComponentManager
    use_multi_threaded_executor: bool = False
    container_name: str = "ut_container"

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

        if cls.use_multi_threaded_executor:
            cls.executor = MultiThreadedExecutor()
        else:
            cls.executor = SingleThreadedExecutor()

        cls.executor = SingleThreadedExecutor()
        cls.component_manager = ComponentManager(cls.executor, cls.container_name)
        cls.executor.add_node(cls.component_manager)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.executor.remove_node(cls.component_manager)
        cls.executor.shutdown()
        rclpy.shutdown()

    def list_nodes(self):
        res = ListNodes.Response()
        req = ListNodes.Request()
        self.__class__.component_manager.on_list_node(req, res)

        return res.unique_ids, res.full_node_names

    def test_gen_unique_id(self):
        current_index = self.component_manager.gen_unique_id()
        assert current_index == 1  # The unique id start from 1

    def test_list_node(self):
        unique_ids, full_node_names = self.list_nodes()
        assert len(full_node_names) == 0
        assert len(unique_ids) == 0

    def test_load_node(self):
        mock_res = LoadNode.Response()
        mock_req = LoadNode.Request()
        mock_req.package_name = TEST_COMPOSITION
        mock_req.plugin_name = TEST_COMPOSITION_FOO
        self.component_manager.on_load_node(mock_req, mock_res)

        print(mock_res.success, mock_res.error_message)

        unique_ids, full_node_names = self.list_nodes()
        assert len(unique_ids) == 1
        assert len(full_node_names) == 1

    def test_unload_node(self):
        mock_res = UnloadNode.Response()
        mock_req = UnloadNode.Request()

        # Unload a non-existing node
        mock_req.unique_id = 0
        self.component_manager.on_unload_node(mock_req, mock_res)
        assert mock_res.error_message
        assert (not mock_res.success)

        # Unload the first node
        ids, node_names = self.list_nodes()
        mock_req.unique_id = ids[0]
        # Don't forget to remove the previous test results
        mock_res = UnloadNode.Response()
        self.component_manager.on_unload_node(mock_req, mock_res)
        assert mock_res.success
        assert (not mock_res.error_message)

        # There should be (n-1) nodes left
        unique_ids, full_node_names = self.list_nodes()
        assert len(unique_ids) == len(ids) - 1
        assert len(full_node_names) == len(node_names) - 1


class TestComponentManagerUTMT(TestComponentManagerUT):
    use_multi_threaded_executor = True
    container_name = 'ut_component_mt'


if __name__ == '__main__':
    unittest.main()
