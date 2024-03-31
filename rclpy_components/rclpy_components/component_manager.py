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

from importlib.metadata import entry_points

from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from rclpy.executors import Executor
from rclpy.node import Node


class ComponentManager(Node):

    def __init__(self, executor: Executor, name='py_component_manager', **kwargs):
        super().__init__(name, **kwargs)
        self.executor = executor
        # Implement the 3 services described in
        # http://design.ros2.org/articles/roslaunch.html#command-line-arguments
        self._list_node_srv = self.create_service(
            ListNodes, '~/_container/list_nodes', self.on_list_node)
        self._load_node_srv = self.create_service(
            LoadNode, '~/_container/load_node', self.on_load_node)
        self._unload_node_srv = self.create_service(
            UnloadNode, '~/_container/unload_node', self.on_unload_node)

        self._components = {}  # key: unique_id, value: full node name and component instance
        self._unique_id_index = 0

    def _next_id(self):
        self._unique_id_index += 1
        return self._unique_id_index

    def _does_name_exist(self, fully_qualified_name: str) -> bool:
        """Return true iff there is already a node with the given fully qualified name."""
        if fully_qualified_name == self.get_fully_qualified_name():
            return True
        for name, instance in self._components.values():
            if fully_qualified_name == name:
                return True
        return False

    def on_list_node(self, req: ListNodes.Request, res: ListNodes.Response):
        for key, name_instance in self._components.items():
            res.unique_ids.append(key)
            res.full_node_names.append(name_instance[0])

        return res

    def on_load_node(self, req: LoadNode.Request, res: LoadNode.Response):
        component_entry_points = entry_points().select(group='rclpy_components')
        if not component_entry_points:
            res.success = False
            res.error_message = 'No rclpy components registered'
            return res

        request_ep_name = f'{req.package_name}::{req.plugin_name}'
        component_entry_point = None
        for ep in component_entry_points:
            if ep.name == request_ep_name:
                component_entry_point = ep
                break

        if not component_entry_point:
            res.success = False
            res.error_message = f'Rclpy component not found: {request_ep_name}'
            return res

        try:
            component_class = component_entry_point.load()
        except Exception as e:
            error_message = str(e)
            self.get_logger().error(f'Failed to load entry point: {error_message}')
            res.success = False
            res.error_message = error_message
            return res

        if req.node_name:
            node_name = req.node_name
        else:
            # TODO(sloretz) use remap rule like rclcpp_components
            node_name = str.lower(str.split(component_entry_point.value, ':')[1])

        params_dict = {'use_global_arguments': False, 'context': self.context}
        if req.parameters:
            params_dict['parameter_overrides'] = req.parameters

        if req.node_namespace:
            params_dict['namespace'] = req.node_namespace

        if req.remap_rules:
            params_dict['cli_args'] = ['--ros-args']
            for rule in req.remap_rules:
                params_dict['cli_args'].extend(['-r', rule])

        self.get_logger().info(
                f'Instantiating {component_entry_point.value} with {node_name}, {params_dict}')
        try:
            component = component_class(node_name, **params_dict)
        except Exception as e:
            error_message = str(e)
            self.get_logger().error(f'Failed to instantiate node: {error_message}')
            res.success = False
            res.error_message = error_message
            return res

        res.full_node_name = component.get_fully_qualified_name()
        if self._does_name_exist(res.full_node_name):
            error_message = f'Node with name {res.full_node_name} already exists in this process.'
            self.get_logger().error(f'Failed to load node: {error_message}')
            res.success = False
            res.error_message = error_message
            component.destroy_node()
            return res

        res.unique_id = self._next_id()
        res.success = True
        self._components[res.unique_id] = (res.full_node_name, component)
        self.executor.add_node(component)
        return res

    def on_unload_node(self, req: UnloadNode.Request, res: UnloadNode.Response):
        uid = req.unique_id
        if uid not in self._components:
            res.success = False
            res.error_message = f'No node found with id: {uid}'
            return res

        _, component_instance = self._components.pop(uid)
        self.executor.remove_node(component_instance)
        res.success = True
        return res
