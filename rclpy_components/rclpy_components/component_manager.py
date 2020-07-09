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

from rclpy.node import Node
from rclpy.executors import Executor
from rclpy.logging import get_logger
from rclpy.exceptions import InvalidNodeNameException, InvalidNamespaceException
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
try:
    from importlib.metadata import entry_points
except ImportError:
    from importlib_metadata import entry_points

RCLPY_COMPONENTS = 'rclpy_components'
logger = get_logger('ComponentManager')


class ComponentManager(Node):

    def __init__(self, executor: Executor, name="py_component_manager", *args, **kwargs):
        # TODO Handle the py args equivalent to rclcpp 'NodeOptions'
        super().__init__(name, *args, **kwargs)
        self.executor = executor
        # Implement the 3 services described in
        # http://design.ros2.org/articles/roslaunch.html#command-line-arguments
        self.list_node_srv_ = self.create_service(ListNodes, "~/_container/list_nodes", self.on_list_node)
        self.load_node_srv_ = self.create_service(LoadNode, "~/_container/load_node", self.on_load_node)
        self.unload_node_srv_ = self.create_service(UnloadNode, "~/_container/unload_node", self.on_unload_node)
        # self.unload_node_srv_ = self.create_service(Empty, "~/_container/supported_types", self.on_supported_types)

        self.components = {}  # key: unique_id, value: full node name and component instance
        self.unique_id_index = 0

        self.executor.spin()

    def gen_unique_id(self):
        self.unique_id_index += 1
        return self.unique_id_index

    def on_list_node(self, req: ListNodes.Request, res: ListNodes.Response):
        res.unique_ids = [int(key) for key in self.components.keys()]
        res.full_node_names = [v[0] for v in self.components.values()]

        return res

    def on_load_node(self, req: LoadNode.Request, res: LoadNode.Response):
        component_entry_points = entry_points().get(RCLPY_COMPONENTS, None)
        if not component_entry_points:
            logger.error('No rclpy components registered')
            res.success = False
            return res

        component_entry_point = None
        for ep in component_entry_points:
            if ep.name == req.plugin_name:
                component_entry_point = ep
                break

        if not component_entry_point:
            logger.error('No rclpy component found by %s' % req.plugin_name)
            res.success = False
            return res

        component_class = component_entry_point.load()
        node_name = req.node_name if req.node_name else \
            str.lower(str.split(component_entry_point.value, ':')[1])

        params_dict = {}
        if req.parameters:
            params_dict['parameter_overrides'] = req.parameters

        if req.node_namespace:
            params_dict['namespace'] = req.node_namespace

        if req.remap_rules:
            params_dict['cli_args'] = ['--ros-args']
            for rule in req.remap_rules:
                params_dict['cli_args'].extend(['-r', rule])

        # TODO Handle the priority of req.node_namespace and req.remap_rules '__ns:=<ns>'
        try:
            component = component_class(node_name, **params_dict)

            # TODO Handle the node_name, node_namespace, and remapping rules.
            res.unique_id = self.gen_unique_id()
            res.full_node_name = '/{}'.format(node_name)

            if req.node_namespace:
                res.full_node_name = '/{}{}'.format(req.node_namespace, res.full_node_name)
            self.components[str(res.unique_id)] = (res.full_node_name, component)
            self.executor.add_node(component)
            res.success = True
            return res
        except (InvalidNodeNameException, InvalidNamespaceException, TypeError) as e:
            error_message = str(e)
            logger.error('Failed to load node: %s' % error_message)
            res.success = False
            res.error_message = error_message
            return res

    def on_unload_node(self, req: UnloadNode.Request, res: UnloadNode.Response):
        uid = str(req.unique_id)
        if uid not in self.components:
            res._error_message = 'No node found with unique_id: %s' % uid
            res.success = False
            return res

        _, component_instance = self.components.pop(uid)
        self.executor.remove_node(component_instance)
        res.success = True
        return res

    # def on_supported_types(self, req: SupportedComponentTypes.Request, res: SupportedComponentTypes.Response):
    #     res.supported_types = ['rclpy_components']
    #     return res
