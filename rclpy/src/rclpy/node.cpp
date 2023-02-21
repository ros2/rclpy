// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pybind11/pybind11.h>

#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <rcl/graph.h>
#include <rcl/types.h>
#include <rcl_interfaces/msg/parameter_type.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcutils/format_string.h>

#include <limits>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rcpputils/find_and_replace.hpp>
#include <rcpputils/scope_exit.hpp>

#include "exceptions.hpp"
#include "logging.hpp"
#include "node.hpp"
#include "utils.hpp"

namespace rclpy
{
const char *
Node::get_fully_qualified_name()
{
  const char * fully_qualified_node_name = rcl_node_get_fully_qualified_name(rcl_node_.get());
  if (!fully_qualified_node_name) {
    throw RCLError("Fully qualified name not set");
  }

  return fully_qualified_node_name;
}

const char *
Node::logger_name()
{
  const char * node_logger_name = rcl_node_get_logger_name(rcl_node_.get());
  if (!node_logger_name) {
    throw RCLError("Logger name not set");
  }

  return node_logger_name;
}

const char *
Node::get_node_name()
{
  const char * node_name = rcl_node_get_name(rcl_node_.get());
  if (!node_name) {
    throw RCLError("Node name not set");
  }

  return node_name;
}

const char *
Node::get_namespace()
{
  const char * node_namespace = rcl_node_get_namespace(rcl_node_.get());
  if (!node_namespace) {
    throw RCLError("Node namespace not set");
  }

  return node_namespace;
}

size_t
Node::get_count_publishers(const char * topic_name)
{
  size_t count = 0;
  rcl_ret_t ret = rcl_count_publishers(rcl_node_.get(), topic_name, &count);
  if (RCL_RET_OK != ret) {
    throw RCLError("Error in rcl_count_publishers");
  }

  return count;
}

size_t
Node::get_count_subscribers(const char * topic_name)
{
  size_t count = 0;
  rcl_ret_t ret = rcl_count_subscribers(rcl_node_.get(), topic_name, &count);
  if (RCL_RET_OK != ret) {
    throw RCLError("Error in rcl_count_subscribers");
  }

  return count;
}

py::list
Node::get_names_impl(bool get_enclaves)
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcutils_string_array_t node_names = rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t node_namespaces = rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t enclaves = rcutils_get_zero_initialized_string_array();

  rcl_ret_t ret = RCL_RET_OK;
  if (get_enclaves) {
    ret = rcl_get_node_names_with_enclaves(
      rcl_node_.get(), allocator, &node_names, &node_namespaces, &enclaves);
  } else {
    ret = rcl_get_node_names(
      rcl_node_.get(), allocator, &node_names, &node_namespaces);
  }
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to get node names");
  }

  RCPPUTILS_SCOPE_EXIT(
    {
      rcutils_ret_t fini_ret = rcutils_string_array_fini(&node_names);
      if (RCUTILS_RET_OK != fini_ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level,
          "[rclpy| %s : %s ]: failed to fini node names during error handling: %s",
          RCUTILS_STRINGIFY(__FILE__), RCUTILS_STRINGIFY(__LINE__), rcl_get_error_string().str);
        rcl_reset_error();
      }
      fini_ret = rcutils_string_array_fini(&node_namespaces);
      if (RCUTILS_RET_OK != fini_ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level,
          "[rclpy| %s : %s ]: failed to fini node namespaces during error handling: %s",
          RCUTILS_STRINGIFY(__FILE__), RCUTILS_STRINGIFY(__LINE__), rcl_get_error_string().str);
        rcl_reset_error();
      }
      fini_ret = rcutils_string_array_fini(&enclaves);
      if (RCUTILS_RET_OK != fini_ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level,
          "[rclpy| %s : %s ]: failed to fini enclaves string array during error handling: %s",
          RCUTILS_STRINGIFY(__FILE__), RCUTILS_STRINGIFY(__LINE__), rcl_get_error_string().str);
        rcl_reset_error();
      }
    });

  py::list pynode_names_and_namespaces(node_names.size);
  for (size_t idx = 0; idx < node_names.size; ++idx) {
    if (get_enclaves) {
      pynode_names_and_namespaces[idx] = py::make_tuple(
        py::str(node_names.data[idx]),
        py::str(node_namespaces.data[idx]),
        py::str(enclaves.data[idx]));
    } else {
      pynode_names_and_namespaces[idx] = py::make_tuple(
        py::str(node_names.data[idx]),
        py::str(node_namespaces.data[idx]));
    }
  }

  return pynode_names_and_namespaces;
}

py::list
Node::get_node_names_and_namespaces()
{
  return get_names_impl(false);
}

py::list
Node::get_node_names_and_namespaces_with_enclaves()
{
  return get_names_impl(true);
}

/// Create an rclpy.parameter.Parameter from an rcl_variant_t
/**
 * \param[in] pyname name of the parameter
 * \param[in] variant a variant to create a Parameter from
 * \param[in] pyparameter_cls the Parameter class
 * \param[in] pyparameter_type_cls the Parameter.Type class
 * \return an instance of pyparameter_cls
 */
py::object
_parameter_from_rcl_variant(
  py::str pyname, rcl_variant_t * variant, py::object pyparameter_cls,
  py::object pyparameter_type_cls)
{
  int type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_NOT_SET;
  py::object value = py::none();
  if (variant->bool_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_BOOL;
    value = py::bool_(*(variant->bool_value));
  } else if (variant->integer_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER;
    value = py::int_(*(variant->integer_value));
  } else if (variant->double_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE;
    value = py::float_(*(variant->double_value));
  } else if (variant->string_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_STRING;
    value = py::str(variant->string_value);
  } else if (variant->byte_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_BYTE_ARRAY;
    value = py::bytes(
      reinterpret_cast<char *>(variant->byte_array_value->values),
      variant->byte_array_value->size);
  } else if (variant->bool_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_BOOL_ARRAY;
    py::list list_value = py::list(variant->bool_array_value->size);
    for (size_t i = 0; i < variant->bool_array_value->size; ++i) {
      list_value[i] = py::bool_(variant->bool_array_value->values[i]);
    }
    value = list_value;
  } else if (variant->integer_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER_ARRAY;
    py::list list_value = py::list(variant->integer_array_value->size);
    for (size_t i = 0; i < variant->integer_array_value->size; ++i) {
      list_value[i] = py::int_(variant->integer_array_value->values[i]);
    }
    value = list_value;
  } else if (variant->double_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE_ARRAY;
    py::list list_value = py::list(variant->double_array_value->size);
    for (size_t i = 0; i < variant->double_array_value->size; ++i) {
      list_value[i] = py::float_(variant->double_array_value->values[i]);
    }
    value = list_value;
  } else if (variant->string_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_STRING_ARRAY;
    py::list list_value = py::list(variant->string_array_value->size);
    for (size_t i = 0; i < variant->string_array_value->size; ++i) {
      list_value[i] = py::str(variant->string_array_value->data[i]);
    }
    value = list_value;
  }

  py::object type = pyparameter_type_cls(py::int_(type_enum_value));
  return pyparameter_cls(pyname, type, value);
}

/// Populate a dict with parameters by node name
/**
 * \param[in] params the parameters for multiple nodes
 * \param[in] pyparameter_cls the Parameter class
 * \param[in] pyparameter_type_cls the Parameter.Type class
 * \param[in] node_fqn the FQN of node
 * \param[out] pynode_params a dictionary to populate with node names and parameters
 */
void
_populate_node_parameters_from_rcl_params(
  const rcl_params_t * params, py::object pyparameter_cls,
  py::object pyparameter_type_cls, py::dict pynode_params,
  const char * node_fqn)
{
  for (size_t i = 0; i < params->num_nodes; ++i) {
    std::string node_name{params->node_names[i]};
    if (node_name.empty()) {
      throw std::runtime_error("expected node name to have at least one character");
    }

    // Make sure all node names start with '/'
    if ('/' != node_name.front()) {
      node_name.insert(node_name.begin(), '/');
    }

    if (node_fqn) {
      // Update the regular expression ["/*" -> "(/\\w+)" and "/**" -> "(/\\w+)*"]
      std::string regex = rcpputils::find_and_replace(node_name, "/*", "(/\\w+)");
      if (!std::regex_match(node_fqn, std::regex(regex))) {
        // No need to parse the items because the user just care about node_fqn
        continue;
      }

      node_name = node_fqn;
    }

    auto pynode_name = py::str(node_name);

    // make a dictionary for the parameters belonging to this specific node name
    if (!pynode_params.contains(pynode_name)) {
      pynode_params[pynode_name] = py::dict();
    }
    py::dict parameter_dict = pynode_params[pynode_name];

    rcl_node_params_t node_params = params->params[i];
    for (size_t j = 0; j < node_params.num_params; ++j) {
      auto pyparam_name = py::str(node_params.parameter_names[j]);

      parameter_dict[pyparam_name] = _parameter_from_rcl_variant(
        pyparam_name, &node_params.parameter_values[j], pyparameter_cls, pyparameter_type_cls);
    }
  }
}

/// Populate a Python dict with node parameters parsed from CLI arguments
/**
 * \param[in] args CLI arguments to parse for parameters
 * \param[in] pyparameter_cls the Parameter class
 * \param[in] pyparameter_type_cls the Parameter.Type class
 * \param[in] node_fqn the FQN of node
 * \param[out] params_by_node_name A Python dict object to place parsed parameters into.
 */
void
_parse_param_overrides(
  const rcl_arguments_t * args, py::object pyparameter_cls,
  py::object pyparameter_type_cls, py::dict pyparams_by_node_name,
  const char * node_fqn)
{
  rcl_params_t * params = nullptr;
  if (RCL_RET_OK != rcl_arguments_get_param_overrides(args, &params)) {
    throw RCLError("failed to get parameter overrides");
  }
  if (params) {
    RCPPUTILS_SCOPE_EXIT({rcl_yaml_node_struct_fini(params);});
    _populate_node_parameters_from_rcl_params(
      params, pyparameter_cls, pyparameter_type_cls, pyparams_by_node_name, node_fqn);
  }
}

py::dict
Node::get_parameters(py::object pyparameter_cls)
{
  py::dict params_by_node_name;
  py::object parameter_type_cls = pyparameter_cls.attr("Type");

  const rcl_node_options_t * node_options = rcl_node_get_options(rcl_node_.get());

  const char * node_fqn = rcl_node_get_fully_qualified_name(rcl_node_.get());
  if (!node_fqn) {
    throw RCLError("failed to get node fully qualified name");
  }

  if (node_options->use_global_arguments) {
    _parse_param_overrides(
      &(rcl_node_.get()->context->global_arguments), pyparameter_cls,
      parameter_type_cls, params_by_node_name, node_fqn);
  }

  _parse_param_overrides(
    &(node_options->arguments), pyparameter_cls,
    parameter_type_cls, params_by_node_name, node_fqn);


  py::str pynode_fqn(node_fqn);
  py::dict node_params;

  if (params_by_node_name.contains(pynode_fqn)) {
    node_params = params_by_node_name[pynode_fqn];
  }

  return node_params;
}

void Node::destroy()
{
  rcl_node_.reset();
  context_.destroy();
}

Node::Node(
  const char * node_name,
  const char * namespace_,
  Context & context,
  py::object pycli_args,
  bool use_global_arguments,
  bool enable_rosout)
: context_(context)
{
  rcl_ret_t ret;
  rcl_arguments_t arguments = rcl_get_zero_initialized_arguments();

  // turn the arguments into an array of C-style strings
  std::vector<const char *> arg_values;
  const char ** const_arg_values = nullptr;
  py::list pyargs;
  if (!pycli_args.is_none()) {
    pyargs = pycli_args;
    if (!pyargs.empty()) {
      arg_values.resize(pyargs.size());
      for (size_t i = 0; i < pyargs.size(); ++i) {
        // CPython owns const char * memory - no need to free it
        arg_values[i] = PyUnicode_AsUTF8(pyargs[i].ptr());
        if (!arg_values[i]) {
          throw py::error_already_set();
        }
      }
      const_arg_values = &(arg_values[0]);
    }
  }

  // call rcl_parse_arguments() so rcl_arguments_t structure is always valid.
  // Otherwise the remapping functions will error if the user passes no arguments to a node and sets
  // use_global_arguments to False.
  rcl_allocator_t allocator = rcl_get_default_allocator();
  if (arg_values.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw py::value_error("too many cli arguments given to node");
  }
  int num_args = static_cast<int>(arg_values.size());
  ret = rcl_parse_arguments(num_args, const_arg_values, allocator, &arguments);

  if (RCL_RET_INVALID_ROS_ARGS == ret) {
    throw RCLInvalidROSArgsError("Failed to parse ROS arguments");
  }
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to parse arguments");
  }

  RCPPUTILS_SCOPE_EXIT(
    {
      if (RCL_RET_OK != rcl_arguments_fini(&arguments)) {
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini arguments: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
    });

  throw_if_unparsed_ros_args(pyargs, arguments);

  rcl_node_ = std::shared_ptr<rcl_node_t>(
    new rcl_node_t,
    [](rcl_node_t * node)
    {
      rcl_ret_t ret = rcl_node_fini(node);
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini node: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete node;
    });

  *rcl_node_ = rcl_get_zero_initialized_node();
  rcl_node_options_t options = rcl_node_get_default_options();
  options.use_global_arguments = use_global_arguments;
  options.arguments = arguments;
  options.enable_rosout = enable_rosout;

  {
    rclpy::LoggingGuard scoped_logging_guard;
    ret = rcl_node_init(
      rcl_node_.get(), node_name, namespace_, context.rcl_ptr(), &options);
  }

  if (RCL_RET_BAD_ALLOC == ret) {
    rcl_reset_error();
    throw std::bad_alloc();
  }
  if (RCL_RET_NODE_INVALID_NAME == ret) {
    throw py::value_error(append_rcl_error("invalid node name"));
  }
  if (RCL_RET_NODE_INVALID_NAMESPACE == ret) {
    throw py::value_error(append_rcl_error("invalid node namespace"));
  }
  if (RCL_RET_OK != ret) {
    throw RCLError("error creating node");
  }
}

py::list
Node::get_action_client_names_and_types_by_node(
  const char * remote_node_name, const char * remote_node_namespace)
{
  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_client_names_and_types_by_node(
    rcl_node_.get(),
    &allocator,
    remote_node_name,
    remote_node_namespace,
    &names_and_types);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get action client names and type");
  }

  return convert_to_py_names_and_types(&names_and_types);
}

py::list
Node::get_action_server_names_and_types_by_node(
  const char * remote_node_name, const char * remote_node_namespace)
{
  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_server_names_and_types_by_node(
    rcl_node_.get(),
    &allocator,
    remote_node_name,
    remote_node_namespace,
    &names_and_types);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get action server names and type");
  }

  return convert_to_py_names_and_types(&names_and_types);
}

py::list
Node::get_action_names_and_types()
{
  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_action_get_names_and_types(rcl_node_.get(), &allocator, &names_and_types);
  if (RCL_RET_OK != ret) {
    throw rclpy::RCLError("Failed to get action names and type");
  }

  return convert_to_py_names_and_types(&names_and_types);
}

void
define_node(py::object module)
{
  py::class_<Node, Destroyable, std::shared_ptr<Node>>(module, "Node")
  .def(py::init<const char *, const char *, Context &, py::object, bool, bool>())
  .def_property_readonly(
    "pointer", [](const Node & node) {
      return reinterpret_cast<size_t>(node.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "get_fully_qualified_name", &Node::get_fully_qualified_name,
    "Get the fully qualified name of the node.")
  .def(
    "logger_name", &Node::logger_name,
    "Get the name of the logger associated with a node.")
  .def(
    "get_node_name", &Node::get_node_name,
    "Get the name of a node.")
  .def(
    "get_namespace", &Node::get_namespace,
    "Get the namespace of a node.")
  .def(
    "get_count_publishers", &Node::get_count_publishers,
    "Returns the count of all the publishers known for that topic in the entire ROS graph.")
  .def(
    "get_count_subscribers", &Node::get_count_subscribers,
    "Returns the count of all the subscribers known for that topic in the entire ROS graph.")
  .def(
    "get_node_names_and_namespaces", &Node::get_node_names_and_namespaces,
    "Get the list of nodes discovered by the provided node")
  .def(
    "get_node_names_and_namespaces_with_enclaves",
    &Node::get_node_names_and_namespaces_with_enclaves,
    "Get the list of nodes discovered by the provided node, with their respective enclaves.")
  .def(
    "get_action_client_names_and_types_by_node",
    &Node::get_action_client_names_and_types_by_node,
    "Get action client names and types by node.")
  .def(
    "get_action_server_names_and_types_by_node",
    &Node::get_action_server_names_and_types_by_node,
    "Get action server names and types by node.")
  .def(
    "get_action_names_and_types", &Node::get_action_names_and_types,
    "Get action names and types.")
  .def(
    "get_parameters", &Node::get_parameters,
    "Get a list of parameters for the current node");
}
}  // namespace rclpy
