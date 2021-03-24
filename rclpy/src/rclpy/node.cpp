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

// Include pybind11 before rclpy_common/handle.h includes Python.h
#include <pybind11/pybind11.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/graph.h>
#include <rcl/types.h>
#include <rcl_interfaces/msg/parameter_type.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcpputils/scope_exit.hpp>
#include <rcutils/format_string.h>

#include <stdexcept>
#include <string>
#include <utility>

#include "rclpy_common/handle.h"

#include "rclpy_common/exceptions.hpp"

#include "node.hpp"


namespace rclpy
{
const char *
get_node_name(py::capsule pynode)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  const char * node_name = rcl_node_get_name(node);
  if (!node_name) {
    throw RCLError("Node name not set");
  }

  return node_name;
}

const char *
get_node_namespace(py::capsule pynode)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  const char * node_namespace = rcl_node_get_namespace(node);
  if (!node_namespace) {
    throw RCLError("Node namespace not set");
  }

  return node_namespace;
}

size_t
get_count_publishers(py::capsule pynode, const char * topic_name)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  size_t count = 0;
  rcl_ret_t ret = rcl_count_publishers(node, topic_name, &count);
  if (RCL_RET_OK != ret) {
    throw RCLError("Error in rcl_count_publishers");
  }

  return count;
}

size_t
get_count_subscribers(py::capsule pynode, const char * topic_name)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  size_t count = 0;
  rcl_ret_t ret = rcl_count_subscribers(node, topic_name, &count);
  if (RCL_RET_OK != ret) {
    throw RCLError("Error in rcl_count_subscribers");
  }

  return count;
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
 * \param[out] pynode_params a dictionary to populate with node names and parameters
 */
void
_populate_node_parameters_from_rcl_params(
  const rcl_params_t * params, py::object pyparameter_cls,
  py::object pyparameter_type_cls, py::dict pynode_params)
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
 * \param[out] params_by_node_name A Python dict object to place parsed parameters into.
 */
void
_parse_param_overrides(
  const rcl_arguments_t * args, py::object pyparameter_cls,
  py::object pyparameter_type_cls, py::dict pyparams_by_node_name)
{
  rcl_params_t * params = nullptr;
  if (RCL_RET_OK != rcl_arguments_get_param_overrides(args, &params)) {
    throw RCLError("failed to get parameter overrides");
  }
  if (params) {
    RCPPUTILS_SCOPE_EXIT({rcl_yaml_node_struct_fini(params);});
    _populate_node_parameters_from_rcl_params(
      params, pyparameter_cls, pyparameter_type_cls, pyparams_by_node_name);
  }
}

py::dict
get_node_parameters(py::object pyparameter_cls, py::capsule pynode)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  py::dict params_by_node_name;
  py::object parameter_type_cls = pyparameter_cls.attr("Type");

  const rcl_node_options_t * node_options = rcl_node_get_options(node);

  if (node_options->use_global_arguments) {
    _parse_param_overrides(
      &(node->context->global_arguments), pyparameter_cls,
      parameter_type_cls, params_by_node_name);
  }

  _parse_param_overrides(
    &(node_options->arguments), pyparameter_cls,
    parameter_type_cls, params_by_node_name);

  const char * node_fqn = rcl_node_get_fully_qualified_name(node);
  if (!node_fqn) {
    throw RCLError("failed to get node fully qualified name");
  }
  py::str pynode_fqn(node_fqn);
  py::str pywildcard_name("/**");
  py::dict node_params;

  // Enforce wildcard matching precedence
  // TODO(cottsay) implement further wildcard matching
  if (params_by_node_name.contains(pywildcard_name)) {
    node_params = params_by_node_name[pywildcard_name];
  }
  if (params_by_node_name.contains(pynode_fqn)) {
    // TODO(sloretz) py::dict should expose dict.update()
    py::dict node_specific_params = params_by_node_name[pynode_fqn];
    for (const std::pair<py::handle, py::handle> & key_value : node_specific_params) {
      node_params[key_value.first] = key_value.second;
    }
  }

  return node_params;
}
}  // namespace rclpy
