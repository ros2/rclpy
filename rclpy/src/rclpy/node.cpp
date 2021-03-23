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

/// Create an rclpy.parameter.Parameter from an rcl_variant_t
/**
 * On failure a Python exception is raised and NULL is returned if:
 *
 * Raises ValueError if the variant points to no data.
 *
 * \param[in] name The name of the parameter as a Python unicode string.
 * \param[in] variant The variant object to create a Parameter from.
 * \param[in] parameter_cls The PythonObject for the Parameter class.
 * \param[in] parameter_type_cls The PythonObject for the Parameter.Type class.
 *
 * Returns a pointer to an rclpy.parameter.Parameter with the name, type, and value from
 * the variant or NULL when raising a Python exception.
 */
static PyObject * _parameter_from_rcl_variant(
  PyObject * name, rcl_variant_t * variant, PyObject * parameter_cls,
  PyObject * parameter_type_cls)
{
  // Default to NOT_SET and a value of Py_None to suppress warnings.
  // A Python error will raise if type and value don't agree.
  int type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_NOT_SET;
  PyObject * value = Py_None;
  PyObject * member_value;
  if (variant->bool_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_BOOL;
    value = *(variant->bool_value) ? Py_True : Py_False;
    Py_INCREF(value);
  } else if (variant->integer_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER;
    value = PyLong_FromLongLong(*(variant->integer_value));
    if (!value) {
      return NULL;
    }
  } else if (variant->double_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE;
    value = PyFloat_FromDouble(*(variant->double_value));
    if (!value) {
      return NULL;
    }
  } else if (variant->string_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_STRING;
    value = PyUnicode_FromString(variant->string_value);
    if (!value) {
      return NULL;
    }
  } else if (variant->byte_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_BYTE_ARRAY;
    value = PyList_New(variant->byte_array_value->size);
    if (!value) {
      return NULL;
    }
    for (size_t i = 0; i < variant->byte_array_value->size; ++i) {
      member_value = PyBytes_FromFormat("%u", variant->byte_array_value->values[i]);
      if (!member_value) {
        Py_DECREF(value);
        return NULL;
      }
      PyList_SET_ITEM(value, i, member_value);
    }
  } else if (variant->bool_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_BOOL_ARRAY;
    value = PyList_New(variant->bool_array_value->size);
    if (!value) {
      return NULL;
    }
    for (size_t i = 0; i < variant->bool_array_value->size; ++i) {
      member_value = variant->bool_array_value->values[i] ? Py_True : Py_False;
      Py_INCREF(member_value);
      PyList_SET_ITEM(value, i, member_value);
    }
  } else if (variant->integer_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER_ARRAY;
    value = PyList_New(variant->integer_array_value->size);
    if (!value) {
      return NULL;
    }
    for (size_t i = 0; i < variant->integer_array_value->size; ++i) {
      member_value = PyLong_FromLongLong(variant->integer_array_value->values[i]);
      if (!member_value) {
        Py_DECREF(value);
        return NULL;
      }
      PyList_SET_ITEM(value, i, member_value);
    }
  } else if (variant->double_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE_ARRAY;
    value = PyList_New(variant->double_array_value->size);
    if (!value) {
      return NULL;
    }
    for (size_t i = 0; i < variant->double_array_value->size; ++i) {
      member_value = PyFloat_FromDouble(variant->double_array_value->values[i]);
      if (!member_value) {
        Py_DECREF(value);
        return NULL;
      }
      PyList_SET_ITEM(value, i, member_value);
    }
  } else if (variant->string_array_value) {
    type_enum_value = rcl_interfaces__msg__ParameterType__PARAMETER_STRING_ARRAY;
    value = PyList_New(variant->string_array_value->size);
    if (!value) {
      return NULL;
    }
    for (size_t i = 0; i < variant->string_array_value->size; ++i) {
      member_value = PyUnicode_FromString(variant->string_array_value->data[i]);
      if (!member_value) {
        Py_DECREF(value);
        return NULL;
      }
      PyList_SET_ITEM(value, i, member_value);
    }
  } else {
    // INCREF Py_None if no other value was set.
    Py_INCREF(value);
  }

  PyObject * args = Py_BuildValue("(i)", type_enum_value);
  if (!args) {
    Py_DECREF(value);
    return NULL;
  }
  PyObject * type = PyObject_CallObject(parameter_type_cls, args);
  Py_DECREF(args);
  args = Py_BuildValue("OOO", name, type, value);
  Py_DECREF(value);
  Py_DECREF(type);
  if (!args) {
    return NULL;
  }

  PyObject * param = PyObject_CallObject(parameter_cls, args);
  Py_DECREF(args);
  return param;
}

/// Populate a Python dict with a dict of node parameters by node name
/**
 * On failure a Python exception is raised and false is returned
 *
 * \param[in] name The name of the parameter
 * \param[in] variant The variant object to create a Parameter from
 * \param[in] parameter_cls The PythonObject for the Parameter class.
 * \param[in] parameter_type_cls The PythonObject for the Parameter.Type class.
 * \param[out] node_params_dict The PythonObject to populate with node names and parameters.
 *
 * Returns true when parameters are set successfully
 *         false when there was an error during parsing.
 */
void
_populate_node_parameters_from_rcl_params(
  const rcl_params_t * params, py::object parameter_cls,
  py::object parameter_type_cls, py::dict pynode_params)
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

    // Get a dictionary for the parameters belonging to this specific node name
    if (!pynode_params.contains(pynode_name)) {
      pynode_params[pynode_name] = py::dict();
    }
    auto parameter_dict = pynode_params[pynode_name].cast<py::dict>();

    rcl_node_params_t node_params = params->params[i];
    for (size_t ii = 0; ii < node_params.num_params; ++ii) {
      auto pyparam_name = py::str(node_params.parameter_names[ii]);

      auto pyparam = py::reinterpret_steal<py::object>(
        _parameter_from_rcl_variant(
          pyparam_name.ptr(), &node_params.parameter_values[ii], parameter_cls.ptr(),
          parameter_type_cls.ptr()));
      parameter_dict[pyparam_name] = pyparam;
    }
  }
}

/// Populate a Python dict with node parameters parsed from CLI arguments
/**
 * On failure a Python exception is raised and false is returned if:
 *
 * Raises RuntimeError if param_files cannot be extracted from arguments.
 * Raises RuntimeError if yaml files do not parse succesfully.
 *
 * \param[in] args The arguments to parse for parameters
 * \param[in] allocator Allocator to use for allocating and deallocating within the function.
 * \param[in] parameter_cls The PythonObject for the Parameter class.
 * \param[in] parameter_type_cls The PythonObject for the Parameter.Type class.
 * \param[out] params_by_node_name A Python dict object to place parsed parameters into.
 */
void
_parse_param_overrides(
  const rcl_arguments_t * args, py::object parameter_cls,
  py::object parameter_type_cls, py::dict params_by_node_name)
{
  rcl_params_t * params = nullptr;
  if (RCL_RET_OK != rcl_arguments_get_param_overrides(args, &params)) {
    throw RCLError("failed to get parameter overrrides");
  }
  if (params) {
    RCPPUTILS_SCOPE_EXIT({rcl_yaml_node_struct_fini(params);});
    _populate_node_parameters_from_rcl_params(
      params, parameter_cls, parameter_type_cls, params_by_node_name);
  }
}

/// Get a list of parameters for the current node from rcl_yaml_param_parser
/**
 * On failure, an exception is raised and NULL is returned if:
 *
 * Raises ValueError if the argument is not a node handle.
 * Raises RCLError if the parameters file fails to parse
 *
 * \param[in] parameter_cls The rclpy.parameter.Parameter class object.
 * \param[in] node_capsule Capsule pointing to the node handle
 * \return A dict mapping parameter names to rclpy.parameter.Parameter (may be empty).
 */
py::dict
get_node_parameters(py::object parameter_cls, py::capsule pynode)
{
  auto node = static_cast<rcl_node_t *>(
    rclpy_handle_get_pointer_from_capsule(pynode.ptr(), "rcl_node_t"));
  if (!node) {
    throw py::error_already_set();
  }

  py::dict params_by_node_name;
  py::object parameter_type_cls = parameter_cls.attr("Type");

  const rcl_node_options_t * node_options = rcl_node_get_options(node);

  if (node_options->use_global_arguments) {
    _parse_param_overrides(
      &(node->context->global_arguments), parameter_cls,
      parameter_type_cls, params_by_node_name);
  }

  _parse_param_overrides(
    &(node_options->arguments), parameter_cls,
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
