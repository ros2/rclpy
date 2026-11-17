// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <rcl_yaml_param_parser/parser.h>
#include <rcl_yaml_param_parser/types.h>

#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>

#include <cstdio>
#include <string>

#include "exceptions.hpp"
#include "yaml_param_parser.hpp"

namespace rclpy
{
namespace
{
/// Convert one parsed parameter value to the matching Python object.
/**
 * The value types follow rcl_variant_t: exactly one member is set, except for
 * an empty sequence or an empty map, which leave every member unset and
 * convert to None so the caller can skip the entry.
 *
 * \\param[in] variant the parsed parameter value
 * \\return the Python bool, int, float, str, bytes or list for the value,
 *   or None if the variant holds no value
 */
py::object
convert_variant_to_py(const rcl_variant_t * variant)
{
  if (nullptr != variant->bool_value) {
    return py::bool_(*variant->bool_value);
  }
  if (nullptr != variant->integer_value) {
    return py::int_(*variant->integer_value);
  }
  if (nullptr != variant->double_value) {
    return py::float_(*variant->double_value);
  }
  if (nullptr != variant->string_value) {
    return py::str(variant->string_value);
  }
  if (nullptr != variant->byte_array_value) {
    return py::bytes(
      reinterpret_cast<const char *>(variant->byte_array_value->values),
      variant->byte_array_value->size);
  }
  if (nullptr != variant->bool_array_value) {
    py::list list(variant->bool_array_value->size);
    for (size_t i = 0; i < variant->bool_array_value->size; ++i) {
      list[i] = py::bool_(variant->bool_array_value->values[i]);
    }
    return list;
  }
  if (nullptr != variant->integer_array_value) {
    py::list list(variant->integer_array_value->size);
    for (size_t i = 0; i < variant->integer_array_value->size; ++i) {
      list[i] = py::int_(variant->integer_array_value->values[i]);
    }
    return list;
  }
  if (nullptr != variant->double_array_value) {
    py::list list(variant->double_array_value->size);
    for (size_t i = 0; i < variant->double_array_value->size; ++i) {
      list[i] = py::float_(variant->double_array_value->values[i]);
    }
    return list;
  }
  if (nullptr != variant->string_array_value) {
    py::list list(variant->string_array_value->size);
    for (size_t i = 0; i < variant->string_array_value->size; ++i) {
      list[i] = py::str(variant->string_array_value->data[i]);
    }
    return list;
  }
  // An empty sequence (e.g. `foo: []`) or an empty map (e.g. `foo: {}`)
  // leaves every member unset; the caller skips such entries.
  return py::none();
}

/// Parse a YAML parameter file with rcl_parse_yaml_file().
/**
 * This is the same parser rclcpp uses for `--params-file`, so parameter
 * types match across client libraries. In particular, numbers in exponent
 * notation (e.g. `5e-06`, `1.0e+20`) parse as doubles here, where PyYAML
 * 1.1 would have resolved them as strings.
 *
 * Raises OSError (FileNotFoundError, PermissionError, ...) if the file
 * cannot be opened, and RCUtilsError if the file fails to parse.
 *
 * \\param[in] file_path path to the YAML parameter file
 * \\return dict mapping node names to dicts of parameter names to values
 */
py::dict
parse_yaml_param_file(const char * file_path)
{
  // Open the file first so a missing or unreadable file raises the same
  // OSError a plain open() would, instead of a generic parse error.
  FILE * file = fopen(file_path, "r");
  if (nullptr == file) {
    PyErr_SetFromErrnoWithFilename(PyExc_OSError, file_path);
    throw py::error_already_set();
  }
  fclose(file);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t * params = rcl_yaml_node_struct_init(allocator);
  if (nullptr == params) {
    throw RCUtilsError(append_rcutils_error("Failed to allocate parameter structure"));
  }

  if (!rcl_parse_yaml_file(file_path, params)) {
    std::string error = append_rcutils_error(
      "Failed to parse YAML parameter file '" + std::string(file_path) + "'");
    rcl_yaml_node_struct_fini(params);
    throw RCUtilsError(error);
  }

  py::dict result;
  for (size_t i = 0; i < params->num_nodes; ++i) {
    py::dict node_params;
    for (size_t j = 0; j < params->params[i].num_params; ++j) {
      py::object value = convert_variant_to_py(&params->params[i].parameter_values[j]);
      // Skip entries with no value (empty sequences and empty maps); they
      // do not describe a parameter.
      if (!value.is_none()) {
        node_params[params->params[i].parameter_names[j]] = value;
      }
    }
    result[params->node_names[i]] = node_params;
  }
  rcl_yaml_node_struct_fini(params);
  return result;
}
}  // namespace

void
define_yaml_param_parser(py::module module)
{
  module.def(
    "rclpy_parse_yaml_param_file", &parse_yaml_param_file,
    "Parse a YAML parameter file with rcl_parse_yaml_file().");
}
}  // namespace rclpy
