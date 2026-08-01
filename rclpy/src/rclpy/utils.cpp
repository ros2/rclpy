// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <rcl/allocator.h>
#include <rcl/arguments.h>
#include <rcl/error_handling.h>
#include <rcl/graph.h>
#include <rcl/publisher.h>
#include <rcl_action/rcl_action.h>
#include <rmw/rmw.h>
#include <rmw/time.h>
#include <rmw/topic_endpoint_info.h>
#include <rmw/types.h>

#include <cassert>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rcpputils/scope_exit.hpp>

#include "exceptions.hpp"
#include "utils.hpp"

namespace rclpy
{

nb::list
convert_to_py_names_and_types(const rcl_names_and_types_t * names_and_types)
{
  assert(names_and_types);

  nb::list py_names_and_types = create_sized_list(names_and_types->names.size);
  for (size_t i = 0u; i < names_and_types->names.size; ++i) {
    nb::list py_types = create_sized_list(names_and_types->types[i].size);
    for (size_t j = 0u; j < names_and_types->types[i].size; ++j) {
      py_types[j] = nb::str(names_and_types->types[i].data[j]);
    }
    py_names_and_types[i] = nb::make_tuple(
      nb::str(names_and_types->names.data[i]), py_types);
  }
  return py_names_and_types;
}

void *
common_get_type_support(nb::object pymessage)
{
  nb::object pymetaclass = pymessage.attr("__class__");

  nb::object value = pymetaclass.attr("_TYPE_SUPPORT");
  void * capsule_ptr = nb::cast<nb::capsule>(value).data();

  return capsule_ptr;
}

std::unique_ptr<void, destroy_ros_message_function *>
create_from_py(nb::object pymessage)
{
  typedef void * create_ros_message_function (void);

  nb::object pymetaclass = pymessage.attr("__class__");

  nb::object value = pymetaclass.attr("_CREATE_ROS_MESSAGE");
  void * capsule_ptr = nb::cast<nb::capsule>(value).data();
  auto create_ros_message =
    reinterpret_cast<create_ros_message_function *>(capsule_ptr);
  if (!create_ros_message) {
    throw nb::python_error();
  }

  value = pymetaclass.attr("_DESTROY_ROS_MESSAGE");
  capsule_ptr = nb::cast<nb::capsule>(value).data();
  auto destroy_ros_message =
    reinterpret_cast<destroy_ros_message_function *>(capsule_ptr);
  if (!destroy_ros_message) {
    throw nb::python_error();
  }

  void * message = create_ros_message();
  if (!message) {
    throw std::bad_alloc();
  }
  return std::unique_ptr<
    void, destroy_ros_message_function *>(message, destroy_ros_message);
}

std::unique_ptr<void, destroy_ros_message_function *>
convert_from_py(nb::object pymessage)
{
  typedef bool convert_from_py_signature (PyObject *, void *);

  std::unique_ptr<void, destroy_ros_message_function *> message =
    create_from_py(pymessage);

  nb::object pymetaclass = pymessage.attr("__class__");

  void * capsule_ptr =
    nb::cast<nb::capsule>(pymetaclass.attr("_CONVERT_FROM_PY")).data();
  auto convert =
    reinterpret_cast<convert_from_py_signature *>(capsule_ptr);
  if (!convert) {
    throw nb::python_error();
  }

  if (!convert(pymessage.ptr(), message.get())) {
    throw nb::python_error();
  }

  return message;
}

nb::object
convert_to_py(void * message, nb::object pyclass)
{
  nb::object pymetaclass = pyclass.attr("__class__");

  void * capsule_ptr =
    nb::cast<nb::capsule>(pymetaclass.attr("_CONVERT_TO_PY")).data();

  typedef PyObject * convert_to_py_function (void *);
  auto convert = reinterpret_cast<convert_to_py_function *>(capsule_ptr);
  if (!convert) {
    throw nb::python_error();
  }
  return nb::steal(convert(message));
}

const char *
get_rmw_implementation_identifier()
{
  return rmw_get_implementation_identifier();
}

void
assert_liveliness(rclpy::Publisher * publisher)
{
  if (RCL_RET_OK != rcl_publisher_assert_liveliness(publisher->rcl_ptr())) {
    throw RCLError("Failed to assert liveliness on the Publisher");
  }
}

nb::list
remove_ros_args(nb::object pycli_args)
{
  rcl_ret_t ret;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_arguments_t parsed_args = rcl_get_zero_initialized_arguments();

  std::vector<const char *> arg_values;
  const char ** const_arg_values = NULL;
  nb::list pyargs;
  if (!pycli_args.is_none()) {
    pyargs = nb::cast<nb::list>(pycli_args);
    if (!pyargs.empty()) {
      arg_values.resize(pyargs.size());
      for (size_t i = 0; i < pyargs.size(); ++i) {
        // CPython owns const char * memory - no need to free it
        arg_values[i] = PyUnicode_AsUTF8(pyargs[i].ptr());
        if (!arg_values[i]) {
          throw nb::python_error();
        }
      }
      const_arg_values = &(arg_values[0]);
    }
  }

  if (arg_values.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw nb::value_error("too many cli arguments");
  }

  int num_args = static_cast<int>(arg_values.size());
  ret = rcl_parse_arguments(num_args, const_arg_values, allocator, &parsed_args);

  if (RCL_RET_INVALID_ROS_ARGS == ret) {
    throw RCLInvalidROSArgsError("Failed to parse ROS arguments");
  }
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to parse arguments");
  }

  RCPPUTILS_SCOPE_EXIT(
    {
      ret = rcl_arguments_fini(&parsed_args);
      if (RCL_RET_OK != ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
          "rcl_arguments_fini failed: ");
        RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
        RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        rcl_reset_error();
      }
    });

  throw_if_unparsed_ros_args(pyargs, parsed_args);

  int nonros_argc = 0;
  const char ** nonros_argv = NULL;

  ret = rcl_remove_ros_arguments(
    const_arg_values,
    &parsed_args,
    allocator,
    &nonros_argc,
    &nonros_argv);
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed rcl_remove_ros_arguments");
  }

/* it was determined that the following warning is likely a front-end parsing issue in MSVC.
 * See: https://github.com/ros2/rclpy/pull/180#issuecomment-375452757
 */
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4090)
#endif
  RCPPUTILS_SCOPE_EXIT(
    {
      allocator.deallocate(nonros_argv, allocator.state);
    });
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

  nb::list result_args = create_sized_list(nonros_argc);
  for (int i = 0; i < nonros_argc; ++i) {
    result_args[i] = nonros_argv[i];
  }

  return result_args;
}

void
throw_if_unparsed_ros_args(nb::list pyargs, const rcl_arguments_t & rcl_args)
{
  int unparsed_ros_args_count = rcl_arguments_get_count_unparsed_ros(&rcl_args);

  if (unparsed_ros_args_count < 0) {
    throw std::runtime_error("failed to count unparsed arguments");
  }
  if (0 == unparsed_ros_args_count) {
    return;
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();

  int * unparsed_indices_c = nullptr;
  rcl_ret_t ret = rcl_arguments_get_unparsed_ros(&rcl_args, allocator, &unparsed_indices_c);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to get unparsed arguments");
  }

  RCPPUTILS_SCOPE_EXIT(allocator.deallocate(unparsed_indices_c, allocator.state));

  nb::list unparsed_args;
  for (int i = 0; i < unparsed_ros_args_count; ++i) {
    int index = unparsed_indices_c[i];
    if (index < 0 || static_cast<size_t>(index) >= pyargs.size()) {
      throw std::runtime_error("got invalid unparsed ROS arg index");
    }
    unparsed_args.append(pyargs[index]);
  }

  throw UnknownROSArgsError(nb::repr(unparsed_args).c_str());
}

nb::dict
rclpy_action_get_rmw_qos_profile(const char * rmw_profile)
{
  nb::dict pyqos_profile;
  if (0 == strcmp(rmw_profile, "rcl_action_qos_profile_status_default")) {
    pyqos_profile = convert_to_qos_dict(&rcl_action_qos_profile_status_default);
  } else {
    std::string error_text = "Requested unknown rmw_qos_profile: ";
    error_text += rmw_profile;
    throw std::runtime_error(error_text);
  }
  return pyqos_profile;
}

nb::dict
_convert_to_py_topic_endpoint_info(const rmw_topic_endpoint_info_t * topic_endpoint_info)
{
  nb::list py_endpoint_gid = create_sized_list(RMW_GID_STORAGE_SIZE);
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; i++) {
    py_endpoint_gid[i] = nb::int_(topic_endpoint_info->endpoint_gid[i]);
  }

  // Create dictionary that represents rmw_topic_endpoint_info_t
  nb::dict py_endpoint_info_dict;
  // Populate keyword arguments
  // A success returns 0, and a failure returns -1
  py_endpoint_info_dict["node_name"] = nb::str(topic_endpoint_info->node_name);
  py_endpoint_info_dict["node_namespace"] = nb::str(topic_endpoint_info->node_namespace);
  py_endpoint_info_dict["topic_type"] = nb::str(topic_endpoint_info->topic_type);
  py_endpoint_info_dict["topic_type_hash"] =
    convert_to_type_hash_dict(&topic_endpoint_info->topic_type_hash);
  py_endpoint_info_dict["endpoint_type"] =
    nb::int_(static_cast<int>(topic_endpoint_info->endpoint_type));
  py_endpoint_info_dict["endpoint_gid"] = py_endpoint_gid;
  py_endpoint_info_dict["qos_profile"] =
    convert_to_qos_dict(&topic_endpoint_info->qos_profile);

  return py_endpoint_info_dict;
}

nb::list
convert_to_py_topic_endpoint_info_list(const rmw_topic_endpoint_info_array_t * info_array)
{
  if (!info_array) {
    throw std::runtime_error("rmw_topic_endpoint_info_array_t pointer is empty");
  }

  nb::list py_info_array = create_sized_list(info_array->size);

  for (size_t i = 0; i < info_array->size; ++i) {
    rmw_topic_endpoint_info_t topic_endpoint_info = info_array->info_array[i];
    // add this dict to the list
    py_info_array[i] = _convert_to_py_topic_endpoint_info(&topic_endpoint_info);
  }
  return py_info_array;
}

nb::dict
_convert_to_py_service_endpoint_info(const rmw_service_endpoint_info_t * service_endpoint_info)
{
  nb::list py_endpoint_gids;
  for(size_t c = 0; c < service_endpoint_info->endpoint_count; c++) {
    nb::list py_endpoint_gid = create_sized_list(RMW_GID_STORAGE_SIZE);
    for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; i++) {
      py_endpoint_gid[i] = nb::int_(service_endpoint_info->endpoint_gids[c][i]);
    }
    py_endpoint_gids.append(py_endpoint_gid);
  }
  // Create dictionary that represents rmw_service_endpoint_info_t
  nb::dict py_endpoint_info_dict;
  // Populate keyword arguments
  // A success returns 0, and a failure returns -1
  py_endpoint_info_dict["node_name"] = nb::str(service_endpoint_info->node_name);
  py_endpoint_info_dict["node_namespace"] = nb::str(service_endpoint_info->node_namespace);
  py_endpoint_info_dict["service_type"] = nb::str(service_endpoint_info->service_type);
  py_endpoint_info_dict["service_type_hash"] =
    convert_to_type_hash_dict(&service_endpoint_info->service_type_hash);
  py_endpoint_info_dict["qos_profiles"] = nb::list();
  py_endpoint_info_dict["endpoint_gids"] = nb::list();

  nb::list qos_profiles_list;
  // Append values to the lists
  for (size_t i = 0; i < service_endpoint_info->endpoint_count; i++) {
    qos_profiles_list.append(
      convert_to_qos_dict(&service_endpoint_info->qos_profiles[i])
    );
  }
  py_endpoint_info_dict["qos_profiles"] = qos_profiles_list;
  py_endpoint_info_dict["endpoint_gids"] = py_endpoint_gids;
  py_endpoint_info_dict["endpoint_type"] =
    nb::int_(static_cast<int>(service_endpoint_info->endpoint_type));
  py_endpoint_info_dict["endpoint_count"] = nb::int_(service_endpoint_info->endpoint_count);

  return py_endpoint_info_dict;
}

nb::list
convert_to_py_service_endpoint_info_list(const rmw_service_endpoint_info_array_t * info_array)
{
  if (!info_array) {
    throw std::runtime_error("rmw_service_endpoint_info_array_t pointer is empty");
  }

  nb::list py_info_array = create_sized_list(info_array->size);

  for (size_t i = 0; i < info_array->size; ++i) {
    rmw_service_endpoint_info_t service_endpoint_info = info_array->info_array[i];
    // add this dict to the list
    py_info_array[i] = _convert_to_py_service_endpoint_info(&service_endpoint_info);
  }
  return py_info_array;
}

nb::object
_convert_to_py_action_endpoint_info(const rcl_action_endpoint_info_t * action_endpoint_info)
{
  // Create dictionary that represents the aggregated endpoint information of
  // all the underlying entities of one action client or one action server.
  // Sub-entities that have not been discovered are represented as None.
  nb::dict py_endpoint_info_dict;
  py_endpoint_info_dict["goal_service_info"] =
    action_endpoint_info->goal_service_info.node_name ?
    nb::object(_convert_to_py_service_endpoint_info(&action_endpoint_info->goal_service_info)) :
    nb::object(nb::none());
  py_endpoint_info_dict["cancel_service_info"] =
    action_endpoint_info->cancel_service_info.node_name ?
    nb::object(_convert_to_py_service_endpoint_info(&action_endpoint_info->cancel_service_info)) :
    nb::object(nb::none());
  py_endpoint_info_dict["result_service_info"] =
    action_endpoint_info->result_service_info.node_name ?
    nb::object(_convert_to_py_service_endpoint_info(&action_endpoint_info->result_service_info)) :
    nb::object(nb::none());
  py_endpoint_info_dict["feedback_topic_info"] =
    action_endpoint_info->feedback_topic_info.node_name ?
    nb::object(_convert_to_py_topic_endpoint_info(&action_endpoint_info->feedback_topic_info)) :
    nb::object(nb::none());
  py_endpoint_info_dict["status_topic_info"] =
    action_endpoint_info->status_topic_info.node_name ?
    nb::object(_convert_to_py_topic_endpoint_info(&action_endpoint_info->status_topic_info)) :
    nb::object(nb::none());

  return py_endpoint_info_dict;
}

nb::list
convert_to_py_action_endpoint_info_list(const rcl_action_endpoint_info_array_t * info_array)
{
  if (!info_array) {
    throw std::runtime_error("rcl_action_endpoint_info_array_t pointer is empty");
  }

  nb::list py_info_array = create_sized_list(info_array->size);

  for (size_t i = 0; i < info_array->size; ++i) {
    // add this dict to the list
    py_info_array[i] = _convert_to_py_action_endpoint_info(&info_array->info_array[i]);
  }
  return py_info_array;
}

static
nb::object
_convert_rmw_time_to_py_duration(const rmw_time_t * duration)
{
  auto pyduration_module = nb::module_::import_("rclpy.duration");
  nb::object pymetaclass = pyduration_module.attr("Duration");
  // to bring in the `_a` literal
  using nb::literals::operator""_a;
  return pymetaclass("seconds"_a = duration->sec, "nanoseconds"_a = duration->nsec);
}

nb::dict
convert_to_qos_dict(const rmw_qos_profile_t * qos_profile)
{
  // Create dictionary and populate arguments with QoSProfile object
  nb::dict pyqos_kwargs;

  pyqos_kwargs["depth"] = nb::int_(qos_profile->depth);
  pyqos_kwargs["history"] = nb::int_(static_cast<int>(qos_profile->history));
  pyqos_kwargs["reliability"] = nb::int_(static_cast<int>(qos_profile->reliability));
  pyqos_kwargs["durability"] = nb::int_(static_cast<int>(qos_profile->durability));
  pyqos_kwargs["lifespan"] = _convert_rmw_time_to_py_duration(&qos_profile->lifespan);
  pyqos_kwargs["deadline"] = _convert_rmw_time_to_py_duration(&qos_profile->deadline);
  pyqos_kwargs["liveliness"] = nb::int_(static_cast<int>(qos_profile->liveliness));
  pyqos_kwargs["liveliness_lease_duration"] = _convert_rmw_time_to_py_duration(
    &qos_profile->liveliness_lease_duration);
  pyqos_kwargs["avoid_ros_namespace_conventions"] =
    nb::bool_(qos_profile->avoid_ros_namespace_conventions);

  return pyqos_kwargs;
}

nb::dict
convert_to_type_hash_dict(const rosidl_type_hash_t * type_hash)
{
  // Create dictionary and populate arguments with type hash object
  nb::dict type_hash_kwargs;

  type_hash_kwargs["version"] = nb::int_(type_hash->version);
  type_hash_kwargs["value"] = nb::bytes(
    reinterpret_cast<const char *>(type_hash->value),
    ROSIDL_TYPE_HASH_SIZE);

  return type_hash_kwargs;
}

void
warn_fini_failure(const char * entity_name)
{
  // Warning should use line number of the current stack frame
  int stack_level = 1;
  PyErr_WarnFormat(
    PyExc_RuntimeWarning, stack_level, "Failed to fini %s: %s",
    entity_name, rcl_get_error_string().str);
  rcl_reset_error();
}
}  // namespace rclpy
