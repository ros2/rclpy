cmake_minimum_required(VERSION 3.5)

project(rclpy)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Figure out Python3 debug/release before anything else can find_package it
if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)

  # Force FindPython3 to use the debug interpreter where ROS 2 expects it
  set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rcl_logging_interface REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rosidl_runtime_c REQUIRED)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# enables using the Python extensions from the build space for testing
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/test_rclpy/__init__.py" "")

ament_python_install_package(${PROJECT_NAME})

# Only build the library if a C typesupport exists
get_rmw_typesupport(typesupport_impls "rmw_implementation" LANGUAGE "c")
if(typesupport_impls STREQUAL "")
  message(STATUS "Skipping rclpy because no C typesupport library was found.")
  return()
endif()

# Set the build location and install location for a CPython extension
function(configure_build_install_location _library_name)
  # Install into test_rclpy folder in build space for unit tests to import
  set_target_properties(${_library_name} PROPERTIES
    # Use generator expression to avoid prepending a build type specific directory on Windows
    LIBRARY_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/test_rclpy>
    RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/test_rclpy>)

  # Install library for actual use
  install(TARGETS ${_library_name}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endfunction()

# Split from main extension and converted to pybind11
pybind11_add_module(_rclpy_pybind11 SHARED
  src/rclpy/_rclpy_logging.cpp
  src/rclpy/_rclpy_pybind11.cpp
  src/rclpy/action_client.cpp
  src/rclpy/action_goal_handle.cpp
  src/rclpy/action_server.cpp
  src/rclpy/client.cpp
  src/rclpy/clock.cpp
  src/rclpy/context.cpp
  src/rclpy/destroyable.cpp
  src/rclpy/duration.cpp
  src/rclpy/clock_event.cpp
  src/rclpy/exceptions.cpp
  src/rclpy/graph.cpp
  src/rclpy/guard_condition.cpp
  src/rclpy/lifecycle.cpp
  src/rclpy/logging.cpp
  src/rclpy/names.cpp
  src/rclpy/node.cpp
  src/rclpy/publisher.cpp
  src/rclpy/qos.cpp
  src/rclpy/qos_event.cpp
  src/rclpy/serialization.cpp
  src/rclpy/service.cpp
  src/rclpy/service_info.cpp
  src/rclpy/service_introspection.cpp
  src/rclpy/signal_handler.cpp
  src/rclpy/subscription.cpp
  src/rclpy/time_point.cpp
  src/rclpy/timer.cpp
  src/rclpy/utils.cpp
  src/rclpy/wait_set.cpp
)

if(CMAKE_C_COMPILER_ID MATCHES "Clang")
  target_link_libraries(_rclpy_pybind11 PRIVATE atomic)
endif()

target_include_directories(_rclpy_pybind11 PRIVATE
  src/rclpy/
)
target_link_libraries(_rclpy_pybind11 PRIVATE
  lifecycle_msgs::lifecycle_msgs__rosidl_generator_c
  lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_c
  rcl::rcl
  rcl_action::rcl_action
  rcl_lifecycle::rcl_lifecycle
  rcl_logging_interface::rcl_logging_interface
  rcpputils::rcpputils
  rcutils::rcutils
  rosidl_runtime_c::rosidl_runtime_c
)
configure_build_install_location(_rclpy_pybind11)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Give cppcheck hints about macro definitions coming from outside this package
  get_target_property(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS rcutils::rcutils INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE "ament_cmake_cppcheck")
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()
  set_tests_properties(cppcheck PROPERTIES TIMEOUT 420)

  find_package(ament_cmake_pytest REQUIRED)
  find_package(rosidl_generator_py REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  rosidl_generator_py_get_typesupports(_typesupport_impls)
  ament_index_get_prefix_path(ament_index_build_path SKIP_AMENT_PREFIX_PATH)
  # Get the first item (it will be the build space version of the build path).
  list(GET ament_index_build_path 0 ament_index_build_path)
  if(WIN32)
    # On Windows prevent CMake errors and prevent it being evaluated as a list.
    string(REPLACE "\\" "/" ament_index_build_path "${ament_index_build_path}")
  endif()

  ament_add_gtest(test_python_allocator
    test/test_python_allocator.cpp)
  target_include_directories(test_python_allocator PRIVATE src/rclpy)
  target_link_libraries(test_python_allocator pybind11::embed)

  if(NOT _typesupport_impls STREQUAL "")
    # Run each test in its own pytest invocation to isolate any global state in rclpy
    set(_rclpy_pytest_tests
      test/test_action_client.py
      test/test_action_graph.py
      test/test_action_server.py
      test/test_callback_group.py
      test/test_client.py
      test/test_clock.py
      test/test_context.py
      test/test_create_node.py
      test/test_create_while_spinning.py
      test/test_destruction.py
      test/test_executor.py
      test/test_expand_topic_name.py
      test/test_guard_condition.py
      test/test_init_shutdown.py
      test/test_logging.py
      test/test_logging_rosout.py
      test/test_messages.py
      test/test_node.py
      test/test_parameter.py
      test/test_parameter_client.py
      test/test_parameter_service.py
      test/test_publisher.py
      test/test_qos.py
      test/test_qos_event.py
      test/test_qos_overriding_options.py
      test/test_rate.py
      test/test_rosout_subscription.py
      test/test_serialization.py
      test/test_service_introspection.py
      test/test_subscription.py
      test/test_task.py
      test/test_time_source.py
      test/test_time.py
      test/test_timer.py
      test/test_topic_or_service_is_hidden.py
      test/test_topic_endpoint_info.py
      test/test_type_support.py
      test/test_utilities.py
      test/test_validate_full_topic_name.py
      test/test_validate_namespace.py
      test/test_validate_node_name.py
      test/test_validate_topic_name.py
      test/test_waitable.py
      test/test_wait_for_message.py
    )

    foreach(_test_path ${_rclpy_pytest_tests})
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(${_test_name} ${_test_path}
        APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
          PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 120
        WERROR ON
      )
    endforeach()
  endif()
endif()

ament_package()
