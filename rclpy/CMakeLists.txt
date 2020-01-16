cmake_minimum_required(VERSION 3.5)

project(rclpy)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

find_package(python_cmake_module REQUIRED)
find_package(PythonExtra MODULE REQUIRED)

set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")

if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

# enables using the Python extensions from the build space for testing
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/test_rclpy/__init__.py" "")

ament_python_install_package(${PROJECT_NAME})

function(set_properties _targetname _build_type)
  set_target_properties(${_targetname} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/test_${PROJECT_NAME}"
    RUNTIME_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/test_${PROJECT_NAME}"
    OUTPUT_NAME "_${_targetname}${PythonExtra_EXTENSION_SUFFIX}"
    SUFFIX "${PythonExtra_EXTENSION_EXTENSION}")
endfunction()

# Only build the library if a C typesupport exists
get_rmw_typesupport(typesupport_impls "rmw_implementation" LANGUAGE "c")
if(typesupport_impls STREQUAL "")
  message(STATUS "Skipping rclpy because no C typesupport library was found.")
  return()
endif()

function(configure_python_c_extension_library _library_name)
  set_properties(${_library_name} "")
  if(WIN32)
    set_properties(${_library_name} "_DEBUG")
    set_properties(${_library_name} "_MINSIZEREL")
    set_properties(${_library_name} "_RELEASE")
    set_properties(${_library_name} "_RELWITHDEBINFO")
  endif()

  target_link_libraries(${_library_name}
    ${PythonExtra_LIBRARIES}
  )

  ament_target_dependencies(${_library_name}
    "PythonExtra"
  )

  install(TARGETS ${_library_name}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endfunction()

add_library(rclpy_common SHARED
  src/rclpy_common/src/common.c
  src/rclpy_common/src/handle.c
)
target_link_libraries(rclpy_common
  ${PythonExtra_LIBRARIES}
)
target_include_directories(rclpy_common
  PUBLIC
  src/rclpy_common/include
)
ament_target_dependencies(rclpy_common
  "rcl"
  "rmw"
  "PythonExtra"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rclpy_common PRIVATE "RCLPY_COMMON_BUILDING_DLL")

install(TARGETS rclpy_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(
  rclpy
  SHARED src/rclpy/_rclpy.c
)
target_link_libraries(rclpy
  rclpy_common
)

configure_python_c_extension_library(rclpy)
ament_target_dependencies(rclpy
  "rcl"
  "rcl_yaml_param_parser"
  "rcutils"
)

add_library(
  rclpy_action
  SHARED src/rclpy/_rclpy_action.c
)
target_link_libraries(rclpy_action
  rclpy_common
)

configure_python_c_extension_library(rclpy_action)
ament_target_dependencies(rclpy_action
  "rcl"
  "rcl_action"
  "rcutils"
)

# Logging support provided by rcutils
add_library(
  rclpy_logging
  SHARED src/rclpy/_rclpy_logging.c
)
configure_python_c_extension_library(rclpy_logging)
ament_target_dependencies(rclpy_logging
  "rcutils"
)

# Signal handling library
add_library(
  rclpy_signal_handler
  SHARED src/rclpy/_rclpy_signal_handler.c
)
target_link_libraries(rclpy_signal_handler
  rclpy_common
)
configure_python_c_extension_library(rclpy_signal_handler)
ament_target_dependencies(rclpy_signal_handler
  "rcl"
  "rcutils"
)

# Pycapsule handling library
add_library(
  rclpy_pycapsule
  SHARED src/rclpy/_rclpy_pycapsule.c
)
configure_python_c_extension_library(rclpy_pycapsule)

# Handle library, used to keep dependencies between C objects.
add_library(
  rclpy_handle
  SHARED src/rclpy/_rclpy_handle.c
)
target_link_libraries(rclpy_handle
  rclpy_common
)
configure_python_c_extension_library(rclpy_handle)
ament_target_dependencies(rclpy_handle
  "rcutils"
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

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

  ament_add_gtest(test_c_handle
    src/test/rclpy_common/test_handle.cpp)
  target_link_libraries(test_c_handle
    rclpy_common)

  if(NOT _typesupport_impls STREQUAL "")
    # Run each test in its own pytest invocation to isolate any global state in rclpy
    set(_rclpy_pytest_tests
      test/test_action_client.py
      test/test_action_graph.py
      test/test_action_server.py
      test/test_callback_group.py
      test/test_client.py
      test/test_clock.py
      test/test_create_node.py
      test/test_create_while_spinning.py
      test/test_destruction.py
      test/test_executor.py
      test/test_expand_topic_name.py
      test/test_guard_condition.py
      test/test_handle.py
      test/test_init_shutdown.py
      test/test_logging.py
      test/test_logging_rosout.py
      test/test_messages.py
      test/test_node.py
      test/test_parameter.py
      test/test_parameters_callback.py
      test/test_qos.py
      test/test_qos_event.py
      test/test_rate.py
      test/test_serialization.py
      test/test_task.py
      test/test_time_source.py
      test/test_time.py
      test/test_timer.py
      test/test_topic_or_service_is_hidden.py
      test/test_topic_endpoint_info.py
      test/test_utilities.py
      test/test_validate_full_topic_name.py
      test/test_validate_namespace.py
      test/test_validate_node_name.py
      test/test_validate_topic_name.py
      test/test_waitable.py
    )

    foreach(_test_path ${_rclpy_pytest_tests})
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(${_test_name} ${_test_path}
        PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
        APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
          PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 120
        WERROR ON
      )
    endforeach()
  endif()
endif()
set(PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")

ament_package()
