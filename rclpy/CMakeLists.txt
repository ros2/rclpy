cmake_minimum_required(VERSION 3.5)

project(rclpy)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
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
find_package(rcl_logging_interface REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

# Find python before pybind11
find_package(python_cmake_module REQUIRED)
find_package(PythonExtra REQUIRED)

set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  # Set the python debug interpreter.
  # pybind11 will setup the build for debug now.
  set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  # pybind11 logic for setting up a debug build when both a debug and release
  # python interpreter are present in the system seems to be pretty much broken.
  # This works around the issue.
  set(PYTHON_LIBRARIES "${PYTHON_DEBUG_LIBRARIES}")
endif()

function(clean_windows_flags target)
  # Hack to avoid pybind11 issue.
  #
  # TODO(ivanpauno):
  # This can be deleted when we update `pybind11_vendor` to a version including
  # https://github.com/pybind/pybind11/pull/2590.
  #
  # They are enabling /LTCG on Windows to reduce binary size,
  # but that doesn't play well with MSVC incremental linking (default for Debug/RelWithDebInfo).
  #
  # See:
  # - https://docs.microsoft.com/en-us/cpp/build/reference/incremental-link-incrementally?view=vs-2019
  # - https://docs.microsoft.com/en-us/cpp/build/reference/ltcg-link-time-code-generation?view=vs-2019

  if(MSVC AND "${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    get_target_property(target_link_libraries ${target} LINK_LIBRARIES)
    list(REMOVE_ITEM target_link_libraries "$<$<NOT:$<CONFIG:Debug>>:-LTCG>")
    set_target_properties(${target} PROPERTIES LINK_LIBRARIES "${target_link_libraries}")

    get_target_property(target_compile_options ${target} COMPILE_OPTIONS)
    list(REMOVE_ITEM target_compile_options "$<$<NOT:$<CONFIG:Debug>>:/GL>")
    set_target_properties(${target} PROPERTIES COMPILE_OPTIONS "${target_compile_options}")
  endif()
endfunction()

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

add_library(rclpy_common SHARED
  src/rclpy_common/src/handle.c
)
target_link_libraries(rclpy_common PUBLIC
  pybind11::pybind11
  ${PYTHON_LIBRARIES}
  rcl::rcl
  rmw::rmw
)
target_include_directories(rclpy_common PUBLIC
  src/rclpy_common/include
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rclpy_common PRIVATE "RCLPY_COMMON_BUILDING_DLL")

install(TARGETS rclpy_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Split from main extension and converted to pybind11
pybind11_add_module(_rclpy_pybind11 SHARED
  src/rclpy/_rclpy_handle.cpp
  src/rclpy/_rclpy_logging.cpp
  src/rclpy/_rclpy_pybind11.cpp
  src/rclpy/_rclpy_pycapsule.cpp
  src/rclpy/action_client.cpp
  src/rclpy/action_goal_handle.cpp
  src/rclpy/action_server.cpp
  src/rclpy/client.cpp
  src/rclpy/clock.cpp
  src/rclpy/context.cpp
  src/rclpy/destroyable.cpp
  src/rclpy/duration.cpp
  src/rclpy/exceptions.cpp
  src/rclpy/graph.cpp
  src/rclpy/guard_condition.cpp
  src/rclpy/logging.cpp
  src/rclpy/names.cpp
  src/rclpy/node.cpp
  src/rclpy/publisher.cpp
  src/rclpy/qos.cpp
  src/rclpy/qos_event.cpp
  src/rclpy/serialization.cpp
  src/rclpy/service.cpp
  src/rclpy/service_info.cpp
  src/rclpy/signal_handler.cpp
  src/rclpy/subscription.cpp
  src/rclpy/time_point.cpp
  src/rclpy/timer.cpp
  src/rclpy/utils.cpp
  src/rclpy/wait_set.cpp
)
target_include_directories(_rclpy_pybind11 PRIVATE
  src/rclpy/
)
target_link_libraries(_rclpy_pybind11 PRIVATE
  rcl::rcl
  rcl_action::rcl_action
  rcl_logging_interface::rcl_logging_interface
  rclpy_common
  rcpputils::rcpputils
  rcutils::rcutils
)
configure_build_install_location(_rclpy_pybind11)
clean_windows_flags(_rclpy_pybind11)

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

  ament_add_gtest(test_c_handle
    src/test/rclpy_common/test_handle.cpp)
  target_link_libraries(test_c_handle
    rclpy_common)

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
      test/test_publisher.py
      test/test_qos.py
      test/test_qos_event.py
      test/test_qos_overriding_options.py
      test/test_rate.py
      test/test_serialization.py
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
