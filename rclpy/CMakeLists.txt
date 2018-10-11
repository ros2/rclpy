cmake_minimum_required(VERSION 3.5)

project(rclpy C)

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

  target_include_directories(${_library_name}
    PUBLIC
    ${PythonExtra_INCLUDE_DIRS}
  )

  install(TARGETS ${_library_name}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endfunction()

add_library(
  rclpy
  SHARED src/rclpy/_rclpy.c
)
configure_python_c_extension_library(rclpy)
ament_target_dependencies(rclpy
  "rcl"
  "rcl_yaml_param_parser"
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

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  rosidl_generator_py_get_typesupports(_typesupport_impls)
  ament_index_get_prefix_path(ament_index_build_path SKIP_AMENT_PREFIX_PATH)
  # Get the first item (it will be the build space version of the build path).
  list(GET ament_index_build_path 0 ament_index_build_path)
  if(WIN32)
    # On Windows prevent CMake errors and prevent it being evaluated as a list.
    string(REPLACE "\\" "/" ament_index_build_path "${ament_index_build_path}")
  endif()
  if(NOT _typesupport_impls STREQUAL "")
    ament_add_pytest_test(rclpytests test
      PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
      APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
        PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 200
    )
  endif()
endif()
set(PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")

ament_package()
