cmake_minimum_required(VERSION 3.5)

project(rclpy C)

if(ANDROID)
  message(STATUS "rclpy does not target Android, skipping...")
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

find_package(python_cmake_module REQUIRED)
find_package(PythonExtra MODULE)

set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
if(NOT WIN32)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

# enables using the Python package from the source space
# This creates a folder in ${CMAKE_CURRENT_BINARY_DIR}/rclpy which has all the
# as well as the compiled modules from the build space
# necessary Python code and C Python extensions for running tests.
configure_file("__init__.py.in" "rclpy/__init__.py" @ONLY)

ament_python_install_package(${PROJECT_NAME})

function(set_properties _targetname _build_type)
  set_target_properties(${_targetname} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
    RUNTIME_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
    OUTPUT_NAME "_${_targetname}${PythonExtra_EXTENSION_SUFFIX}"
    SUFFIX "${PythonExtra_EXTENSION_EXTENSION}")
endfunction()

# Only build the library if a C typesupport exists
get_rmw_typesupport(typesupport_impls "rmw_implementation" LANGUAGE "c")
if(typesupport_impls STREQUAL "")
  message(STATUS "Skipping rclpy because no C typesupport library was found.")
  return()
endif()

add_library(
  ${PROJECT_NAME}
  SHARED src/rclpy/_rclpy.c
)

set_properties(${PROJECT_NAME} "")
if(WIN32)
  set_properties(${PROJECT_NAME} "_DEBUG")
  set_properties(${PROJECT_NAME} "_MINSIZEREL")
  set_properties(${PROJECT_NAME} "_RELEASE")
  set_properties(${PROJECT_NAME} "_RELWITHDEBINFO")
endif()

target_link_libraries(${PROJECT_NAME}
  ${PythonExtra_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  "rcl"
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${PythonExtra_INCLUDE_DIRS}
)

# make an entry in the ament index for each rmw implementation for which
# a rclpy Python extension is built
# this is used in the Python code to avoid trying to import a Python
# extension for rmw implementations which do not have one
ament_index_register_resource("rmw_python_extension"
  PACKAGE_NAME ${rmw_implementation}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)


add_library(
  "${PROJECT_NAME}_implementation_rmw_agnostic"
  SHARED src/rclpy/_rclpy_implementation_rmw_agnostic.c
)
set_properties("${PROJECT_NAME}_implementation_rmw_agnostic" "")
if(WIN32)
  set_properties("${PROJECT_NAME}_implementation_rmw_agnostic" "_DEBUG")
  set_properties("${PROJECT_NAME}_implementation_rmw_agnostic" "_MINSIZEREL")
  set_properties("${PROJECT_NAME}_implementation_rmw_agnostic" "_RELEASE")
  set_properties("${PROJECT_NAME}_implementation_rmw_agnostic" "_RELWITHDEBINFO")
endif()

target_link_libraries("${PROJECT_NAME}_implementation_rmw_agnostic"
  ${rcl_LIBRARIES}
  ${PythonExtra_LIBRARIES}
)

target_include_directories("${PROJECT_NAME}_implementation_rmw_agnostic"
  PUBLIC
  ${rcl_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${PythonExtra_INCLUDE_DIRS}
)

install(TARGETS "${PROJECT_NAME}_implementation_rmw_agnostic"
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  rosidl_generator_py_get_typesupports(_typesupport_impls)
  ament_index_get_prefix_path(ament_index_build_path SKIP_AMENT_PREFIX_PATH)
  # Get the first item (it will be the build space version of the build path).
  list(GET ament_index_build_path 0 ament_index_build_path)
  if(WIN32)
    # On Windows prevent CMake errors and prevent it being evaluated as a list.
    string(REPLACE "\\" "/" ament_index_build_path "${ament_index_build_path}")
  endif()
  if(NOT _typesupport_impls STREQUAL "")
    ament_add_nose_test(rclpytests test
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
      PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
      APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
    )
  endif()
endif()
set(PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")

ament_package()
